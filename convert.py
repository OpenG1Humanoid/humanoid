#!/usr/bin/env python3
"""Export every printable part from the OpenSCAD files to named STLs, using the openscad command line.

    python export_stls.py                      # everything -> STL_export/<group>/<name>.stl
    python export_stls.py --list               # show every part and its file name, export nothing
    python export_stls.py --only head torso    # some groups (arms, hardware, head, torso, legs)
    python export_stls.py --out my_folder -j 8 # output folder, parallel jobs
    python export_stls.py --openscad "C:/Program Files/OpenSCAD/openscad.exe"

It does exactly what you would do by hand (open the .scad, set PART/part/side, F6, Export STL),
once per part. The results go to a separate folder, not STL/, because several .scad files
import STLs from STL/ and must not read a half-written file.
"""
import argparse
import os
import shutil
import subprocess
import sys
import struct
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SCAD = ROOT / "OpenScad"


BACKEND = []


def s(v):                       # an OpenSCAD string literal for -D
    return f'"{v}"'


def jobs():
    """(group, output name, .scad file, {variable: value})"""
    J = []
    # arms + servo torso: one .scad per part, the part is already set inside the file
    for folder, group in (("parts_left", "arms"), ("parts_right", "arms"), ("parts_torso", "arms"),
                          ("hardware", "hardware")):
        for f in sorted((SCAD / folder).glob("*.scad")):
            J.append((group, f.stem, f, {}))
    # dome head
    head = SCAD / "head" / "head.scad"
    J.append(("head", "head_frame", head, {"PART": s("frame")}))
    for i in range(5):
        J.append(("head", f"head_panel_{i}", head, {"PART": s("panel"), "PANEL": i}))
    J.append(("head", "head_brow_left", head, {"PART": s("brow"), "BROW_SIDE": 1}))
    J.append(("head", "head_brow_right", head, {"PART": s("brow"), "BROW_SIDE": -1}))
    # framed torso
    tf = SCAD / "torso_frame" / "torso_frame.scad"
    for p in ("front_panel", "back_panel", "top_panel", "bottom_plate", "compute_tray", "driver_tray",
              "shoulder_block_L", "shoulder_block_R", "hatch_L", "hatch_R"):
        J.append(("torso", f"torso_{p}", tf, {"PART": s(p)}))
    # legs, pelvis and waist
    lg = SCAD / "legs" / "g1_leg_actuators.scad"
    for p in ("pelvis_left_half", "pelvis_right_half", "waist_yaw_link", "waist_stator_carrier"):
        J.append(("legs", p, lg, {"part": s(p)}))
    for side in ("left", "right"):
        for p in ("hip_pitch_link", "hip_roll_link", "thigh", "shin"):
            J.append(("legs", f"{side}_{p}", lg, {"part": s(p), "side": s(side)}))
    return J


# ------------------------------------------------------------------ mesh welding
# Several .scad files cut the original G1 arm meshes. Those meshes are closed in shape, but some
# triangle corners are a few micrometres apart instead of identical, and OpenSCAD's CGAL engine
# then stops with "The given mesh is not closed!". Welding the corners fixes that, and the shape
# moves by less than 0.01 mm. The originals are kept in STL/arms/original/.
WELD_DIRS = [ROOT / "STL" / "arms", ROOT / "STL" / "torso"]


def _read_stl(p):
    b = p.read_bytes()
    n = struct.unpack("<I", b[80:84])[0] if len(b) >= 84 else 0
    if len(b) == 84 + 50 * n:
        import numpy as np
        a = np.frombuffer(b, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)
        return a["v"].astype(np.float64)
    import numpy as np, re
    v = re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", b)
    return np.array(v, dtype=np.float64).reshape(-1, 3, 3)


def _open_edges(F):
    import numpy as np
    e = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), 1)
    _, c = np.unique(e, axis=0, return_counts=True)
    return int((c == 1).sum())


def _fill_holes(F):
    """close each boundary loop with a fan of triangles (holes in these meshes are small and flat)"""
    import numpy as np
    from collections import defaultdict
    E = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    key = defaultdict(int)
    for a, b in E:
        key[(a, b)] += 1
    nxt = defaultdict(list)
    for (a, b) in key:                       # boundary edge a->b has no partner b->a; the hole runs b->a
        if (b, a) not in key:
            nxt[b].append(a)
    new = []
    while any(nxt.values()):
        start = next(k for k, v in nxt.items() if v)
        loop = [start]; v = nxt[start].pop()
        while v != start and nxt.get(v):
            if v in loop:                    # pinched loop: close the part back to v as its own hole
                i = loop.index(v); sub = loop[i:]; loop = loop[:i + 1]
                new += [[sub[0], sub[k], sub[k + 1]] for k in range(1, len(sub) - 1)]
            else:
                loop.append(v)
            v = nxt[v].pop()
        if v == start and len(loop) >= 3:
            new += [[loop[0], loop[i], loop[i + 1]] for i in range(1, len(loop) - 1)]
    return np.vstack([F, np.array(new, dtype=F.dtype)]) if new else F


def weld(p, tol_frac=2e-6):
    """merge vertices closer than tol (relative to the part size); returns (open edges before, after)"""
    import numpy as np
    from scipy.spatial import cKDTree
    T = _read_stl(p)
    V = T.reshape(-1, 3)
    _, inv = np.unique(V.astype(np.float32), axis=0, return_inverse=True)
    before = _open_edges(inv.ravel().reshape(-1, 3))
    if before == 0:
        return 0, 0
    tol = tol_frac * np.ptp(V, axis=0).max()
    parent = np.arange(len(V))
    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]; i = parent[i]
        return i
    for i, j in cKDTree(V).query_pairs(tol, output_type="ndarray"):
        ri, rj = root(i), root(j)
        if ri != rj:
            parent[max(ri, rj)] = min(ri, rj)
    r = np.array([root(i) for i in range(len(V))])
    F = r.reshape(-1, 3)
    keep = (F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])
    F = F[keep]
    F = _fill_holes(F)
    after = _open_edges(F)
    W = V[F].astype(np.float32)
    nrm = np.cross(W[:, 1] - W[:, 0], W[:, 2] - W[:, 0])
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-30)
    rec = np.zeros(len(W), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["n"], rec["v"] = nrm, W
    bak = p.parent / "original" / p.name
    if not bak.exists():
        bak.parent.mkdir(exist_ok=True)
        shutil.copy2(p, bak)
    with open(p, "wb") as fh:
        fh.write(b"welded by export_stls.py".ljust(80) + struct.pack("<I", len(W)) + rec.tobytes())
    return before, after


def weld_inputs():
    try:
        import numpy, scipy  # noqa: F401
    except ImportError:
        print("  (skipping mesh welding: pip install numpy scipy)"); return
    for d in WELD_DIRS:
        for p in sorted(d.glob("*")):
            if p.suffix.lower() == ".stl" and p.is_file():
                b, a = weld(p)
                if b:
                    print(f"  welded {p.relative_to(ROOT)}: {b} open edges -> {a}")


def find_openscad(given):
    for c in [given, shutil.which("openscad"), shutil.which("openscad-nightly"),
              r"C:\Program Files\OpenSCAD\openscad.exe", r"C:\Program Files\OpenSCAD (Nightly)\openscad.exe",
              "/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD"]:
        if c and Path(c).exists():
            return str(c)
    sys.exit("openscad not found: install it, or pass --openscad /path/to/openscad")


def export(exe, binstl, out, job):
    group, name, scad, defs = job
    dst = out / group / f"{name}.stl"
    dst.parent.mkdir(parents=True, exist_ok=True)
    tmp = dst.with_suffix(".tmp.stl")
    cmd = [exe, "-o", str(tmp)] + (["--export-format", "binstl"] if binstl else []) + BACKEND
    for k, v in defs.items():
        cmd += ["-D", f"{k}={v}"]
    cmd.append(str(scad))
    t = time.time()
    r = subprocess.run(cmd, cwd=scad.parent, capture_output=True, text=True)
    log = (r.stderr or "") + (r.stdout or "")
    if r.returncode or not tmp.exists() or tmp.stat().st_size < 200 or "top level object is empty" in log:
        tmp.unlink(missing_ok=True)
        why = next((l for l in log.splitlines() if "ERROR" in l or "WARNING: Can't open" in l or "empty" in l),
                   log.strip().splitlines()[-1] if log.strip() else f"exit code {r.returncode}")
        return job, False, time.time() - t, why.strip()
    os.replace(tmp, dst)
    b = dst.read_bytes()
    n = struct.unpack("<I", b[80:84])[0] if len(b) == 84 + 50 * struct.unpack("<I", b[80:84])[0] else b.count(b"endfacet")
    return job, True, time.time() - t, f"{n:,} triangles"


def main():
    groups = sorted({j[0] for j in jobs()})
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(ROOT / "STL_export"), help="output folder (default STL_export/)")
    ap.add_argument("--only", nargs="+", choices=groups, help="export just these groups")
    ap.add_argument("--list", action="store_true", help="print the parts and file names, export nothing")
    ap.add_argument("--openscad", help="path to the openscad executable")
    ap.add_argument("--no-weld", action="store_true", help="don't repair the G1 arm/torso meshes first")
    ap.add_argument("--cgal", action="store_true", help="use the old CGAL engine even if Manifold is available")
    ap.add_argument("-j", "--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2), help="parts exported at once")
    a = ap.parse_args()
    out = Path(a.out).resolve()
    J = [j for j in jobs() if not a.only or j[0] in a.only]

    if a.list:
        for g, name, scad, defs in J:
            d = " ".join(f"{k}={v}" for k, v in defs.items())
            print(f"{g + '/' + name + '.stl':48s} <- {scad.relative_to(ROOT)} {d}")
        print(f"\n{len(J)} parts")
        return

    exe = find_openscad(a.openscad)
    h = subprocess.run([exe, "--help"], capture_output=True, text=True)
    helptext = h.stderr + h.stdout
    binstl = "binstl" in helptext
    global BACKEND
    if not a.cgal and "--backend" in helptext:
        BACKEND = ["--backend=Manifold"]            # OpenSCAD 2025 / nightly: much faster, tolerant of meshes
    elif not a.cgal and "manifold" in helptext.lower():
        BACKEND = ["--enable=manifold"]             # 2024 nightlies
    print(f"{exe}\n  engine: {'Manifold' if BACKEND else 'CGAL (slow; a 2025 or nightly OpenSCAD is much faster)'}")
    if not a.no_weld:
        weld_inputs()
    print(f"exporting {len(J)} parts to {out}, {a.jobs} at a time (rendering can take minutes per part)\n")
    t0 = time.time(); failed = []
    with ThreadPoolExecutor(a.jobs) as pool:
        for job, ok, dt, info in pool.map(lambda j: export(exe, binstl, out, j), J):
            print(f"  {'ok  ' if ok else 'FAIL'} {job[0] + '/' + job[1] + '.stl':48s} {dt:6.0f} s  {info}", flush=True)
            if not ok:
                failed.append(job)
    print(f"\n{len(J) - len(failed)}/{len(J)} exported in {time.time() - t0:.0f} s to {out}")
    if failed:
        print("failed: " + ", ".join(f"{g}/{n}" for g, n, _, _ in failed))
        sys.exit(1)


if __name__ == "__main__":
    main()