#!/usr/bin/env python3
"""Gather every printable part into STL_export/<group>/<name>.stl.

The arm, leg and torso parts are made by the Python generators (see README "Making the STLs") and
are copied here; only the head is rendered with the openscad command line.

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
STALE = []           # wrapper .scad files whose part no longer exists in the library (left over from older versions)
SOLID_TORSO = "--solid-torso" in sys.argv   # the old one-piece torso (replaced by the framed torso)
OPENSCAD_ARMS = "--openscad-arms" in sys.argv   # old route: arms through OpenSCAD instead of build_arm_stls.py


def s(v):                       # an OpenSCAD string literal for -D
    return f'"{v}"'


def jobs():
    """(group, output name, .scad file, {variable: value})"""
    J = []
    STALE.clear()
    # arms + servo torso: one .scad per part, the part is already set inside the file
    import re
    lib = (SCAD / "lib" / "g1_servo_arm.scad").read_text()
    known = set(re.findall(r'if \(part == "([^"]+)"\)', lib))
    folders = [("parts_left", "arms"), ("parts_right", "arms"), ("hardware", "hardware")] if OPENSCAD_ARMS else []
    if SOLID_TORSO: folders.append(("parts_torso", "arms"))
    for folder, group in folders:
        for f in sorted((SCAD / folder).glob("*.scad")):
            m = re.search(r'part\s*=\s*"([^"]+)"', f.read_text())
            if m and m.group(1) not in known:
                STALE.append(f); continue
            J.append((group, f.stem, f, {}))
    # dome head
    head = SCAD / "head" / "head.scad"
    J.append(("head", "head_frame", head, {"PART": s("frame")}))
    for i in range(5):
        J.append(("head", f"head_panel_{i}", head, {"PART": s("panel"), "PANEL": i}))
    J.append(("head", "head_brow_left", head, {"PART": s("brow"), "BROW_SIDE": 1}))
    J.append(("head", "head_brow_right", head, {"PART": s("brow"), "BROW_SIDE": -1}))
    # framed torso, legs: made by their Python generators (copied below), not OpenSCAD
    return J


# parts made by the Python generators (no .scad of their own): copied into the export as they are
import glob as _glob
_AP = ROOT / "STL" / "arms_print"
_ARM_FILES = sorted(_glob.glob(str(_AP / "**" / "*.stl"), recursive=True))      # any subfolder of STL/arms_print
_HW = ("hub_", "idler_pin")
COPY = [("hardware" if Path(f).stem.startswith(_HW) else "arms", Path(f).stem, Path(f)) for f in _ARM_FILES] + \
       [("torso", f"torso_{Path(f).stem}", Path(f)) for f in sorted(_glob.glob(str(ROOT / "STL" / "torso_frame" / "*.stl")))] + \
       [("legs", f"{sd}_{p}", ROOT / "STL" / "legs" / f"{sd}_{p}_integrated.stl") for sd in ("left", "right")
        for p in ("hip_pitch_link", "hip_roll_link", "hip_yaw_link_knee")] + \
       [("legs", f"pelvis_{sd}_half", ROOT / "STL" / "legs" / f"pelvis_{sd}_half_waist.stl") for sd in ("left", "right")] + \
       [("legs", "waist_yaw_link", ROOT / "STL" / "legs" / "waist_yaw_link_integrated.stl")] + \
       [("legs", f"{sd}_shin_ankle", ROOT / "STL" / "legs" / f"{sd}_knee_link_ankle.stl") for sd in ("left", "right")] + \
       [("legs", f"{sd}_foot", ROOT / "STL" / "legs" / f"{sd}_ankle_roll_link_ears.stl") for sd in ("left", "right")] + \
       [("legs", "ankle_crank_x4", ROOT / "STL" / "legs" / "ankle_crank.stl"),
        ("legs", "stator_carrier_x5", ROOT / "STL" / "legs" / "waist_stator_carrier.stl")]


# ------------------------------------------------------------------ mesh welding
# Several .scad files cut the original G1 arm meshes. Those meshes are closed in shape, but some
# triangle corners are a few micrometres apart instead of identical, and OpenSCAD's CGAL engine
# then stops with "The given mesh is not closed!". Welding the corners fixes that, and the shape
# moves by less than 0.01 mm. The originals are kept in STL/arms/original/.
WELD_DIRS = [ROOT / "STL" / "arms"]


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


def _drop_duplicates(F):
    """remove repeated triangles: a pair facing opposite ways is an internal double wall (both go),
    a pair facing the same way is a copy (one goes)"""
    import numpy as np
    key = np.sort(F, axis=1)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel(); keep = np.ones(len(F), bool)
    for g in np.where(cnt > 1)[0]:
        idx = np.where(inv == g)[0]
        orient = [tuple(np.roll(F[i], -int(np.argmin(F[i])))) for i in idx]   # same cyclic order = same facing
        keep[idx] = False
        groups = {}
        for i, o in zip(idx, orient): groups.setdefault(o, []).append(i)
        facings = list(groups.values())
        if len(facings) == 1: keep[facings[0][0]] = True                  # plain copies: keep one
    return F[keep]


def _split_nonmanifold(V, F, nudge):
    """Two solid pieces touching along an edge leave 4 triangles on that edge. Pair the triangles
    that bound the same piece, give each piece its own copy of the corner points, and move each copy
    `nudge` into its own piece, so the mesh becomes separate closed shells (STL has no vertex ids,
    so the copies must also be apart in space). Every pairing (2 per edge) is tried until the
    result has no shared edges left."""
    import itertools
    import numpy as np
    from collections import defaultdict
    E = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), 1)
    u, c = np.unique(E, axis=0, return_counts=True)
    bad = [tuple(x) for x in u[c == 4]]
    if not bad or len(bad) > 10: return V, F, 0
    N = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]); N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-30)
    edge_faces = defaultdict(list)
    for fi, f in enumerate(F):
        for k in range(3):
            edge_faces[tuple(sorted((f[k], f[(k + 1) % 3])))].append(fi)
    base = defaultdict(set)
    for e, fs in edge_faces.items():
        if e not in bad and len(fs) == 2: base[fs[0]].add(fs[1]); base[fs[1]].add(fs[0])
    options = []
    for e in bad:
        fs = edge_faces[e]
        a, b = V[e[0]], V[e[1]]; t = (b - a) / np.linalg.norm(b - a)
        wing = {}
        for fi in fs:
            third = [v for v in F[fi] if v not in e][0]
            w = V[third] - a; w -= t * (w @ t); wing[fi] = w / np.linalg.norm(w)
        x0 = wing[fs[0]]; y0 = np.cross(t, x0)
        order = sorted(fs, key=lambda fi: np.arctan2(wing[fi] @ y0, wing[fi] @ x0))
        options.append([[(order[s0], order[s0 + 1]), (order[(s0 + 2) % 4], order[(s0 + 3) % 4])] for s0 in (0, 1)]
                       + [[(order[0], order[2]), (order[1], order[3])]])
    vert_faces = defaultdict(list)
    for fi, f in enumerate(F):
        for v in f: vert_faces[v].append(fi)
    for choice in itertools.product(*options):
        links = defaultdict(set, {k: set(v) for k, v in base.items()})
        for pairs in choice:
            for f1, f2 in pairs: links[f1].add(f2); links[f2].add(f1)
        V2, F2 = V.copy(), F.copy(); split = 0
        for v in {x for e in bad for x in e}:
            fs = set(vert_faces[v]); groups = []
            while fs:
                stack = [fs.pop()]; g = set(stack)
                while stack:
                    cur = stack.pop()
                    for nb in links[cur]:
                        if nb in fs: fs.remove(nb); g.add(nb); stack.append(nb)
                groups.append(g)
            if len(groups) < 2: continue
            for g in groups:
                inward = -sum(N[fi] for fi in g); inward /= max(np.linalg.norm(inward), 1e-30)
                nv = len(V2); V2 = np.vstack([V2, V[v] + nudge * inward])
                for fi in g: F2[fi][F2[fi] == v] = nv
                split += 1
        if _nonmanifold_edges(F2) == 0 and _open_edges(F2) == 0 and _misoriented_edges(F2) == 0:
            return V2, F2, split
    return V, F, 0


def _misoriented_edges(F):
    """edges not used exactly once in each direction (a closed, consistently wound surface has none)"""
    import numpy as np
    d = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])
    u, c = np.unique(d, axis=0, return_counts=True)
    have = {tuple(x): k for x, k in zip(u, c)}
    return int(sum(1 for (a, b), k in have.items() if k != 1 or have.get((b, a), 0) != 1))


def _nonmanifold_edges(F):
    import numpy as np
    e = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), 1)
    _, c = np.unique(e, axis=0, return_counts=True)
    return int((c > 2).sum())


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
    F0 = inv.ravel().reshape(-1, 3)
    before = _open_edges(F0) + _nonmanifold_edges(F0) + (len(F0) - len(_drop_duplicates(F0)))
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
    F = _drop_duplicates(F[keep])
    F = _fill_holes(F)
    V, F, _ = _split_nonmanifold(V, F, nudge=5 * tol)
    after = _open_edges(F) + _nonmanifold_edges(F)
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
                    print(f"  repaired {p.relative_to(ROOT)}: {b} problems -> {a}")
                if a:
                    print(f"  WARNING {p.relative_to(ROOT)} still has {a} defects: use a 2025/nightly OpenSCAD (Manifold engine)")


def find_openscad(given):
    for c in [given, shutil.which("openscad"), shutil.which("openscad-nightly"),
              r"C:\Program Files\OpenSCAD\openscad.exe", r"C:\Program Files\OpenSCAD (Nightly)\openscad.exe",
              "/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD"]:
        if c and Path(c).exists():
            return str(c)
    return None


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
    groups = sorted({j[0] for j in jobs()} | {c[0] for c in COPY})
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(ROOT / "STL_export"), help="output folder (default STL_export/)")
    ap.add_argument("--only", nargs="+", choices=groups, help="export just these groups")
    ap.add_argument("--list", action="store_true", help="print the parts and file names, export nothing")
    ap.add_argument("--openscad", help="path to the openscad executable")
    ap.add_argument("--openscad-arms", action="store_true", help="old route: render the arm parts with OpenSCAD instead of copying build_arm_stls.py output")
    ap.add_argument("--solid-torso", action="store_true", help="also export the old one-piece torso_link (not printable on most beds)")
    ap.add_argument("--no-weld", action="store_true", help="don't repair the G1 arm/torso meshes first")
    ap.add_argument("--cgal", action="store_true", help="use the old CGAL engine even if Manifold is available")
    ap.add_argument("-j", "--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2), help="parts exported at once")
    a = ap.parse_args()
    out = Path(a.out).resolve()
    J = [j for j in jobs() if not a.only or j[0] in a.only]

    C = [c for c in COPY if not a.only or c[0] in a.only]
    if STALE:
        print("old wrapper files for parts that no longer exist (skipped - you can delete them):")
        for f in STALE: print(f"    {f.relative_to(ROOT)}")
        print()
    if a.list:
        for g, name, src in C:
            print(f"{g + '/' + name + '.stl':48s} <- copy of {src.relative_to(ROOT)}")
        for g, name, scad, defs in J:
            d = " ".join(f"{k}={v}" for k, v in defs.items())
            print(f"{g + '/' + name + '.stl':48s} <- {scad.relative_to(ROOT)} {d}")
        print(f"\n{len(J) + len(C)} parts")
        return

    exe = find_openscad(a.openscad)
    BACKEND.clear(); binstl = False
    if exe is None:
        if J:
            print("OpenSCAD not found, so the head parts are skipped (everything else is copied).\n"
                  "  To export the head too: install OpenSCAD, or pass --openscad \"C:/path/to/openscad.exe\"\n")
        J = []
    else:
        h = subprocess.run([exe, "--help"], capture_output=True, text=True)
        helptext = h.stderr + h.stdout
        binstl = "binstl" in helptext
        if not a.cgal and "--backend" in helptext:
            BACKEND.append("--backend=Manifold")
        elif not a.cgal and "manifold" in helptext.lower():
            BACKEND.append("--enable=manifold")
        print(f"{exe}\n  engine: {'Manifold' if BACKEND else 'CGAL'}")
    if OPENSCAD_ARMS and exe and not a.no_weld:
        weld_inputs()
    n_arm = sum(c[0] in ("arms", "hardware") for c in C)
    print(f"arm + hub parts found in {_AP}: {n_arm}")
    if n_arm == 0 and (not a.only or {"arms", "hardware"} & set(a.only)):
        print("NOTE: STL/arms holds Unitree's input meshes, not printable parts. The printable arm parts go in\n"
              "      STL/arms_print: build them with  python OpenScad/generator/build_arm_stls.py  (about 50 min),\n"
              "      or copy the 'arms' and 'hardware' folders from STL_export_python.zip into STL/arms_print\n")
    for g, name, src in C:
        dst = out / g / f"{name}.stl"; dst.parent.mkdir(parents=True, exist_ok=True)
        if src.exists(): shutil.copy2(src, dst); print(f"  copy {g + '/' + name + '.stl':48s} (from {src.relative_to(ROOT)})")
        else: print(f"  MISSING {src.relative_to(ROOT)}: run its generator (see OpenScad/legs/README.md)")
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
