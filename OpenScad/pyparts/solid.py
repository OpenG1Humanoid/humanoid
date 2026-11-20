"""Shared tools for the Python part generators: signed-distance primitives, mesh SDFs, and finalize(),
which every generator uses to write its STLs.

finalize() guarantees a printable part:
  * sealed air pockets inside the part are filled,
  * crumbs smaller than `debris_cm3` are deleted,
  * any other piece that is not joined to the main body is bridged to it if it is within `bridge_mm`
    (Unitree's meshes are often several bodies that touch), otherwise the build STOPS with an error
    naming where the loose piece is - a loose piece is a design mistake, not something to hide,
  * the written STL is read back and checked to be ONE closed, outward-facing shell.
"""
import os
import struct
import sys

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'legs', 'waist'))
from meshsdf import load, occupancy, mesh_sdf, vox_sdf  # noqa: E402,F401


class LooseParts(RuntimeError):
    pass


# ------------------------------------------------------------------ grid
def grid(lo, hi, h):
    lo = np.asarray(lo, float); n = tuple(int(np.ceil((b - a) / h)) for a, b in zip(lo, hi))
    ax = [lo[i] + h * (np.arange(n[i]) + 0.5) for i in range(3)]
    return lo, n, ax


# ------------------------------------------------------------------ primitive SDFs in a local frame
# Every function takes local coordinates x, y, z (arrays) and returns a signed distance (mm, < 0 inside).
def _slab(z, z0, z1):
    a, b = min(z0, z1), max(z0, z1); return np.maximum(a - z, z - b)


def _ext(d2, dz):
    """intersection of a 2D profile (d2) extruded with a z slab (dz), exact outside the corners"""
    return np.minimum(np.maximum(d2, dz), 0) + np.hypot(np.maximum(d2, 0), np.maximum(dz, 0))


def prim_sdf(p, x, y, z):
    k = p[0]
    if k == 'cyl':
        return _ext(np.hypot(x, y) - p[1], _slab(z, p[2], p[3]))
    if k == 'ann':
        r = np.hypot(x, y); return _ext(np.maximum(r - p[2], p[1] - r), _slab(z, p[3], p[4]))
    if k == 'cylat':
        return _ext(np.hypot(x - p[1], y - p[2]) - p[3], _slab(z, p[4], p[5]))
    if k == 'box':
        dx = np.maximum(p[1] - x, x - p[2]); dy = np.maximum(p[3] - y, y - p[4])
        d2 = np.minimum(np.maximum(dx, dy), 0) + np.hypot(np.maximum(dx, 0), np.maximum(dy, 0))
        return _ext(d2, _slab(z, p[5], p[6]))
    if k == 'boxr':
        a = np.radians(p[1]); u = x * np.cos(a) + y * np.sin(a); v = -x * np.sin(a) + y * np.cos(a)
        return prim_sdf(('box', p[2], p[3], p[4], p[5], p[6], p[7]), u, v, z)
    if k == 'hex':                       # p[1] = across flats
        R = p[1] / 2; d2 = np.max(np.stack([np.abs(x * np.cos(t) + y * np.sin(t)) for t in np.radians([30, 90, 150])]), 0) - R
        return _ext(d2, _slab(z, p[2], p[3]))
    if k == 'rad':                       # radial cylinder: angle, z, r0, r1, diameter
        a = np.radians(p[1]); u = x * np.cos(a) + y * np.sin(a); v = -x * np.sin(a) + y * np.cos(a)
        return _ext(np.hypot(v, z - p[2]) - p[5] / 2, _slab(u, p[3], p[4]))
    if k == 'dcyl':                      # cylinder cut by a flat at distance p[4] towards angle p[5]
        a = np.radians(p[5]); u = x * np.cos(a) + y * np.sin(a)
        return np.maximum(prim_sdf(('cyl', p[1], p[2], p[3]), x, y, z), u - p[4])
    if k == 'cone':                      # radius p[1] at z = p[3] to radius p[2] at z = p[4]
        t = np.clip((z - p[3]) / (p[4] - p[3]), 0, 1); r = p[1] + (p[2] - p[1]) * t
        return _ext(np.hypot(x, y) - r, _slab(z, p[3], p[4]))
    raise ValueError(k)


def prim_world(prim, M, P):
    """SDF of a primitive placed by the 4x4 matrix M, at world points P (n,3)"""
    Mi = np.linalg.inv(M); Q = P @ Mi[:3, :3].T + Mi[:3, 3]
    return prim_sdf(prim, Q[:, 0], Q[:, 1], Q[:, 2])


# ------------------------------------------------------------------ finalize
def _label(mask, conn=1):
    st = ndimage.generate_binary_structure(3, conn); return ndimage.label(mask, st)


def finalize(F, lo, h, path, name=None, bridge_mm=1.5, debris_cm3=0.02, header=b'Open G1 part', drop_loose_cm3=0.0, step=1, min_wall=0.0):
    """clean the field F (mm, < 0 inside, samples at lo + h*(i+0.5)), write `path`, verify it.
    drop_loose_cm3: loose pieces up to this size that are too far to bridge are deleted instead of
    stopping the build - only for stray bodies in Unitree's meshes; every one is listed in the report.
    Returns a report dict. Raises LooseParts if a real piece cannot be joined."""
    name = name or os.path.basename(path)
    F = F.astype(np.float32, copy=True)
    if step > 1:     # coarser mesh: clean the same coarser grid that gets meshed, so the two agree
        F = np.ascontiguousarray(F[::step, ::step, ::step]); lo = np.asarray(lo, float) + 0.5 * h - 0.5 * h * step; h = h * step; step = 1
    vox = h ** 3 / 1000.0                                  # cm3 per voxel
    rep = dict(part=name, voids_filled=0, crumbs_removed=0, bridged=[], dropped=[], thin_removed_cm3=0.0)
    # the surface is drawn a hair inside zero (exact-zero samples on cut faces give pinched edges), and the
    # same level decides what is solid, so the cleaned pieces and the meshed pieces are the same pieces
    lev = -1e-4 * h
    F[np.abs(F - lev) < 1e-6 * h] = lev - 1e-6 * h
    # 1. sealed air pockets
    solid = F < lev
    empty, ne = _label(~solid)
    border = set(np.unique(np.concatenate([empty[0].ravel(), empty[-1].ravel(), empty[:, 0].ravel(), empty[:, -1].ravel(),
                                          empty[:, :, 0].ravel(), empty[:, :, -1].ravel()])))
    for k in range(1, ne + 1):
        if k not in border:
            m = empty == k; F[m] = -0.5 * h; rep['voids_filled'] += 1
    del empty
    # 1b. minimum wall: skins thinner than min_wall (left where a cut passes close to the surface, or
    #     in thin-shell source meshes) do not print; they are removed (morphological opening)
    if min_wall > 0:
        # a sliver = solid where NO point within min_wall is deeper than min_wall/2 (edges and corners of
        # thick parts have deep material right next to them, so they are left alone)
        solid = F < lev
        k = 2 * int(np.ceil(min_wall / h)) + 1
        depth = ndimage.maximum_filter(np.where(solid, -F, 0).astype(np.float32), size=k)
        thin = solid & (depth < min_wall / 2)
        rep['thin_removed_cm3'] = round(float(thin.sum() * vox), 3)
        F[thin] = 0.5 * h
    # 2. pieces
    for _ in range(4):
        solid = F < lev
        lab, nl = _label(solid)
        if nl <= 1: break
        sizes = np.bincount(lab.ravel())[1:]; main = int(np.argmax(sizes)) + 1
        mainmask = lab == main
        dist = None; changed = False
        for k in range(1, nl + 1):
            if k == main: continue
            m = lab == k; vol = sizes[k - 1] * vox
            if vol < debris_cm3:
                F[m] = 0.5 * h; rep['crumbs_removed'] += 1; changed = True; continue
            if dist is None: dist = ndimage.distance_transform_edt(~mainmask) * h
            gap = float(dist[m].min())
            where = (lo + h * (np.argwhere(m).mean(0) + 0.5)).round(1).tolist()
            if gap > bridge_mm and vol <= drop_loose_cm3:
                F[m] = 0.5 * h; rep['dropped'].append(dict(cm3=round(vol, 3), at_mm=where, gap_mm=round(gap, 2))); changed = True; continue
            if gap > bridge_mm:
                raise LooseParts(f'{name}: a {vol:.2f} cm3 piece at {where} mm is {gap:.1f} mm from the main body. '
                                 f'Fix the design so it is attached.')
            it = int(np.ceil(gap / h)) + 1
            bridge = ndimage.binary_dilation(m, iterations=it) & ndimage.binary_dilation(mainmask, iterations=it)
            F[bridge] = np.minimum(F[bridge], -0.5 * h)
            rep['bridged'].append(dict(cm3=round(vol, 3), at_mm=where, gap_mm=round(gap, 2))); changed = True
        if not changed: break
    # 3. mesh + write
    from skimage import measure
    big = float(np.nanmax(np.abs(F))) + 1
    # iso level a hair off zero: cut faces that line up exactly with the sample grid would otherwise give pinched edges
    v, f, _, _ = measure.marching_cubes(np.pad(F, 1, constant_values=big), lev, spacing=(h,) * 3, step_size=step)
    v = v - h + lo + 0.5 * h
    T = v[f]
    vol = np.einsum('ij,ij->i', T[:, 0], np.cross(T[:, 1], T[:, 2])).sum() / 6
    if vol < 0: T = T[:, ::-1]
    # weld points that land exactly on top of each other and drop the triangles that collapse
    T32 = T.astype(np.float32); key = T32.reshape(-1, 3).view(np.int32).reshape(-1, 3)
    _, inv = np.unique(key, axis=0, return_inverse=True); Fi = inv.ravel().reshape(-1, 3)
    T = T32[(Fi[:, 0] != Fi[:, 1]) & (Fi[:, 1] != Fi[:, 2]) & (Fi[:, 0] != Fi[:, 2])]
    write_tris(path, T, header)
    # 4. read back and verify
    chk = check_stl(path)
    rep.update(triangles=len(T), volume_cm3=round(abs(vol) / 1000, 2), shells=chk['shells'], open_edges=chk['open_edges'])
    if chk['shells'] != 1 or chk['open_edges'] or chk['volume_cm3'] <= 0:
        raise LooseParts(f'{name}: written STL is not one closed shell: {chk}')
    return rep


def write_tris(path, T, header=b'Open G1 part'):
    T = np.asarray(T, np.float32)
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(T), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])); rec['n'], rec['v'] = n, T
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'wb') as fh:
        fh.write(header[:80].ljust(80, b' ') + struct.pack('<I', len(T)) + rec.tobytes())


def check_stl(path):
    """shells (pieces sharing vertices), open edges and volume of an STL, as a slicer would see it"""
    T = load(path)
    V = T.reshape(-1, 3).astype(np.float32).view(np.int32)          # exact vertex identity, as written
    _, inv = np.unique(V, axis=0, return_inverse=True); Fc = inv.ravel().reshape(-1, 3)
    keep = (Fc[:, 0] != Fc[:, 1]) & (Fc[:, 1] != Fc[:, 2]) & (Fc[:, 0] != Fc[:, 2])
    Fc = Fc[keep]; T = T[keep]; n = len(Fc)
    A = coo_matrix((np.ones(3 * n), (np.repeat(np.arange(n), 3), Fc.ravel())), shape=(n, Fc.max() + 1)).tocsr()
    ns, lab = connected_components(A @ A.T, directed=False)
    e = np.sort(np.concatenate([Fc[:, [0, 1]], Fc[:, [1, 2]], Fc[:, [2, 0]]]), 1)
    _, c = np.unique(e, axis=0, return_counts=True)
    vol = np.einsum('ij,ij->i', T[:, 0], np.cross(T[:, 1], T[:, 2])).sum() / 6 / 1000
    pieces = sorted((float(np.einsum('ij,ij->i', T[lab == k, 0], np.cross(T[lab == k, 1], T[lab == k, 2])).sum() / 6 / 1000)
                     for k in range(ns)), key=lambda v: -abs(v))
    return dict(shells=int(ns), open_edges=int((c == 1).sum()), volume_cm3=round(float(vol), 3), pieces_cm3=[round(p, 3) for p in pieces[:5]])
