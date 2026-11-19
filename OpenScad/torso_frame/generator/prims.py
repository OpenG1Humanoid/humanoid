"""Helpers shared by the torso generator (copied from the arm generator so this folder runs on its own)."""
import os, sys
import numpy as np
from scipy import ndimage
HERE = os.path.dirname(os.path.abspath(__file__))
TF = os.path.dirname(HERE)                    # torso_frame/
PKG = os.path.dirname(os.path.dirname(TF))    # repo root (OpenScad/, STL/)
WORK = os.path.join(HERE, 'work'); os.makedirs(WORK, exist_ok=True)
sys.path.insert(0, os.path.join(PKG, 'OpenScad', 'generator'))
import design as D                            # arm/torso servo features (shoulder pitch pockets + hatches)

def load(p):
    b = open(p, 'rb').read()
    if b[:5] == b'solid' and b'facet' in b[:400]:
        import re
        return np.array(re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', b), dtype=float).reshape(-1, 3, 3)
    n = int.from_bytes(b[80:84], 'little')
    d = np.frombuffer(b[84:84+n*50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return d['v'].reshape(-1, 3, 3).astype(float)

def torso_tris():
    """original torso_link_rev_1_0, split into its bodies (stl/torso, metres) -> mm"""
    return np.vstack([load(os.path.join(PKG, 'STL', 'torso', b))*1000 for b in D.TORSO_BODIES])

def occupancy(T, lo, hi, h):
    """winding-number occupancy of triangle soup T (mm), sampled at lo + h*(i+0.5)"""
    jit = np.array([0.000137, 0.000271, 0.000419])
    nx, ny, nz = np.ceil((np.array(hi)-lo)/h).astype(int)
    xs = lo[0]+h*(np.arange(nx)+0.5)+jit[0]; ys = lo[1]+h*(np.arange(ny)+0.5)+jit[1]; zs = lo[2]+h*(np.arange(nz)+0.5)+jit[2]
    diff = np.zeros((nx+1, ny, nz), np.int32)
    A, B, C = T[:, 0], T[:, 1], T[:, 2]; nrm = np.cross(B-A, C-A)
    for t in range(len(T)):
        if abs(nrm[t, 0]) < 1e-12: continue
        y0, y1 = min(A[t, 1], B[t, 1], C[t, 1]), max(A[t, 1], B[t, 1], C[t, 1]); z0, z1 = min(A[t, 2], B[t, 2], C[t, 2]), max(A[t, 2], B[t, 2], C[t, 2])
        iy = np.where((ys >= y0) & (ys <= y1))[0]; iz = np.where((zs >= z0) & (zs <= z1))[0]
        if len(iy) == 0 or len(iz) == 0: continue
        Y, Z = np.meshgrid(ys[iy], zs[iz], indexing='ij')
        a, b, c = A[t, 1:], B[t, 1:], C[t, 1:]
        v0 = c-a; v1 = b-a; d00 = v0@v0; d01 = v0@v1; d11 = v1@v1; den = d00*d11-d01*d01
        if abs(den) < 1e-14: continue
        py = Y-a[0]; pz = Z-a[1]; d20 = py*v0[0]+pz*v0[1]; d21 = py*v1[0]+pz*v1[1]
        u = (d11*d20-d01*d21)/den; v = (d00*d21-d01*d20)/den
        m = (u >= 0) & (v >= 0) & (u+v <= 1)
        if not m.any(): continue
        xc = A[t, 0]+u*(C[t, 0]-A[t, 0])+v*(B[t, 0]-A[t, 0])
        k = np.clip(np.ceil((xc-lo[0]-jit[0])/h-0.5).astype(int), 0, nx)
        s = 1 if nrm[t, 0] > 0 else -1
        gy, gz = np.meshgrid(iy, iz, indexing='ij')
        np.add.at(diff, (np.zeros_like(k[m]), gy[m], gz[m]), s)
        np.add.at(diff, (k[m], gy[m], gz[m]), -s)
    occ = np.cumsum(diff, axis=0)[:nx] > 0
    return ndimage.binary_fill_holes(occ)

def prim_mask(P, M, prim):
    Mi = np.linalg.inv(M); Q = P@Mi[:3, :3].T+Mi[:3, 3]; x, y, z = Q[:, 0], Q[:, 1], Q[:, 2]; k = prim[0]
    def zr(a, b): lo_, hi_ = min(a, b), max(a, b); return (z >= lo_) & (z <= hi_)
    if k == "cyl": return (np.hypot(x, y) <= prim[1]) & zr(prim[2], prim[3])
    if k == "ann": r = np.hypot(x, y); return (r >= prim[1]) & (r <= prim[2]) & zr(prim[3], prim[4])
    if k == "box": return (x >= prim[1]) & (x <= prim[2]) & (y >= prim[3]) & (y <= prim[4]) & zr(prim[5], prim[6])
    if k == "cylat": return (np.hypot(x-prim[1], y-prim[2]) <= prim[3]) & zr(prim[4], prim[5])
    if k == "hex":
        R = prim[1]/2; a = np.stack([np.abs(x*np.cos(t)+y*np.sin(t)) for t in np.radians([30, 90, 150])]); return np.all(a <= R, 0) & zr(prim[2], prim[3])
    if k == "rad":
        a = np.radians(prim[1]); u = x*np.cos(a)+y*np.sin(a); v = -x*np.sin(a)+y*np.cos(a)
        return (u >= prim[3]) & (u <= prim[4]) & (np.hypot(v, z-prim[2]) <= prim[5]/2)
    raise ValueError(k)

def head_stl():
    p = os.environ.get('G1_HEAD_STL', os.path.join(PKG, 'STL', 'head', 'head_link.STL'))
    if not os.path.isfile(p):
        raise SystemExit("Need Unitree's head_link.STL (the stock collar becomes the head socket): "
                         "set G1_HEAD_STL=/path/to/head_link.STL or copy it to STL/head/head_link.STL")
    return p
