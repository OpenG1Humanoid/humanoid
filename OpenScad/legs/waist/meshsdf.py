"""Signed distance (mm, negative inside) of a triangle mesh on a grid, exact near the surface.

Grid samples sit at lo + h*(i + 0.5). Within `band` mm of the surface the distance is the exact
point-triangle distance (sign from the nearest triangle's normal, or from the winding-number
occupancy where the normal is ambiguous); further away it is a voxel distance transform."""
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

def load(p, scale=1.0):
    b = open(p, 'rb').read()
    if b[:5] == b'solid' and b'facet' in b[:300]:
        import re
        return np.array(re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', b), float).reshape(-1, 3, 3)*scale
    n = int.from_bytes(b[80:84], 'little')
    return np.frombuffer(b[84:84+50*n], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))['v'].reshape(-1, 3, 3).astype(float)*scale

def occupancy(T, lo, n, h):
    """winding-number occupancy at lo + h*(i+0.5), rays along +x"""
    jit = np.array([0.000137, 0.000271, 0.000419])
    nx, ny, nz = n
    ys = lo[1]+h*(np.arange(ny)+0.5)+jit[1]; zs = lo[2]+h*(np.arange(nz)+0.5)+jit[2]
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
    return np.cumsum(diff, axis=0)[:nx] > 0

def closest(P, A, B, C):
    ab = B-A; ac = C-A; ap = P-A
    d1 = (ab*ap).sum(1); d2 = (ac*ap).sum(1); bp = P-B; d3 = (ab*bp).sum(1); d4 = (ac*bp).sum(1)
    cp = P-C; d5 = (ab*cp).sum(1); d6 = (ac*cp).sum(1)
    va = d3*d6-d5*d4; vb = d5*d2-d1*d6; vc = d1*d4-d3*d2
    den = va+vb+vc; den = np.where(np.abs(den) < 1e-12, 1e-12, den)
    Q = A+ab*(vb/den)[:, None]+ac*(vc/den)[:, None]
    m = (vc <= 0) & (d1 >= 0) & (d3 <= 0); t = d1/np.where(d1-d3 == 0, 1e-12, d1-d3); Q[m] = (A+ab*t[:, None])[m]
    m = (vb <= 0) & (d2 >= 0) & (d6 <= 0); t = d2/np.where(d2-d6 == 0, 1e-12, d2-d6); Q[m] = (A+ac*t[:, None])[m]
    m = (va <= 0) & ((d4-d3) >= 0) & ((d5-d6) >= 0); t = (d4-d3)/np.where((d4-d3)+(d5-d6) == 0, 1e-12, (d4-d3)+(d5-d6)); Q[m] = (B+(C-B)*t[:, None])[m]
    m = (d1 <= 0) & (d2 <= 0); Q[m] = A[m]
    m = (d3 >= 0) & (d4 <= d3); Q[m] = B[m]
    m = (d6 >= 0) & (d5 <= d6); Q[m] = C[m]
    return np.linalg.norm(P-Q, axis=1), Q

def subdivide(T, emax):
    while True:
        e = np.max(np.linalg.norm(T-np.roll(T, 1, axis=1), axis=2), 1); big = e > emax
        if not big.any(): return T
        A, B, C = T[big, 0], T[big, 1], T[big, 2]; ab, bc, ca = (A+B)/2, (B+C)/2, (C+A)/2
        T = np.concatenate([T[~big], np.stack([A, ab, ca], 1), np.stack([ab, B, bc], 1), np.stack([ca, bc, C], 1), np.stack([ab, bc, ca], 1)])

def vox_sdf(occ, h):
    return np.where(occ, -(ndimage.distance_transform_edt(occ)-0.5), ndimage.distance_transform_edt(~occ)-0.5).astype(np.float32)*h

def mesh_sdf(T, lo, n, h, band=1.6, occ=None):
    if occ is None: occ = occupancy(T, lo, n, h)
    F = vox_sdf(occ, h)
    idx = np.argwhere(np.abs(F) <= band); P = lo+(idx+0.5)*h
    Ts = subdivide(T, max(2.0, 4*h)); N = np.cross(Ts[:, 1]-Ts[:, 0], Ts[:, 2]-Ts[:, 0]); N /= np.linalg.norm(N, axis=1, keepdims=True)+1e-12
    tree = cKDTree(Ts.mean(1)); K = 16
    for s in range(0, len(P), 300000):
        Q = P[s:s+300000]; dd, ii = tree.query(Q, k=K)
        best = np.full(len(Q), np.inf); bq = np.zeros_like(Q); bt = np.zeros(len(Q), int)
        for j in range(K):
            t = Ts[ii[:, j]]; dj, qj = closest(Q, t[:, 0], t[:, 1], t[:, 2])
            m = dj < best; best[m] = dj[m]; bq[m] = qj[m]; bt[m] = ii[m, j]
        dn = ((Q-bq)*N[bt]).sum(1); vin = occ[tuple(idx[s:s+300000].T)]
        sg = np.where(vin, -1.0, 1.0); clear = np.abs(dn) > 0.5*best; sg[clear] = np.sign(dn[clear])
        F[tuple(idx[s:s+300000].T)] = (sg*best).astype(np.float32)
    return F, occ

# analytic primitives (grid points X, Y, Z broadcastable)
def zcyl(X, Y, Z, r, z0, z1, cx=0.0, cy=0.0):
    return np.maximum(np.hypot(X-cx, Y-cy)-r, np.maximum(z0-Z, Z-z1))
def ycyl(X, Y, Z, x0, z0, r, y0, y1):
    return np.maximum(np.hypot(X-x0, Z-z0)-r, np.maximum(y0-Y, Y-y1))
def box(X, Y, Z, lo, hi):
    c = [(a+b)/2 for a, b in zip(lo, hi)]; e = [(b-a)/2 for a, b in zip(lo, hi)]
    q = [np.abs(X-c[0])-e[0], np.abs(Y-c[1])-e[1], np.abs(Z-c[2])-e[2]]
    out = np.sqrt(sum(np.maximum(v, 0)**2 for v in q)); return out+np.minimum(np.maximum(q[0], np.maximum(q[1], q[2])), 0)

def to_stl(F, lo, h, path, step=1):
    from skimage import measure
    from voxmesh import write_stl
    big = float(np.nanmax(np.abs(F)))+1
    v, f, _, _ = measure.marching_cubes(np.pad(F, 1, constant_values=big), 0.0, spacing=(h,)*3, step_size=step)
    v = v-h+lo+0.5*h
    T = v[f]; vol = np.einsum('ij,ij->i', T[:, 0], np.cross(T[:, 1], T[:, 2])).sum()/6
    write_stl(path, v, f, flip=vol < 0); return len(f), abs(vol)
