"""Exact signed distance to the original torso surface in a 1 mm band round the envelope (float32 grid, NaN elsewhere)."""
import sys, os, numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
from prims import torso_tris, WORK

def closest_dist(P, A, B, C):
    """vectorised point-triangle distance (Ericson, Real-Time Collision Detection 5.1.5)"""
    ab = B-A; ac = C-A; ap = P-A
    d1 = (ab*ap).sum(1); d2 = (ac*ap).sum(1)
    bp = P-B; d3 = (ab*bp).sum(1); d4 = (ac*bp).sum(1)
    cp = P-C; d5 = (ab*cp).sum(1); d6 = (ac*cp).sum(1)
    va = d3*d6 - d5*d4; vb = d5*d2 - d1*d6; vc = d1*d4 - d3*d2
    Q = np.empty_like(P)
    den = va+vb+vc; den = np.where(np.abs(den) < 1e-12, 1e-12, den)
    v = vb/den; w = vc/den; Q[:] = A + ab*v[:, None] + ac*w[:, None]          # interior
    m = (vc <= 0) & (d1 >= 0) & (d3 <= 0); t = d1/np.where(d1-d3 == 0, 1e-12, d1-d3); Q[m] = (A + ab*t[:, None])[m]
    m = (vb <= 0) & (d2 >= 0) & (d6 <= 0); t = d2/np.where(d2-d6 == 0, 1e-12, d2-d6); Q[m] = (A + ac*t[:, None])[m]
    m = (va <= 0) & ((d4-d3) >= 0) & ((d5-d6) >= 0)
    t = (d4-d3)/np.where((d4-d3)+(d5-d6) == 0, 1e-12, (d4-d3)+(d5-d6)); Q[m] = (B + (C-B)*t[:, None])[m]
    m = (d1 <= 0) & (d2 <= 0); Q[m] = A[m]
    m = (d3 >= 0) & (d4 <= d3); Q[m] = B[m]
    m = (d6 >= 0) & (d5 <= d6); Q[m] = C[m]
    return np.linalg.norm(P-Q, axis=1), Q

if __name__ == '__main__':
    d = np.load(os.path.join(WORK, 'E_0.5.npz')); E = d['E']; lo = d['lo'].astype(float); h = float(d['h'])
    T = torso_tris()
    # subdivide big triangles so nearest-centroid search finds the true nearest triangle
    while True:
        e = np.max(np.linalg.norm(T - np.roll(T, 1, axis=1), axis=2), 1)
        big = e > 3.0
        if not big.any(): break
        A, B, C = T[big, 0], T[big, 1], T[big, 2]; ab, bc, ca = (A+B)/2, (B+C)/2, (C+A)/2
        T = np.concatenate([T[~big], np.stack([A, ab, ca], 1), np.stack([ab, B, bc], 1), np.stack([ca, bc, C], 1), np.stack([ab, bc, ca], 1)])
    print('triangles after subdivision', len(T))
    bnd = E ^ ndimage.binary_erosion(E)
    band = ndimage.binary_dilation(bnd, iterations=3)          # +-1.5 mm
    idx = np.argwhere(band); P = lo + idx*h
    print('band points', len(P))
    cen = T.mean(1); tree = cKDTree(cen)
    rmax = np.linalg.norm(T - cen[:, None], axis=2).max(1)     # triangle circumradius bound
    N = np.cross(T[:, 1]-T[:, 0], T[:, 2]-T[:, 0]); N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
    K = 16; out = np.empty(len(P), np.float32); sgn = np.empty(len(P), np.float32)
    vox_in = E[tuple(idx.T)]
    for s in range(0, len(P), 400000):
        Q = P[s:s+400000]; dd, ii = tree.query(Q, k=K)
        best = np.full(len(Q), np.inf); bq = np.zeros_like(Q); bt = np.zeros(len(Q), int)
        for j in range(K):
            tri = T[ii[:, j]]
            dj, qj = closest_dist(Q, tri[:, 0], tri[:, 1], tri[:, 2])
            m = dj < best; best[m] = dj[m]; bq[m] = qj[m]; bt[m] = ii[m, j]
        dvec = Q - bq; dn = (dvec*N[bt]).sum(1)
        # sign from the nearest triangle's outward normal when it is unambiguous, else from the voxels
        sg = np.where(vox_in[s:s+400000], -1.0, 1.0)
        clear = np.abs(dn) > 0.5*best
        sg[clear] = np.sign(dn[clear])
        sg[best < 1e-6] = 0
        out[s:s+400000] = best; sgn[s:s+400000] = sg
        print(s, flush=True)
    S = np.full(E.shape, np.nan, np.float32)
    val = out*sgn
    ok = out <= 2.1                                            # beyond this the envelope is not on the mesh (fills, caps)
    S[tuple(idx[ok].T)] = val[ok]
    np.save(os.path.join(WORK, 'outer_sdf.npy'), S)
    print('valid', ok.mean())
