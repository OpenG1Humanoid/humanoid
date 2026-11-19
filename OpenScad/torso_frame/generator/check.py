"""Checks: frame clearance to the outer surface, part overlaps, removal paths, outer-surface accuracy."""
import os, sys, itertools, json
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
import layout as L
from prims import WORK, TF, torso_tris
from outer_sdf import closest_dist
from voxmesh import load_stl_mm
res = {}
# 1. frame clearance (distance from each 2020 member to the outside, 0.5 mm grid)
d = np.load(os.path.join(WORK, 'E_0.5.npz')); E = d['E']; lo = d['lo']; h = float(d['h'])
D = ndimage.distance_transform_edt(E).astype(np.float32)*h
ix = lambda v, a: int(round((v-lo[a])/h))
cl = {}
for nm, b, ax in L.members():
    x0, x1, y0, y1, z0, z1 = b
    cl[nm] = float(D[ix(x0, 0):ix(x1, 0)+1, ix(y0, 1):ix(y1, 1)+1, ix(z0, 2):ix(z1, 2)+1].min())
res['frame_min_distance_to_outside_mm'] = cl; del D
print('frame clearance (mm):', {k: round(v, 1) for k, v in cl.items()})
# 2. overlaps + 3. removal
p = np.load(os.path.join(WORK, 'parts_0.5.npz')); n = p['front_panel'].shape
xs, ys, zs = [lo[i]+h*np.arange(n[i]) for i in range(3)]
alu = np.zeros(n, bool)
for nm, b, ax in L.members():
    x0, x1, y0, y1, z0, z1 = b
    alu |= ((xs[:, None, None] > x0+.01) & (xs[:, None, None] < x1-.01) & (ys[None, :, None] > y0+.01) & (ys[None, :, None] < y1-.01)
            & (zs[None, None, :] > z0+.01) & (zs[None, None, :] < z1-.01))
P = {k: p[k] for k in p.files if k not in ('lo', 'h')}; P['aluminium'] = alu
ov = [(a, b, float((P[a] & P[b]).sum()*h**3)) for a, b in itertools.combinations(P, 2) if (P[a] & P[b]).any()]
res['overlaps_mm3'] = ov; print('overlaps:', ov or 'none')
obst = alu | P['compute_tray'] | P['driver_tray'] | P['bottom_plate']
rem = {}
for part, ax, sg in (('front_panel', 0, 1), ('back_panel', 0, -1), ('top_panel', 2, 1), ('shoulder_block_L', 2, 1), ('shoulder_block_R', 2, 1)):
    ahead = (np.flip(np.cumsum(np.flip(obst, ax), ax), ax) > 0) if sg > 0 else (np.cumsum(obst, ax) > 0)
    rem[part] = int((P[part] & ahead).sum())
res['removal_blocked_voxels'] = rem; print('removal blocked voxels:', rem)
# 4. outer surface vs original triangles
del P, alu, obst, p, ahead
import gc; gc.collect()
T = torso_tris()
while True:
    e = np.max(np.linalg.norm(T-np.roll(T, 1, axis=1), axis=2), 1); big = e > 3.0
    if not big.any(): break
    A, B, C = T[big, 0], T[big, 1], T[big, 2]; ab, bc, ca = (A+B)/2, (B+C)/2, (C+A)/2
    T = np.concatenate([T[~big], np.stack([A, ab, ca], 1), np.stack([ab, B, bc], 1), np.stack([ca, bc, C], 1), np.stack([ab, bc, ca], 1)])
tree = cKDTree(T.mean(1))
depth = ndimage.distance_transform_edt(E).astype(np.float32)*h
Dou = ndimage.distance_transform_edt(~E).astype(np.float32)*h
np.subtract(depth, Dou, out=depth); del Dou   # >0 inside, <0 outside (one of the two terms is 0)
acc = {}
for k in ('front_panel', 'back_panel', 'top_panel', 'shoulder_block_L', 'shoulder_block_R'):
    V = np.unique((load_stl_mm(os.path.join(TF, 'stl', f'{k}.stl'))/1000).reshape(-1, 3), axis=0)
    i = np.clip(np.round((V-lo)/h).astype(int), 0, np.array(E.shape)-1); V = V[np.abs(depth[tuple(i.T)]) <= 1.0]
    dd, ii = tree.query(V, k=16); best = np.full(len(V), np.inf)
    for j in range(16):
        t = T[ii[:, j]]; best = np.minimum(best, closest_dist(V, t[:, 0], t[:, 1], t[:, 2])[0])
    acc[k] = dict(median_mm=round(float(np.median(best)), 3), share_within_0_05mm=round(float((best < 0.05).mean()), 3))
res['outer_surface_vs_original'] = acc; print('outer surface:', acc)
json.dump(res, open(os.path.join(TF, 'checks.json'), 'w'), indent=1)
