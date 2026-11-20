"""Clearance checks for the ankle linkage over the whole G1 ankle range (run after gen_ankle.py).

Checks, at 15 x 9 (pitch, roll) poses:
  * each push rod (6 mm diameter, rod-end housings 8 mm diameter for 12 mm at each end) vs the shin
  * the heel ears and the whole foot vs the shin
  * the crank plates (swept) vs the shin
"""
import json, os, sys
import numpy as np
from scipy.spatial import cKDTree
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', 'waist'))
import gen_ankle as g
from meshsdf import load
STL = g.STL

shin = np.unique(load(os.path.join(STL, 'left_knee_link_ankle.stl')).reshape(-1, 3).round(1), axis=0)
foot = load(os.path.join(STL, 'left_ankle_roll_link_ears.stl')).reshape(-1, 3)
shin_tree = cKDTree(shin)
foot = np.unique(foot.round(0), axis=0)          # 1 mm sample of the foot surface
ears = foot[(foot[:, 0] > g.EAR['x'] - 12) & (foot[:, 0] < g.EAR['x'] + 12) & (np.abs(foot[:, 1]) > g.EAR['y_in'] - 1) & (foot[:, 2] > 5)]

def to_shin(P, qp, qr):
    return g.J_APITCH + (g.Ry(qp) @ (g.J_AROLL[:, None] + g.Rx(qr) @ P.T)).T

J, _ = g.linkage_report()
rep = dict(rod_min_mm=99.0, rod_worst=None, ear_min_mm=99.0, foot_min_mm=99.0, foot_worst=None)
for qp in np.linspace(*g.Q_PITCH, 11):
    for qr in np.linspace(*g.Q_ROLL, 5):
        for i, w in enumerate('AB'):
            ph = g.solve_phi(w, qp, qr, J[i] @ [qp, qr])
            a, b = g.crank_tip(w, ph), g.foot_point(w, qp, qr)
            t = np.linspace(0, 1, 120)[:, None]; P = a + t * (b - a)
            L = np.linalg.norm(b - a); s = t[:, 0] * L
            r = np.where((s < 12) | (s > L - 12), 4.0, 3.0)          # rod-end housing / rod radius
            d = shin_tree.query(P)[0] - r
            if d.min() < rep['rod_min_mm']: rep['rod_min_mm'] = float(d.min()); rep['rod_worst'] = [w, round(qp, 2), round(qr, 2)]
        de = shin_tree.query(to_shin(ears, qp, qr))[0].min()
        rep['ear_min_mm'] = min(rep['ear_min_mm'], float(de))
        df = shin_tree.query(to_shin(foot, qp, qr))[0].min()
        if df < rep['foot_min_mm']: rep['foot_min_mm'] = float(df); rep['foot_worst'] = [round(qp, 2), round(qr, 2)]

# crank plates: swept disc of radius RC + 8 in y = Y_FACE + 2 .. Y_FACE + 10 (each side) vs the shin
for w, sgn in (('A', 1), ('B', -1)):
    zc = g.ZU if w == 'A' else g.ZL
    near = shin[(np.hypot(shin[:, 0] - g.XM, shin[:, 2] - zc) < g.RC + 8) & (sgn * shin[:, 1] > g.Y_FACE + 2 - 0.5)]
    rep[f'crank_{w}_shin_points_in_sweep'] = int(len(near))
rep['shin_max_abs_y_mm'] = float(np.abs(shin[:, 1]).max())
for k, v in rep.items():
    print(f'  {k:32s} {round(v, 2) if isinstance(v, float) else v}')
json.dump(rep, open(os.path.join(HERE, 'ankle_checks.json'), 'w'), indent=1)
