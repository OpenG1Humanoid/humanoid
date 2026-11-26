"""Pitch-side sweep about the roll axis, seen from the roll link. The pitch link is taken as its mesh (no hole
filling); only the roll cartridge + cap get their enclosed holes filled (the servo inside)."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, stlio
from scipy import ndimage as ndi
from scipy.ndimage import map_coordinates
import reinforce as Rf
from reinforce import occupancy
def sweep(side, rng, lo, n, h=0.4):
    parts, rl = Rf.pitch_side_tris(side)
    allv = np.concatenate([p.reshape(-1, 3) for p in parts])
    rmax = np.hypot(allv[:, 1], allv[:, 2]).max() + 2
    plo = np.array([allv[:, 0].min() - 2, -rmax, -rmax]); phi_ = np.array([allv[:, 0].max() + 2, rmax, rmax])
    pn = np.ceil((phi_ - plo) / h).astype(int) + 1
    P = occupancy(parts[0], plo, pn, h)
    Q = np.zeros_like(P)
    for T in parts[1:]: Q |= occupancy(T, plo, pn, h)
    for i in range(Q.shape[0]):
        if Q[i].any(): Q[i] = ndi.binary_fill_holes(Q[i])
    P |= Q
    dphi = 0.5; nphi = int(360 / dphi); nr = int(rmax / h) + 1
    xs = lo[0] + h * np.arange(n[0])
    rs = h * np.arange(nr); ph = np.radians(dphi * np.arange(nphi))
    Rg, Pg = np.meshgrid(rs, ph, indexing='ij'); Yc = Rg * np.cos(Pg); Zc = Rg * np.sin(Pg)
    k0 = int(np.floor(rng[0] / dphi)); k1 = int(np.ceil(rng[1] / dphi))
    ys = lo[1] + h * np.arange(n[1]); zs = lo[2] + h * np.arange(n[2])
    Yg, Zg = np.meshgrid(ys, zs, indexing='ij')
    rr = np.hypot(Yg, Zg) / h; pp = (np.degrees(np.arctan2(Zg, Yg)) % 360) / dphi
    SW = np.zeros(tuple(n), bool)
    for i, x in enumerate(xs):
        ix = (x - plo[0]) / h
        if ix < 0 or ix > pn[0] - 1: continue
        occ = map_coordinates(P[int(round(ix))].astype(np.float32), [(Yc - plo[1]) / h, (Zc - plo[2]) / h], order=1) > 0.3
        if not occ.any(): continue
        c = np.concatenate([occ, occ, occ], axis=1).astype(np.int32).cumsum(1)
        a = np.arange(nphi) + nphi + k0 - 1; b = np.arange(nphi) + nphi + k1
        sc = (c[:, b] - c[:, a]) > 0
        SW[i] = map_coordinates(sc.astype(np.float32), [rr, pp], order=0, mode='nearest') > 0.5
    return SW
if __name__ == '__main__':
    side = sys.argv[1]; h = 0.4
    T = stlio.load(f'/home/claude/openg1humanoid/humanoid/STL/arms/{side}/links/{side}_shoulder_roll_link.stl')
    V = T.reshape(-1, 3); lo = V.min(0) - 8; hi = V.max(0) + 8; n = np.ceil((hi - lo) / h).astype(int) + 1
    S = occupancy(T, lo, n, h)
    for rng in [(-95.0, 133.0), (-133.0, 95.0), (0.0, 0.0)]:
        SW = sweep(side, rng, lo, n, h)
        print(side, rng, 'overlap with roll link cm3', round((SW & S).sum() * h ** 3 / 1000, 2), flush=True)
        np.savez_compressed(f'/home/claude/work/sw2_{side}_{int(rng[0])}_{int(rng[1])}.npz', SW=SW, lo=lo)
