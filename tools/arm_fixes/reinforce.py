"""Thicken the shoulder-roll yoke bowl inward, keeping clear of everything on the pitch side over the roll range.

The stock G1 roll link joins its two ears to the base through a ~1.5-2 mm shell. This adds up to T_THICK of
material on the inside of that shell, only where:
  - it is inside the bowl (radially inside the shell's inner face, about the yaw axis), never on the outside;
  - it keeps CLEAR mm from everything on the pitch side (pitch link, roll cartridge + servo, cap) swept over ROLL_RANGE;
  - it leaves the yaw-cartridge pocket, its cable notch/exit and the two M3x18 screw columns open.
Run after jobs.py (it uses the edited roll cartridge/cap from ARMS_OUT):  python reinforce.py [left|right]"""
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from meshedit import *
from stlio import load, save
from asm import mods
from scipy import ndimage as ndi
from scipy.ndimage import map_coordinates

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.environ.get('ARMS_SRC', os.path.join(HERE, 'original_arms')) + '/'
NEW = os.environ.get('ARMS_OUT', os.path.join(HERE, 'out')) + '/'   # edited roll cartridge/cap (used for the sweep)
OUT = NEW

ROLL_RANGE = (-133.0, 133.0)   # deg; covers the -91..129 slider either way round
T_THICK = 6.0                  # max added wall thickness inward (mm)
CLEAR = 2.0                    # clearance to anything on the pitch side, swept (mm)
HC = 0.4                       # coarse grid for fill design
YAW_C = {'left': (0.0, 6.24), 'right': (0.0, -6.24)}   # yaw axis (x, y) in the roll link frame


def pitch_side_tris(side):
    D = mods(side); pl = D[f'{side}_shoulder_pitch_link']; rl = D[f'{side}_shoulder_roll_link']
    Mi = np.linalg.inv(rl['M'])            # pitch frame -> roll frame (at roll angle 0)
    def xf(T, M): V = T.reshape(-1, 3) @ M[:3, :3].T + M[:3, 3]; return V.reshape(-1, 3, 3)
    out = [xf(load(SRC + f'{side}/links/{side}_shoulder_pitch_link.stl'), Mi)]
    for pn in ['cartridge', 'cap']:
        P = pl['parts'][f'{side}_shoulder_roll_{pn}']
        f = f'{side}/actuators/shoulder_roll/{side}_shoulder_roll_{pn}.stl'
        out.append(xf(load((NEW if os.path.exists(NEW + f) else SRC) + f), Mi @ P))
    return out, rl


def occupancy(T, lo, n, h):
    """solid occupancy of a closed mesh on a given grid"""
    G = Grid.__new__(Grid)
    import ctypes
    from meshedit import _lib, _dp, _ip, _fp
    G.h = h; G.n = np.array(n, np.int32); G.o = np.array(lo, float) + np.array([0.123e-3, 0.271e-3, 0.0379e-3])
    d = np.empty(tuple(G.n), np.float32)
    tr = np.ascontiguousarray(T.reshape(-1), np.float64); o = np.ascontiguousarray(G.o); nn = np.ascontiguousarray(G.n)
    _lib.sdf_grid(tr.ctypes.data_as(_dp), len(T), o.ctypes.data_as(_dp), h, nn.ctypes.data_as(_ip), 2 * h, d.ctypes.data_as(_fp))
    return d < 0


def design_fill(side, keepouts):
    R = load(SRC + f'{side}/links/{side}_shoulder_roll_link.stl')
    V = R.reshape(-1, 3); lo = V.min(0) - 8; hi = V.max(0) + 8
    n = np.ceil((hi - lo) / HC).astype(int) + 1
    S = occupancy(R, lo, n, HC)
    # pitch-side parts at roll=0 on a grid big enough for the whole sweep
    parts, rl = pitch_side_tris(side)
    allv = np.concatenate([p.reshape(-1, 3) for p in parts])
    rmax = np.hypot(allv[:, 1], allv[:, 2]).max() + 2
    plo = np.array([min(allv[:, 0].min(), lo[0]) - 2, -rmax, -rmax]); phi_ = np.array([max(allv[:, 0].max(), hi[0]) + 2, rmax, rmax])
    pn = np.ceil((phi_ - plo) / HC).astype(int) + 1
    P = np.zeros(tuple(pn), bool)
    for T in parts: P |= occupancy(T, plo, pn, HC)
    # the servo inside the cartridge and the cartridge bore count as pitch-side solid too:
    # fill each x-slice's enclosed holes
    for i in range(P.shape[0]):
        if P[i].any(): P[i] = ndi.binary_fill_holes(P[i])
    # ---- sweep about the roll axis (x) in cylindrical coordinates
    dphi = 0.5; nphi = int(360 / dphi); nr = int(rmax / HC) + 1
    xs = lo[0] + HC * np.arange(n[0])
    rs = HC * np.arange(nr); ph = np.radians(dphi * np.arange(nphi))
    Rg, Pg = np.meshgrid(rs, ph, indexing='ij')
    Yc = Rg * np.cos(Pg); Zc = Rg * np.sin(Pg)
    lo_t, hi_t = np.radians(ROLL_RANGE[0]), np.radians(ROLL_RANGE[1])
    k0 = int(np.floor(ROLL_RANGE[0] / dphi)); k1 = int(np.ceil(ROLL_RANGE[1] / dphi))
    W = k1 - k0 + 1
    swept_cyl = np.zeros((len(xs), nr, nphi), bool)
    for i, x in enumerate(xs):
        ix = (x - plo[0]) / HC
        if ix < 0 or ix > pn[0] - 1: continue
        occ = map_coordinates(P[int(round(ix))].astype(np.float32), [(Yc - plo[1]) / HC, (Zc - plo[2]) / HC], order=1) > 0.3
        if not occ.any(): continue
        # pitch parts seen from the roll link rotate by -theta: swept(phi) = OR_theta occ(phi + theta)
        c = np.concatenate([occ, occ, occ], axis=1).astype(np.int32).cumsum(1)
        # window [phi + k0, phi + k1] (in index units), wrapped
        a = np.arange(nphi) + nphi + k0 - 1; b = np.arange(nphi) + nphi + k1
        swept_cyl[i] = (c[:, b] - c[:, a]) > 0
    # back to the cartesian grid
    ys = lo[1] + HC * np.arange(n[1]); zs = lo[2] + HC * np.arange(n[2])
    Yg, Zg = np.meshgrid(ys, zs, indexing='ij')
    rr = np.hypot(Yg, Zg) / HC; pp = (np.degrees(np.arctan2(Zg, Yg)) % 360) / dphi
    SW = np.zeros(tuple(n), bool)
    for i in range(n[0]):
        if swept_cyl[i].any():
            SW[i] = map_coordinates(swept_cyl[i].astype(np.float32), [rr, pp], order=0, mode='grid-wrap' if False else 'nearest') > 0.5
    d_sweep = ndi.distance_transform_edt(~SW) * HC
    # ---- "inside the bowl": the bowl is roughly a surface of revolution about the yaw axis (vertical).
    # Looking inward from outside along each radial ray (per z slice), everything past the inner face of the
    # first wall hit is inside. Open sides (no wall hit) and the outside are never filled.
    d_out = ndi.distance_transform_edt(~S) * HC
    yc = YAW_C[side]
    nrho = int(70 / HC); nth = 720
    rho = HC * np.arange(nrho); th = np.radians(np.arange(nth) * 360 / nth)
    Rr, Tt = np.meshgrid(rho, th, indexing='ij')
    px = (Rr * np.cos(Tt) - lo[0]) / HC; py = (yc[1] + Rr * np.sin(Tt) - lo[1]) / HC
    px += yc[0] / HC
    Xc, Yc2 = np.meshgrid(lo[0] + HC * np.arange(n[0]), lo[1] + HC * np.arange(n[1]), indexing='ij')
    rq = np.hypot(Xc - yc[0], Yc2 - yc[1]) / HC
    tq = (np.degrees(np.arctan2(Yc2 - yc[1], Xc - yc[0])) % 360) / (360 / nth)
    closed = np.zeros(tuple(n), bool)
    for k in range(n[2]):
        sl = S[:, :, k]
        if not sl.any(): continue
        pol = map_coordinates(sl.astype(np.float32), [px, py], order=0, mode='constant', cval=0) > 0.5   # (rho, th)
        inner = np.full(nth, -1.0)
        for j in range(nth):
            col = pol[:, j]
            idx = np.nonzero(col)[0]
            if len(idx) == 0: continue
            last = idx[-1]                     # outer face of the outermost wall
            run = last
            while run > 0 and col[run - 1]: run -= 1
            inner[j] = run                     # inner face of that wall (in rho index)
        lim = map_coordinates(inner, [tq], order=0, mode='grid-wrap')
        closed[:, :, k] = (lim > 0) & (rq < lim)
    closed &= ~S
    X, Y, Z = np.meshgrid(xs, ys, zs, indexing='ij')
    fill = closed & ~S & (d_out <= T_THICK) & (d_sweep >= CLEAR)
    for k in keepouts:
        if isinstance(k, tuple) and k[0] == 'pocket':
            # the yaw-cartridge pocket: empty space connected to a seed inside it, within the base cylinder
            c = k[1]
            cyl = ((X - c[0]) ** 2 + (Y - c[1]) ** 2 < 27.0 ** 2) & (Z < -44.0)
            lab, _ = ndi.label(~S & cyl)
            seed = tuple(np.round((np.array([c[0], c[1], -90.0]) - lo) / HC).astype(int))
            pk = lab == lab[seed]
            fill &= ~ndi.binary_dilation(pk, iterations=2)
        else:
            fill &= ~k(X, Y, Z)
    # drop thin specks; keep only pieces touching the link
    fill = ndi.binary_opening(fill, iterations=1)
    lab, nl = ndi.label(fill)
    touch = np.unique(lab[ndi.binary_dilation(S) & fill]); keep = np.isin(lab, touch[touch > 0])
    fill &= keep
    # close any pocket the fill sealed off (small voids between the old shell and the new material)
    U = S | fill
    fill |= ndi.binary_fill_holes(U) & ~U
    # overlap the fill 0.8 mm into the link so the union fuses, then smooth the voxel steps
    fx = fill | (ndi.binary_dilation(fill, iterations=2) & S)
    sdf = (ndi.distance_transform_edt(~fx) - ndi.distance_transform_edt(fx)) * HC
    sdf = ndi.gaussian_filter(sdf, 1.5) + 0.0137
    info = dict(fill_cm3=round(fill.sum() * HC ** 3 / 1000, 1), swept_vox=int(SW.sum()))
    return R, sdf.astype(np.float32), lo, info, (S, SW, fill, xs, ys, zs)


def apply_grid_fill(G, sdf, lo, h):
    """union a coarse SDF field into the fine grid, slab by slab"""
    ys = G.o[1] + G.h * np.arange(G.n[1]); zs = G.o[2] + G.h * np.arange(G.n[2])
    Yg, Zg = np.meshgrid((ys - lo[1]) / h, (zs - lo[2]) / h, indexing='ij')
    for i in range(G.n[0]):
        x = (G.o[0] + G.h * i - lo[0]) / h
        v = map_coordinates(sdf, [np.full_like(Yg, x), Yg, Zg], order=1, mode='nearest').astype(np.float32)
        np.minimum(G.d[i], v, out=G.d[i])


def keepouts_for(side):
    D = mods(side); rl = D[f'{side}_shoulder_roll_link']
    Pc = rl['parts'][f'{side}_shoulder_yaw_cartridge']
    c = Pc[:3, 3]
    # yaw cartridge M3x18 holes (from the top of the yoke base) and the yaw cable exit
    pts = []
    for lx, ly in [(5.3, 13.3), (5.3, -13.3)]:
        pts.append((Pc[:3, :3] @ np.array([lx, ly, 0]) + c)[:2])
    # yaw cable exit through the top of the base (far-end slot of the yaw cartridge)
    pts.append((Pc[:3, :3] @ np.array([18.0, 0, 0]) + c)[:2])
    ks = [lambda X, Y, Z, p=p: ((X - p[0]) ** 2 + (Y - p[1]) ** 2 < 4.5 ** 2) for p in pts]
    # keep the yaw-cartridge pocket untouched
    # yaw-cartridge pocket (r 20.9) and its cable notch toward +x
    ks.append(lambda X, Y, Z: ((X - c[0]) ** 2 + (Y - c[1]) ** 2 < 21.8 ** 2) & (Z < -44.0))
    ks.append(lambda X, Y, Z: (X > c[0] + 15.0) & (X < c[0] + 27.0) & (np.abs(Y - c[1]) < 4.5) & (Z < -44.0))
    # everything close to the roll axis (horn plate, idler pin, bearing, servo)
    ks.append(lambda X, Y, Z: (Y ** 2 + Z ** 2 < 38.0 ** 2))
    return ks, pts


def build(side, h=0.2):
    d = np.load(os.path.join(OUT, f'fill_{side}.npz'))
    f = f'{side}/links/{side}_shoulder_roll_link.stl'
    T = load(SRC + f)
    G = Grid(T, h)
    apply_grid_fill(G, d['sdf'], d['lo'], HC)
    V, F = G.mesh(max_err=0.02)
    info = check(V, F)
    os.makedirs(os.path.dirname(OUT + f), exist_ok=True)
    save(OUT + f, V[F])
    # print-oriented copy in actuators/shoulder_roll/ (same mesh, translated)
    A = T.reshape(-1, 3); B = load(SRC + f'{side}/actuators/shoulder_roll/{side}_shoulder_roll_link.stl').reshape(-1, 3)
    t = (B - A).mean(0); assert np.abs(A + t - B).max() < 1e-3
    save(OUT + f'{side}/actuators/shoulder_roll/{side}_shoulder_roll_link.stl', V[F] + t)
    return info


if __name__ == '__main__':
    import json
    for side in (sys.argv[1:] or ['left', 'right']):
        ks, pts = keepouts_for(side)
        R, sdf, lo, info, dbg = design_fill(side, ks)
        os.makedirs(OUT, exist_ok=True)
        np.savez_compressed(os.path.join(OUT, f'fill_{side}.npz'), sdf=sdf, lo=lo)
        print(side, info, build(side), flush=True)
