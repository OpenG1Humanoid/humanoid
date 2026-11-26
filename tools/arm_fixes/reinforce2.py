"""Inside-only thickening for a C-shaped yoke link (wrist roll, wrist yaw).

For a yoke whose ears carry joint axis A:
  - "inside" = radially inside the first wall met when looking in from outside towards A (per slice along A);
  - new material keeps CLEAR mm from everything on the other side of the joint, swept over the joint range;
  - nothing is added within R_AXIS of A (horn plate, idler pin, bearings) or in the given keep-outs.
"""
import sys, os, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from meshedit import *
from stlio import load, save
from asm import mods
from scipy import ndimage as ndi
from scipy.ndimage import map_coordinates
from reinforce import occupancy, apply_grid_fill

SRC = os.environ.get('ARMS_SRC', os.path.join(HERE, 'original_arms')) + '/'
NEW = os.environ.get('ARMS_OUT', os.path.join(HERE, 'out')) + '/'
OUT = NEW
HC = 0.35


def frame_from_axis(p, d):
    """4x4 that maps a frame with x = d through p into the link frame"""
    d = np.asarray(d, float); d /= np.linalg.norm(d)
    a = np.array([0, 0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0, 0])
    e2 = np.cross(d, a); e2 /= np.linalg.norm(e2); e3 = np.cross(d, e2)
    M = np.eye(4); M[:3, 0] = d; M[:3, 1] = e2; M[:3, 2] = e3; M[:3, 3] = p
    return M


def xf(T, M):
    V = T.reshape(-1, 3) @ M[:3, :3].T + M[:3, 3]; return V.reshape(-1, 3, 3)


def design(link_T, moving, axis_p, axis_d, rng, T_THICK=5.0, CLEAR=2.0, R_AXIS=16.0, keepouts=()):
    """all inputs in the link frame. moving: list of triangle arrays (at joint angle 0)."""
    Ma = frame_from_axis(axis_p, axis_d); Mi = np.linalg.inv(Ma)
    L = xf(link_T, Mi); mv = [xf(m, Mi) for m in moving]          # canonical: joint axis = x through origin
    V = L.reshape(-1, 3); lo = V.min(0) - 8; hi = V.max(0) + 8
    n = np.ceil((hi - lo) / HC).astype(int) + 1
    S = occupancy(L, lo, n, HC)
    allv = np.concatenate([m.reshape(-1, 3) for m in mv])
    rmax = max(np.hypot(allv[:, 1], allv[:, 2]).max(), np.hypot(V[:, 1], V[:, 2]).max()) + 2
    plo = np.array([lo[0], -rmax, -rmax]); phi_ = np.array([hi[0], rmax, rmax])
    pn = np.ceil((phi_ - plo) / HC).astype(int) + 1
    P = np.zeros(tuple(pn), bool)
    for m in mv: P |= occupancy(m, plo, pn, HC)
    for i in range(P.shape[0]):                       # servos inside cartridges count as solid
        if P[i].any(): P[i] = ndi.binary_fill_holes(P[i])
    dphi = 0.5; nphi = int(360 / dphi); nr = int(rmax / HC) + 1
    rs = HC * np.arange(nr); ph = np.radians(dphi * np.arange(nphi))
    Rg, Pg = np.meshgrid(rs, ph, indexing='ij'); Yc = Rg * np.cos(Pg); Zc = Rg * np.sin(Pg)
    k0 = int(np.floor(rng[0] / dphi)); k1 = int(np.ceil(rng[1] / dphi))
    xs = lo[0] + HC * np.arange(n[0]); ys = lo[1] + HC * np.arange(n[1]); zs = lo[2] + HC * np.arange(n[2])
    Yg, Zg = np.meshgrid(ys, zs, indexing='ij')
    rr = np.hypot(Yg, Zg) / HC; pp = (np.degrees(np.arctan2(Zg, Yg)) % 360) / dphi
    SW = np.zeros(tuple(n), bool)
    for i, x in enumerate(xs):
        ix = (x - plo[0]) / HC
        if ix < 0 or ix > pn[0] - 1: continue
        occ = map_coordinates(P[int(round(ix))].astype(np.float32), [(Yc - plo[1]) / HC, (Zc - plo[2]) / HC], order=1) > 0.3
        if not occ.any(): continue
        c = np.concatenate([occ, occ, occ], axis=1).astype(np.int32).cumsum(1)
        a = np.arange(nphi) + nphi + k0 - 1; b = np.arange(nphi) + nphi + k1
        sw = (c[:, b] - c[:, a]) > 0
        SW[i] = map_coordinates(sw.astype(np.float32), [rr, pp], order=0, mode='nearest') > 0.5
    d_sweep = ndi.distance_transform_edt(~SW) * HC
    d_out = ndi.distance_transform_edt(~S) * HC
    # inside: per x-slice (along the joint axis), polar about the axis: inside the inner face of the outermost wall
    nth = 720; rho = HC * np.arange(int(rmax / HC) + 2); th = np.radians(np.arange(nth) * 360 / nth)
    RR, TT = np.meshgrid(rho, th, indexing='ij')
    py = (RR * np.cos(TT) - lo[1]) / HC; pz = (RR * np.sin(TT) - lo[2]) / HC
    rq = np.hypot(Yg, Zg) / HC; tq = (np.degrees(np.arctan2(Zg, Yg)) % 360) / (360 / nth)
    inside = np.zeros(tuple(n), bool)
    for i in range(n[0]):
        sl = S[i]
        if not sl.any(): continue
        pol = map_coordinates(sl.astype(np.float32), [py, pz], order=0, mode='constant', cval=0) > 0.5
        inner = np.full(nth, -1.0)
        for j in range(nth):
            idx = np.nonzero(pol[:, j])[0]
            if len(idx) == 0: continue
            run = idx[-1]
            while run > 0 and pol[run - 1, j]: run -= 1
            inner[j] = run
        lim = map_coordinates(inner, [tq], order=0, mode='grid-wrap')
        inside[i] = (lim > 0) & (rq < lim)
    X, Y, Z = np.meshgrid(xs, ys, zs, indexing='ij')
    fill = inside & ~S & (d_out <= T_THICK) & (d_sweep >= CLEAR) & (np.hypot(Y, Z) > R_AXIS)
    # keep-outs are given in the link frame
    if keepouts:
        W = np.stack([X, Y, Z], -1) @ Ma[:3, :3].T + Ma[:3, 3]
        for k in keepouts: fill &= ~k(W[..., 0], W[..., 1], W[..., 2])
    fill = ndi.binary_opening(fill, iterations=1)
    lab, nl = ndi.label(fill)
    touch = np.unique(lab[ndi.binary_dilation(S) & fill]); fill &= np.isin(lab, touch[touch > 0])
    U = S | fill; fill |= ndi.binary_fill_holes(U) & ~U
    fx = fill | (ndi.binary_dilation(fill, iterations=2) & S)
    sdf = (ndi.distance_transform_edt(~fx) - ndi.distance_transform_edt(fx)) * HC
    sdf = ndi.gaussian_filter(sdf, 1.5) + 0.0137
    return dict(sdf=sdf.astype(np.float32), lo=lo, Ma=Ma, S=S, SW=SW, fill=fill,
                fill_cm3=round(fill.sum() * HC ** 3 / 1000, 1))


class FieldFill(Prim):
    """fill defined on a coarse grid in a rotated frame"""
    def __init__(self, sdf, lo, h, Ma):
        self.f = sdf; self.lo = lo; self.h = h; self.M = Ma; self.Mi = np.linalg.inv(Ma)
    def sdf(self, P):
        L = P @ self.Mi[:3, :3].T + self.Mi[:3, 3]
        return map_coordinates(self.f, [(L[..., k] - self.lo[k]) / self.h for k in range(3)], order=1, mode='nearest', cval=10)
    def aabb(self):
        hi = self.lo + self.h * (np.array(self.f.shape) - 1)
        c = np.array([[x, y, z] for x in (self.lo[0], hi[0]) for y in (self.lo[1], hi[1]) for z in (self.lo[2], hi[2])])
        w = c @ self.M[:3, :3].T + self.M[:3, 3]; return w.min(0), w.max(0)


def joint_setup(side, which):
    D = mods(side)
    def part(folder, name, M):
        f = f'{side}/actuators/{folder}/{side}_{name}.stl'
        return xf(load((NEW if os.path.exists(NEW + f) else SRC) + f), M)
    if which == 'wrist_yaw':
        yl = D[f'{side}_wrist_yaw_link']; pl = D[f'{side}_wrist_pitch_link']
        Mi = np.linalg.inv(yl['M'])                                   # pitch frame -> yaw link frame
        moving = [xf(load(SRC + f'{side}/links/{side}_wrist_pitch_link.stl'), Mi)]
        for pn, M in pl['parts'].items():
            if 'wrist_yaw_cartridge' in pn or 'wrist_yaw_cap' in pn or 'wrist_pitch_cartridge' in pn or 'wrist_pitch_cap' in pn:
                moving.append(part(pn.split(side + '_')[1].rsplit('_', 1)[0], pn.split(side + '_', 1)[1], Mi @ M))
        link = f'{side}/links/{side}_wrist_yaw_link.stl'
        return link, moving, np.zeros(3), yl['v'], (-96.0, 96.0)
    if which == 'wrist_roll':
        rl = D[f'{side}_wrist_roll_link']; pl = D[f'{side}_wrist_pitch_link']; yl = D[f'{side}_wrist_yaw_link']
        Mp = pl['M']                                                  # pitch frame -> roll link frame (angle 0)
        moving = [xf(load(SRC + f'{side}/links/{side}_wrist_pitch_link.stl'), Mp)]
        for pn, M in pl['parts'].items():
            if 'cartridge' in pn or 'cap' in pn:
                moving.append(part(pn.split(side + '_')[1].rsplit('_', 1)[0], pn.split(side + '_', 1)[1], Mp @ M))
        # wrist yaw link at a few yaw angles (it rides on the pitch link)
        Ty = load(SRC + f'{side}/links/{side}_wrist_yaw_link.stl')
        for a in (-92, -46, 0, 46, 92):
            c, s = np.cos(np.radians(a)), np.sin(np.radians(a))
            Rz = np.eye(4); Rz[:2, :2] = [[c, -s], [s, c]]
            moving.append(xf(Ty, Mp @ yl['M'] @ Rz))
        link = f'{side}/links/{side}_wrist_roll_link.stl'
        return link, moving, Mp[:3, 3], Mp[:3, :3] @ pl['v'], (-96.0, 96.0)


def module_dirs(side, which):
    return {'wrist_yaw': 'wrist_yaw', 'wrist_roll': 'wrist_roll'}[which]


def keepouts(which):
    if which == 'wrist_roll':
        # 4x M3x6 fitted from inside the cup through the base into the wrist-roll horn plate: keep them reachable
        return [lambda X, Y, Z, y0=y0, z0=z0: ((Y - y0) ** 2 + (Z - z0) ** 2 < 3.6 ** 2) & (X > -2)
                for y0 in (-6.2, 6.2) for z0 in (-6.2, 6.2)]
    return []


def build(side, which, h=0.15):
    link, moving, ap, ad, rng = joint_setup(side, which)
    T = load(SRC + link)
    r = design(T, moving, ap, ad, rng, keepouts=keepouts(which))
    G = Grid(T, h)
    G.apply(FieldFill(r['sdf'], r['lo'], HC, r['Ma']), 'fill')
    V, F = G.mesh(max_err=0.02)
    info = check(V, F); info['fill_cm3'] = r['fill_cm3']
    save(OUT + link, V[F])
    folder = module_dirs(side, which)
    cp = f'{side}/actuators/{folder}/{side}_{which}_link.stl'
    if os.path.exists(SRC + cp):
        A = T.reshape(-1, 3); B = load(SRC + cp).reshape(-1, 3); t = (B - A).mean(0)
        assert np.abs(A + t - B).max() < 1e-3
        save(OUT + cp, V[F] + t)
    return info


if __name__ == '__main__':
    for side in ['left', 'right']:
        for which in sys.argv[1:]:
            print(side, which, build(side, which), flush=True)
