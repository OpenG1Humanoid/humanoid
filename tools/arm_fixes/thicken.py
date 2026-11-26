"""Smooth inward thickening of the roll link's yoke walls: the walls grow by up to TH towards the pitch side,
stopping CLEAR short of everything that swings with it (sweep2.py, real roll range). New surfaces are the offset
of the original inner faces (dS = TH) or the clearance surface (d_sweep = CLEAR), both smooth fields.
Only points between a wall and the sweep are used (wall-away and sweep-away directions opposite)."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, stlio
from scipy import ndimage as ndi
from scipy.ndimage import map_coordinates
from reinforce import occupancy
TH = 4.0; CLEAR = 2.0
SWF = {'left': 'sw2_left_-95_133.npz', 'right': 'sw2_right_-133_95.npz'}
Y0 = {'left': 6.24, 'right': -6.24}
def field(side, T, h=0.35, z_min=-72.0, z_max=-12.0, r_axis=24.0):
    d = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), SWF[side])); SW = d['SW']; slo = d['lo']; sh = 0.4
    V = T.reshape(-1, 3); lo = V.min(0) - 3; lo[2] = z_min - 4; hi = V.max(0) + 3; n = np.ceil((hi - lo) / h).astype(int) + 1
    S = occupancy(T, lo, n, h)
    S = ndi.binary_closing(S, iterations=3) | S                        # close ~1 mm seams/slits in the skin first
    dS = ndi.distance_transform_edt(~S) * h
    dsw0 = ndi.gaussian_filter(ndi.distance_transform_edt(~SW) * sh, 1.0 / sh)
    xs = lo[0] + h * np.arange(n[0]); ys = lo[1] + h * np.arange(n[1]); zs = lo[2] + h * np.arange(n[2])
    dsw = np.empty(S.shape, np.float32)
    for i, x in enumerate(xs):                                       # slab-wise trilinear resample
        Yg, Zg = np.meshgrid((ys - slo[1]) / sh, (zs - slo[2]) / sh, indexing='ij')
        dsw[i] = map_coordinates(dsw0, [np.full_like(Yg, (x - slo[0]) / sh), Yg, Zg], order=1, mode='nearest')
    dS = dS.astype(np.float32)
    gs = np.gradient(ndi.gaussian_filter(dS, 2.0)); gw = np.gradient(dsw)
    num = sum((a * b).astype(np.float32) for a, b in zip(gs, gw))
    den = np.sqrt(sum((a * a).astype(np.float32) for a in gs)) * np.sqrt(sum((b * b).astype(np.float32) for b in gw)) + 1e-9
    dot = num / den
    del gs, gw, num, den
    between = ndi.gaussian_filter((dot < -0.3).astype(np.float32), 2.0) > 0.5
    # only the yoke's inside: per z slice, along each ray from the yaw axis, up to the first wall
    y0 = Y0[side]
    Xs, Ys = np.meshgrid(xs, ys - y0, indexing='ij'); R = np.hypot(Xs, Ys); nb = 720
    bins = ((np.arctan2(Ys, Xs) + np.pi) / (2 * np.pi) * nb).astype(int) % nb
    inside = np.zeros(S.shape, bool)
    for k, z in enumerate(zs):
        if z < z_min or z > z_max: continue
        sk = S[:, :, k] & (R > (25.5 if z < -45.6 else 3.0))
        if not sk.any(): continue
        rin = np.full(nb, np.inf); np.minimum.at(rin, bins[sk], R[sk])
        ok = np.isfinite(rin); ok = ndi.binary_erosion(np.r_[ok, ok, ok], iterations=2)[nb:2 * nb]
        rin[~ok] = 0.0
        inside[:, :, k] = R < rin[bins]
    between &= inside
    X, Y, Z = np.meshgrid(xs, ys, zs, indexing='ij')
    keep = (np.hypot(Y, Z) > r_axis) & (Z > z_min)                  # roll-axis horn plate / idler; yaw cylinder below
    # above the yoke floor, offset only from the walls (the floor stays flat)
    kf = np.searchsorted(zs, -45.4)
    Sw = S.copy(); Sw[:, :, :kf] = False
    dSw = (ndi.distance_transform_edt(~Sw) * h).astype(np.float32)
    dS = np.where(Z > -45.6, dSw, dS); del dSw, Sw
    f = np.maximum(dS - TH, CLEAR - dsw)                              # <= 0: add
    f = np.where(between & keep & ~S, f, np.maximum(f, 1.0))
    f = np.where(S, np.minimum(f, -0.5), f)                           # overlap into the link
    f = ndi.gaussian_filter(f, 0.6)
    add = (f < 0) & ~S
    return np.pad(f, 2, constant_values=10.0).astype(np.float32), lo - 2 * h, h, round(add.sum() * h ** 3 / 1000, 1), (S, add, lo)
if __name__ == '__main__':
    side = sys.argv[1]
    T = stlio.load(f'/home/claude/openg1humanoid/humanoid/STL/arms/{side}/links/{side}_shoulder_roll_link.stl')
    f, lo, h, vol, (S, add, lo0) = field(side, T)
    print(side, 'thickening cm3', vol)
    np.savez_compressed(f'/home/claude/work/thk_{side}.npz', S=S, add=add, lo=lo0)
