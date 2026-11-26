"""OpenG1 micro shoulder yaw, servo moved into the upper arm (yaw link).

Starting parts: the repo's original roll link and yaw link, and the micro cartridge / horn plate with the horn-pocket
and cable fixes. The whole yaw stack is turned upside down (mirrored about the joint plane) and turned 45 deg about the
yaw axis to sit in the deepest part of the upper arm's solid top block:

  yaw link   : old horn-plate socket filled; pocket for the cartridge (bearing ring + lug cross) cut into the top;
               2 M3 inserts in the pocket floor; wire groove in the floor; servo-plug exit channel through the side.
  cartridge  : lugs carried down to the bottom ring as solid columns; each lug has a through hole, with the
               M3 head counterbored just under the bearing, so 2x M3x30 go in from the top into the yaw link.
  horn plate : 2 M3 inserts in the key's end face.
  roll link  : the empty ring between the yaw cylinder and the outer skin, the old cartridge pocket, the old screw
               holes and the old cable hole are filled solid up to the yoke floor (kept 2 mm clear of the pitch-side
               sweep). A hex socket for the horn-plate key is cut in the bottom, and 2 M3x50 go down from the yoke
               floor into the key's inserts.
"""
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from meshedit import *
from stlio import load, save
from scipy import ndimage as ndi
from scipy.ndimage import map_coordinates
from reinforce import occupancy, apply_grid_fill, CLEAR
from lugfix import Extrude, mask2d, cartridge_column, roll_channel, H2

REPO = '/home/claude/openg1humanoid/humanoid/STL/arms/'
MIC = '/home/claude/micro/STL/arms/'
OUT = os.environ.get('NEWYAW_OUT', '/home/claude/final3/STL/arms') + '/'
Y0 = {'left': 6.24, 'right': -6.24}
TH = {'left': 45.0, 'right': -45.0}
LUG = [(5.3, 13.3), (5.3, -13.3)]
HORN_INS = [(0.0, 6.5), (0.0, -6.5)]
FLOOR_TOP = -45.6


def Tr(x, y, z):
    M = np.eye(4); M[:3, 3] = (x, y, z); return M
def Rz(d):
    c, s = np.cos(np.radians(d)), np.sin(np.radians(d)); M = np.eye(4); M[:2, :2] = [[c, -s], [s, c]]; return M
FLIP = np.diag([-1.0, 1.0, -1.0, 1.0])
def G_old(side): return Rz(TH[side]) @ FLIP                        # old yaw-frame position -> new yaw frame
def G_c(side): return G_old(side) @ Tr(0.0003, 0.0, 0.25)         # cartridge local -> yaw frame
def G_h(side): return G_old(side) @ Tr(-0.001, 0.0, -6.7007)      # horn plate local -> yaw frame
def Y2R(side): return Tr(0.0, Y0[side], -103.2)                   # yaw frame -> roll frame (yaw = 0)
def apply(M, p): return M[:3, :3] @ np.asarray(p, float) + M[:3, 3]


def _save(f, V, F, copy_from=None):
    os.makedirs(os.path.dirname(OUT + f), exist_ok=True); save(OUT + f, V[F])
    if copy_from:                                                   # print-oriented copy = translated link
        src, dst = copy_from
        A = load(REPO + src.replace('ACT', 'links')).reshape(-1, 3); B = load(REPO + src).reshape(-1, 3)
        t = (B - A).mean(0); assert np.abs(A + t - B).max() < 1e-3
        os.makedirs(os.path.dirname(OUT + dst), exist_ok=True); save(OUT + dst, V[F] + t)


def solid_only(V, F):
    """drop enclosed voids and stray specks: keep the largest shell"""
    nc, lab = components(V, F)
    if nc == 1: return V, F
    T = V[F]; fl = lab[F[:, 0]]
    vol = np.bincount(fl, weights=np.einsum('ij,ij->i', T[:, 0], np.cross(T[:, 1], T[:, 2])) / 6, minlength=nc)
    F = F[fl == np.argmax(vol)]
    return weld(V[F].reshape(-1, 3), np.arange(len(F) * 3).reshape(-1, 3))


def mask_sdf(add, S, lo, h):
    fx = add | (ndi.binary_dilation(add, iterations=4) & S)
    sdf = (ndi.distance_transform_edt(~fx) - ndi.distance_transform_edt(fx)) * h
    sdf = ndi.gaussian_filter(sdf, 0.7)
    return np.pad(sdf, 2, constant_values=10.0).astype(np.float32), lo - 2 * h


def slice_fill(S, zs, zmin, zmax, close_r, h, close_above=None):
    """per z slice: (optional closing) then fill enclosed holes; returns added voxels"""
    add = np.zeros_like(S)
    r = close_r / h
    for k, z in enumerate(zs):
        if z < zmin or z > zmax: continue
        s = S[:, :, k]
        if not s.any(): continue
        if close_above is None or z > close_above:
            dil = ndi.distance_transform_edt(~s) <= r
            s = s | (ndi.distance_transform_edt(dil) > r)
        add[:, :, k] = ndi.binary_fill_holes(s) & ~S[:, :, k]
    return add


# ------------------------------------------------------------------ roll link
def ridge_slices(S, xs, ys, zs, y0, zmin, zmax, z_open=-62.0, r_skin=25.5):
    """solid fill inside the roll link's outer skin, slice by slice.
    Below z_open the skin is a closed ring: fill every enclosed hole.
    Above it the skin is a C open at the front: along each ray from the yaw axis that hits the skin, fill up to the
    skin's inner face (nothing in directions with no skin, nothing outside the skin)."""
    X, Y = np.meshgrid(xs, ys - y0, indexing='ij')
    R = np.hypot(X, Y); A = np.arctan2(Y, X); nb = 720
    bins = ((A + np.pi) / (2 * np.pi) * nb).astype(int) % nb
    add = np.zeros_like(S); ridge = np.zeros_like(S)
    for k, z in enumerate(zs):
        if z < zmin or z > zmax: continue
        s = S[:, :, k]
        if not s.any(): continue
        a = (ndi.binary_fill_holes(s) | ((R < 22.0) & (z < FLOOR_TOP - 0.2))) & ~s   # enclosed holes + old pocket/holes
        add[:, :, k] = a
        if z > z_open:
            sk = s & (R > r_skin)
            rin = np.full(nb, np.inf)
            np.minimum.at(rin, bins[sk], R[sk])
            ok = np.isfinite(rin)
            ok = ndi.binary_erosion(np.r_[ok, ok, ok], iterations=2)[nb:2 * nb]   # stay off the C's open ends
            rin[~ok] = 0.0
            ridge[:, :, k] = (R < rin[bins]) & ~s & ~a
    return add, ridge


def roll_fill(side, T, h=0.2):
    y0 = Y0[side]
    lo = np.array([-35.0, y0 - 35.0, -104.0]); hi = np.array([35.0, y0 + 35.0, FLOOR_TOP])
    n = np.ceil((hi - lo) / h).astype(int) + 1
    S = occupancy(T, lo, n, h)
    xs = lo[0] + h * np.arange(n[0]); ys = lo[1] + h * np.arange(n[1]); zs = lo[2] + h * np.arange(n[2])
    add, ridge = ridge_slices(S, xs, ys, zs, y0, -104.0, FLOOR_TOP)
    # the open ridge (above the closed ring) keeps 2 mm clear of everything that swings with the pitch side
    from thicken import SWF
    d = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), SWF[side])); SW = d['SW']; slo = d['lo']; sh = 0.4      # sweep2.py, real roll range
    dsw = ndi.distance_transform_edt(~SW) * sh
    X, Y, Z = np.meshgrid((xs - slo[0]) / sh, (ys - slo[1]) / sh, (zs - slo[2]) / sh, indexing='ij')
    D = map_coordinates(dsw, [X.ravel(), Y.ravel(), Z.ravel()], order=1, mode='nearest').reshape(X.shape)
    add |= ridge & (D >= CLEAR)
    sdf, lo2 = mask_sdf(add, S, lo, h)
    return sdf, lo2, h, round(add.sum() * h ** 3 / 1000, 1)


def socket_prim(side):
    """hex socket for the horn-plate key: the key's own outline + 0.2 mm, moved with the horn plate into the roll link"""
    Th = load(MIC + f'{side}/actuators/shoulder_yaw/{side}_shoulder_yaw_hornplate.stl')
    lo = np.array([-14.0, -14.0]); n = (np.array([28.0, 28.0]) / H2).astype(int)
    key = ndi.binary_fill_holes(mask2d(Th, 1.0, lo, n))
    m = ndi.binary_dilation(key, iterations=int(round(0.2 / H2)))
    # horn local z: key end face at 0, key 6.7 long; socket from 0.4 above the face to 0.6 into the plate's shoulder gap
    return Extrude(m, lo, -0.4, 7.2, M=Y2R(side) @ G_h(side))


def roll_link(side, h=0.2):
    f = f'{side}/links/{side}_shoulder_roll_link.stl'
    T = load(REPO + f)
    sdf, lo, hf, vol = roll_fill(side, T)
    G = Grid(T, h)
    apply_grid_fill(G, sdf, lo, hf)
    from thicken import field
    tf, tlo, th, tvol = field(side, T)[:4]                            # walls thickened inward, smoothly
    apply_grid_fill(G, tf, tlo, th)
    G.apply(socket_prim(side), 'cut')
    y0 = Y0[side]
    G.apply(Cyl(0.0, y0, 30.0, -110.0, -103.15), 'cut')       # bottom face back on the joint plane
    G.apply(Cyl(0.0, y0, 14.6, -110.0, -102.9), 'cut')        # 0.3 mm over the horn plate's shoulder
    M = Y2R(side) @ G_h(side)
    key_top = apply(M, (0, 0, 0))[2]                                  # roll z of the key's end face
    for p in HORN_INS:
        w = apply(M, (p[0], p[1], 0.0))
        G.apply(Cyl(w[0], w[1], 1.7, key_top - 0.5, FLOOR_TOP + 12.0), 'cut')       # M3 clearance
        G.apply(Cyl(w[0], w[1], 3.3, FLOOR_TOP - 3.6, FLOOR_TOP + 12.0), 'cut')     # head counterbore, up through the thicker floor
    V, F = G.mesh(max_err=0.02)
    V, F = solid_only(V, F)
    info = dict(fill_cm3=vol, thicken_cm3=tvol, key_top=round(key_top, 2), **check(V, F))
    _save(f, V, F, (f'{side}/ACT/shoulder_roll/{side}_shoulder_roll_link.stl'.replace('ACT', 'actuators'),
                    f'{side}/actuators/shoulder_roll/{side}_shoulder_roll_link.stl'))
    return info


# ------------------------------------------------------------------ yaw link
def yaw_link(side, h=0.2):
    f = f'{side}/links/{side}_shoulder_yaw_link.stl'
    T = load(REPO + f)
    # fill the old horn-plate socket, its radial screw slots and holes (top plate solid again)
    hs = 0.2; lo = np.array([-36.0, -36.0, -8.0]); hi = np.array([36.0, 36.0, -0.19])
    n = np.ceil((hi - lo) / hs).astype(int) + 1
    S = occupancy(T, lo, n, hs); zs = lo[2] + hs * np.arange(n[2])
    add = slice_fill(S, zs, -8.0, -0.25, 4.0, hs)
    xs = lo[0] + hs * np.arange(n[0]); ys = lo[1] + hs * np.arange(n[1])
    X, Y = np.meshgrid(xs, ys, indexing='ij'); add &= (X ** 2 + Y ** 2 < 31.0 ** 2)[:, :, None]
    sdf, lo2 = mask_sdf(add, S, lo, hs)
    G = Grid(T, h)
    apply_grid_fill(G, sdf, lo2, hs)
    Gc = G_c(side)
    Tr0 = load(REPO + f'{side}/links/{side}_shoulder_roll_link.stl')
    _, m, mlo = roll_channel(Tr0, Y0[side])                           # cartridge cross + clearance (roll frame)
    mlo = mlo - np.array([0.0003, Y0[side]])                         # -> cartridge local
    G.apply(Extrude(m, mlo, -1.0, 37.85, M=Gc), 'cut')                # lug cross + servo housing
    G.apply(Cyl(0.0, 0.0, 20.75, -1.0, 14.2, M=Gc), 'cut')           # bearing ring
    G.apply(Box([-8.4, -2.1, 37.5], [16.0, 2.1, 39.6], M=Gc), 'cut')  # wire groove under the servo
    G.apply(Box([15.0, -5.75, 30.5], [48.0, 5.75, 37.9], M=Gc), 'cut')  # servo-plug exit channel
    for lx, ly in LUG:
        G.apply(Cyl(lx, ly, 2.0, 37.0, 44.0, M=Gc), 'cut')           # M3 heat-set insert, 6 mm
    V, F = G.mesh(max_err=0.02)
    info = check(V, F)
    _save(f, V, F, (f'{side}/actuators/shoulder_yaw/{side}_shoulder_yaw_link.stl',
                    f'{side}/actuators/shoulder_yaw/{side}_shoulder_yaw_link.stl'))
    return info


# ------------------------------------------------------------------ cartridge, horn plate
def cartridge(side):
    f = f'{side}/actuators/shoulder_yaw/{side}_shoulder_yaw_cartridge.stl'
    T = load(MIC + f)
    ops = [('fill', cartridge_column(T)[0])]
    for lx, ly in LUG:
        ops.append(('cut', Cyl(lx, ly, 3.2, 6.3, 13.0)))     # head counterbore + access, under the bearing
        ops.append(('cut', Cyl(lx, ly, 1.7, 12.5, 39.0)))    # M3 clearance through the lug column
    R, info, _ = run(T, ops, 0.1, max_err=0.02)
    os.makedirs(os.path.dirname(OUT + f), exist_ok=True); save(OUT + f, R)
    return info


def hornplate(side):
    f = f'{side}/actuators/shoulder_yaw/{side}_shoulder_yaw_hornplate.stl'
    T = load(MIC + f)
    ops = [('cut', Cyl(x, y, 2.0, -1.0, 6.0)) for x, y in HORN_INS]
    R, info, _ = run(T, ops, 0.08, max_err=0.01)
    os.makedirs(os.path.dirname(OUT + f), exist_ok=True); save(OUT + f, R)
    return info


if __name__ == '__main__':
    what, side = sys.argv[1], sys.argv[2]
    print(side, what, {'roll': roll_link, 'yaw': yaw_link, 'cart': cartridge, 'horn': hornplate}[what](side), flush=True)
