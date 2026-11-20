"""Build every printable ARM part directly in Python (no OpenSCAD), from the same design.py that
drives the OpenSCAD library, and write them through pyparts/solid.finalize(), so every STL is checked
to be one closed piece with no floating plastic.

    python build_arm_stls.py                 # everything (about 20-30 minutes)
    python build_arm_stls.py elbow_link      # only parts whose name contains this text
    python build_arm_stls.py --side left

Writes STL/arms_print/<side>_<part>.stl and STL/arms_print/hardware/<part>.stl:
  links (servo bores, sockets, spigots, screw holes), wrist plugs and caps, servo cartridges and
  their caps (left and right are mirror images), hubs and the idler pin.
The right arm uses Unitree's right-hand meshes (as the OpenSCAD library does).
"""
import json, os, sys, time
import numpy as np
from scipy.spatial import ConvexHull
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'OpenScad', 'pyparts'))
import design as D
from solid import load, mesh_sdf, prim_sdf, finalize, grid
ARMS = os.path.join(ROOT, 'STL', 'arms'); OUT = os.path.join(ROOT, 'STL', 'arms_print')
SHELLS = json.load(open(os.path.join(HERE, 'shells.json')))
H_BIG, H_SMALL, H_CART, H_HW = 0.4, 0.3, 0.25, 0.15
FILL_LINKS = {'shoulder_roll_link'}   # Unitree mesh is a thin hollow shell: printed solid (cuts give the walls)
SKIN_GROW = 0.6                       # how far thin skins are grown, each side (mm)
MIN_SKIN = 2.0                        # Unitree skins thinner than this are thickened to it (before the cuts)
NO_THICKEN = {'wrist_roll_link', 'wrist_yaw_link'}   # thin covers by design: the wrist pitch link sits inside them
MIN_WALL = 0.8                        # skins thinner than this are removed (not on the cosmetic caps)
MY = np.diag([1.0, -1.0, 1.0, 1.0])


# ------------------------------------------------------------------ meshes in the LEFT link frame (mm)
def link_meshes(link, side):
    """list of triangle arrays; the right side's own meshes are mirrored into the left frame"""
    if link == 'shoulder_yaw_link':
        Ts = [load(os.path.join(ARMS, n)) * 1000 for n in SHELLS[f'{side}_shoulder_yaw_link']]
    else:
        f = f'{side}_shoulder_roll_link_fixed.stl' if link == 'shoulder_roll_link' else f'{side}_{link}.STL'
        Ts = [load(os.path.join(ARMS, f)) * 1000]
    if side == 'right':
        Ts = [mirror_tris(T) for T in Ts]
    return Ts


def mirror_tris(T):
    T = T.copy(); T[:, :, 1] *= -1; return T[:, ::-1]


def hand_tris(side):
    T = load(os.path.join(ARMS, f'{side}_hand_clean.stl')) * 1000
    if side == 'right': T = mirror_tris(T)
    return T + np.array([-84.0, 0, 0])            # hand frame -> wrist_yaw frame


# ------------------------------------------------------------------ helpers
def ops_of(link, kinds):
    return [o for o in D.OPS if o['link'] == link and o['kind'] in kinds]


def prim_bbox(p):
    """local bounding box of a primitive (for the grid extent)"""
    k = p[0]
    if k in ('cyl', 'dcyl'): r, z0, z1 = p[1], p[2], p[3]; return [-r, -r, min(z0, z1)], [r, r, max(z0, z1)]
    if k == 'ann': r, z0, z1 = p[2], p[3], p[4]; return [-r, -r, min(z0, z1)], [r, r, max(z0, z1)]
    if k == 'cone': r = max(p[1], p[2]); return [-r, -r, min(p[3], p[4])], [r, r, max(p[3], p[4])]
    if k == 'box': return [p[1], p[3], min(p[5], p[6])], [p[2], p[4], max(p[5], p[6])]
    if k == 'boxr': r = max(abs(p[2]), abs(p[3])) + max(abs(p[4]), abs(p[5])); return [-r, -r, min(p[6], p[7])], [r, r, max(p[6], p[7])]
    if k == 'cylat': return [p[1] - p[3], p[2] - p[3], min(p[4], p[5])], [p[1] + p[3], p[2] + p[3], max(p[4], p[5])]
    return None


def world_bbox(o):
    b = prim_bbox(o['prim'])
    if b is None: return None
    c = np.array([[x, y, z] for x in (b[0][0], b[1][0]) for y in (b[0][1], b[1][1]) for z in (b[0][2], b[1][2])])
    w = c @ o['M'][:3, :3].T + o['M'][:3, 3]; return w.min(0), w.max(0)


def eval_ops(ops, P, combine):
    """union (min) of the SDFs of a list of ops at points P (n,3); +inf if none"""
    out = np.full(len(P), np.inf, np.float32)
    for o in ops:
        Mi = np.linalg.inv(o['M']); Q = P @ Mi[:3, :3].T + Mi[:3, 3]
        out = np.minimum(out, prim_sdf(o['prim'], Q[:, 0], Q[:, 1], Q[:, 2]).astype(np.float32))
    return out


def field_on_grid(lo, n, h, fn, slab=24):
    """evaluate fn(P) -> (len(P),) over the grid in x-slabs (memory)"""
    out = np.empty(n, np.float32)
    ys = lo[1] + h * (np.arange(n[1]) + 0.5); zs = lo[2] + h * (np.arange(n[2]) + 0.5)
    Y, Z = np.meshgrid(ys, zs, indexing='ij')
    for i0 in range(0, n[0], slab):
        xs = lo[0] + h * (np.arange(i0, min(i0 + slab, n[0])) + 0.5)
        P = np.stack([np.repeat(xs, Y.size), np.tile(Y.ravel(), len(xs)), np.tile(Z.ravel(), len(xs))], 1)
        out[i0:i0 + len(xs)] = fn(P).reshape(len(xs), n[1], n[2])
    return out


def hull_sdf_fn(V):
    hull = ConvexHull(V); E = hull.equations.astype(np.float32)   # a.x + b <= 0 inside, |a| = 1
    def f(P):
        out = np.empty(len(P), np.float32)
        for s in range(0, len(P), 20000):
            out[s:s + 20000] = (P[s:s + 20000].astype(np.float32) @ E[:, :3].T + E[:, 3]).max(1)
        return out
    return f


def mirrored(F, lo, n, h):
    """the same field, mirrored in y (left part -> right part)"""
    lo2 = np.array(lo, float); lo2[1] = -(lo[1] + h * n[1])
    return np.ascontiguousarray(F[:, ::-1, :]), lo2


def write(F, lo, h, name, report, **kw):
    path = os.path.join(OUT, name)
    kw.setdefault('min_wall', 0.0 if '_cap_w' in name else MIN_WALL)
    r = finalize(F, lo, h, path, **kw)
    report[name] = {k: r[k] for k in ('triangles', 'volume_cm3', 'crumbs_removed', 'voids_filled', 'bridged', 'dropped', 'shells')}
    print(f'  {name:46s} {r["volume_cm3"]:8.2f} cm3  {r["triangles"]:7d} tris  one piece: {r["shells"] == 1}'
          + (f'  bridged {r["bridged"]}' if r['bridged'] else '') + (f'  dropped {r["dropped"]}' if r['dropped'] else ''), flush=True)


# ------------------------------------------------------------------ links (+ their plugs and caps)
def build_link(link, side, report, want):
    t0 = time.time()
    meshes = link_meshes(link, side)
    hand = hand_tris(side) if link == 'wrist_yaw_link' else None
    allV = np.concatenate([T.reshape(-1, 3) for T in meshes])
    lo_b, hi_b = allV.min(0), allV.max(0)
    if hand is not None:
        Hv = hand.reshape(-1, 3); Hv = Hv[Hv[:, 0] >= D.HAND_TRIM_X - 84 - 1]
        lo_b = np.minimum(lo_b, Hv.min(0)); hi_b = np.maximum(hi_b, Hv.max(0))
    for o in ops_of(link, ('add', 'ring', 'addhull')):
        b = world_bbox(o)
        if b: lo_b = np.minimum(lo_b, b[0]); hi_b = np.maximum(hi_b, b[1])
    h = H_SMALL if link.startswith('wrist') else H_BIG
    lo, n, _ = grid(lo_b - 3, hi_b + 3, h)
    # body: union of the mesh shells (+ the trimmed hand)
    F = None
    for T in meshes:
        Fi, occ = mesh_sdf(T, lo, n, h); F = Fi if F is None else np.minimum(F, Fi)
        if link in FILL_LINKS:              # thin hollow shell in Unitree's mesh: use its filled outer shape
            from scipy import ndimage
            env = ndimage.binary_closing(occ, structure=ndimage.generate_binary_structure(3, 1), iterations=int(round(3.0 / h)))
            env = ndimage.binary_fill_holes(env | occ)
            for ax in range(3):            # also fill pockets open on one side, slice by slice along each axis
                env |= np.stack([ndimage.binary_fill_holes(np.take(env, k, axis=ax)) for k in range(env.shape[ax])], axis=ax)
            # filled shape kept 1 mm INSIDE the true surface: it fills the hollows but never replaces the
            # real (smooth) outer surface, which stays the exact mesh distance
            from solid import vox_sdf
            F = np.minimum(F, vox_sdf(env, h) + 1.0)
    if hand is not None:
        Fh, _ = mesh_sdf(hand, lo, n, h)
        xs = (lo[0] + h * (np.arange(n[0]) + 0.5)).astype(np.float32)[:, None, None]
        F = np.minimum(F, np.maximum(Fh, (D.HAND_TRIM_X - 84) - xs))
        pin = dict(M=np.array([[0, 0, -1, 99.5], [1, 0, 0, 1.888], [0, -1, 0, -10], [0, 0, 0, 1.0]]) @ np.array(
            [[1, 0, 0, 10.4829], [0, 1, 0, 22.95], [0, 0, 1, 7.59], [0, 0, 0, 1.0]]), prim=('cyl', 0.85, 0.0, 8.01))
        F = np.minimum(F, field_on_grid(lo, n, h, lambda P: eval_ops([pin], P, min)))
    # bodyplus: material added inside the link's own outline (bosses) and plain additions (sleeves)
    ah = ops_of(link, ('addhull',))
    if ah:
        hs = hull_sdf_fn(np.concatenate([T.reshape(-1, 3) for T in meshes]))
        F = np.minimum(F, field_on_grid(lo, n, h, lambda P: np.maximum(eval_ops(ah, P, min), hs(P))))
    ad = ops_of(link, ('add',))
    if ad: F = np.minimum(F, field_on_grid(lo, n, h, lambda P: eval_ops(ad, P, min)))
    # thin skins in Unitree's meshes (cosmetic shrouds, ~1 mm) do not print well: grow every skin thinner
    # than MIN_SKIN to at least MIN_SKIN, before the joint cuts, so bores/pockets still land exactly
    if link not in NO_THICKEN:
        # smooth thickening: offset the exact surface outwards by up to SKIN_GROW, blended in only around
        # skins thinner than MIN_SKIN (no voxel edits on the surface, so it stays smooth)
        from scipy import ndimage
        st = ndimage.generate_binary_structure(3, 1)
        solid = F < 0
        k = max(1, int(round(MIN_SKIN / (2 * h))))
        thin = solid & ~ndimage.binary_opening(solid, structure=st, iterations=k)
        w = ndimage.gaussian_filter(ndimage.binary_dilation(thin, structure=st, iterations=k + 1).astype(np.float32), sigma=1.5 / h)
        w = np.clip(w * 2.0, 0, 1)
        F = (F - SKIN_GROW * w).astype(np.float32)
        print(f'    {link}: thin skins thickened (up to {SKIN_GROW} mm each side)', flush=True)
    bodyplus = F
    cuts = ops_of(link, ('cut',)); hatches = ops_of(link, ('plug', 'cap')); rings = ops_of(link, ('ring',))
    Fc = field_on_grid(lo, n, h, lambda P: eval_ops(cuts, P, min)) if cuts else None
    Fh = field_on_grid(lo, n, h, lambda P: eval_ops(hatches, P, min)) if hatches else None
    main = bodyplus.copy()
    if Fc is not None: main = np.maximum(main, -Fc)
    if Fh is not None: main = np.maximum(main, -Fh)
    if rings: main = np.minimum(main, field_on_grid(lo, n, h, lambda P: eval_ops(rings, P, min)))
    parts = [(f'{side}_{link}.stl', main)]
    for o in hatches:                               # removable hatch / cosmetic cap pieces
        Fo = field_on_grid(lo, n, h, lambda P: eval_ops([o], P, min))
        Fp = np.maximum(bodyplus, Fo)
        if o['kind'] == 'plug' and Fc is not None: Fp = np.maximum(Fp, -Fc)
        parts.append((f'{side}_{link}_{o["kind"]}_{o["tag"]}.stl', Fp))
    for name, Fp in parts:
        if want and want not in name: continue
        if side == 'right':
            Fm, lom = mirrored(Fp, lo, n, h); write(Fm, lom, h, name, report)
        else:
            write(Fp, lo, h, name, report)
    print(f'    ({link} {side}: {time.time() - t0:.0f} s)', flush=True)


# ------------------------------------------------------------------ cartridges (same geometry as emit.py writes for OpenSCAD)
def cart_fields(t, g):
    Rc, zf, zr, B, fl = g['Rc'], g['zf'], g['zr'], g['B'], g['flat']
    ys = (g['by0'] - Rc) / 2; xs = min(0.42 * Rc, np.sqrt(max((Rc - 4.5) ** 2 - ys ** 2, 1)))
    if fl: xs = min(xs, fl[0] - 4.5)
    scr = [(xs, ys), (-xs, ys)]; ca = g['cable_ang']; S25 = D.M25; S3 = D.M3
    def shp(r, z0, z1): return ('dcyl', r, z0, z1, fl[0] + (r - Rc), fl[1]) if fl else ('cyl', r, z0, z1)
    I = np.eye(4)
    def rot(a): M = np.eye(4); c, s = np.cos(np.radians(a)), np.sin(np.radians(a)); M[:2, :2] = [[c, -s], [s, c]]; return M
    def o(prim, M=I): return dict(M=M, prim=prim)
    # cartridge
    add = [o(shp(Rc, zf if g['mode'] == 'front' else g['ct'] - 0.3, zr))]
    if g['mode'] != 'front': add.append(o(('cyl', g['r_nose'], zf, g['ct'] - 0.2)))
    if g['mode'] == 'front':
        add += [o(('box', Rc - 1.8, Rc + 5.4, -4.2, 4.2, zf, zf + 3.8), rot(a)) for a in g['lug_angles']]
    cut = [o(('cyl', B['od'] / 2 + 0.05, zf - 1, zf + B['w'])),
           o(('cyl', g['r_in'], zf + B['w'] - 0.01, g['ct'] + 0.01)),
           o(('box', -g['W'] / 2, g['W'] / 2, g['by0'], g['by1'], g['ct'], zr + 1)),
           o(('box', 0, Rc + 1, -2.6, 2.6, zr - 3, zr + 0.1), rot(ca))]
    if g['mode'] == 'front':
        cut.append(o(('box', max(Rc - 3, B['od'] / 2 + 1.0), Rc + 1, -2.6, 2.6, zf - 1, zr + 1), rot(ca)))
        for a in g['lug_angles']:
            M = rot(a) @ np.array([[1, 0, 0, Rc + 2.4], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
            cut += [o(('cyl', S25['clr'] / 2, zf - 1, zf + 5), M), o(('cyl', (S25['head'] + 0.4) / 2, zf - 1, zf + S25['head_h']), M)]
    for x, y in scr: cut.append(o(('cylat', x, y, S25['ins'] / 2, zr - S25['ins_l'], zr + 0.01)))
    # cap
    if g['mode'] == 'front':
        zc = zr + 3.0; cadd = [o(shp(Rc, zr, zc))]
        ccut = [o(('box', Rc - 4, Rc + 1, -2.6, 2.6, zr - 1, zr + 4), rot(ca))]
        for x, y in scr: ccut += [o(('cylat', x, y, S25['clr'] / 2, zr - 1, zr + 5)), o(('cylat', x, y, (S25['head'] + 0.4) / 2, zc - 1.8, zc + 1.2))]
        zext = (zr - 1, zc + 1)
    else:
        L = g['host_len']
        cadd = [o(shp(Rc, zr, L)), o(('dcyl', Rc + 1.5, L - 2.95, L, fl[0] + 1.5, fl[1]) if fl else ('cyl', Rc + 1.5, L - 2.95, L))]
        cadd += [o(('box', Rc - 1.8, Rc + 5.4, -4.2, 4.2, L - 3.9, L), rot(a)) for a in g['cap_lug_angles']]
        ccut = [o(('cyl', 11.05, L - 7.3, L + 0.7)), o(('cyl', 5.0, L - 9.5, L - 7.2)),
                o(('box', Rc - 4, Rc + 1, -2.6, 2.6, zr - 1, L + 1), rot(ca))]
        for x, y in scr: ccut += [o(('cylat', x, y, S25['clr'] / 2, zr - 1, L + 1)), o(('cylat', x, y, (S25['head'] + 0.4) / 2, zr + 5, L + 1))]
        for a in g['cap_lug_angles']:
            M = rot(a) @ np.array([[1, 0, 0, Rc + 2.4], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
            ccut += [o(('cyl', S3['clr'] / 2, L - 9, L + 1), M), o(('cyl', 3.1, L - 1.9, L + 1.1), M)]
        zext = (zr - 1, L + 1)
    R = Rc + 7
    return (add, cut, (-R, -R, zf - 1.5), (R, R, zr + 1.5)), (cadd, ccut, (-R, -R, zext[0]), (R, R, zext[1]))


def build_cartridge(t, g, report, want, sides):
    for kind, (add, cut, lo_b, hi_b) in zip(('cartridge', 'cartridge_cap'), cart_fields(t, g)):
        if want and not any(want in f'{side}_{kind}_{t}.stl' for side in sides): continue
        h = H_CART; lo, n, _ = grid(lo_b, hi_b, h)
        F = field_on_grid(lo, n, h, lambda P: np.maximum(eval_ops(add, P, min), -eval_ops(cut, P, min)))
        for side in sides:
            name = f'{side}_{kind}_{t}.stl'
            if want and want not in name: continue
            if side == 'right':
                Fm, lom = mirrored(F, lo, n, h); write(Fm, lom, h, name, report)
            else:
                write(F, lo, h, name, report)


# ------------------------------------------------------------------ hubs + idler pin (as emit.py's hub_solid / hub_outer / idler_pin)
def hub_ops(name):
    def o(prim, M=np.eye(4)): return dict(M=M, prim=prim)
    def rz(a, x=0.0, y=0.0, z=0.0): M = np.eye(4); c, s = np.cos(np.radians(a)), np.sin(np.radians(a)); M[:2, :2] = [[c, -s], [s, c]]; M[:3, 3] = [x * c, x * s, z]; return M
    if name == 'idler_pin':
        return [o(('cyl', 4.5, -6.4, -5.3)), o(('cyl', 4.95 / 2, -5.3, 7.0))], [], (-6, -6, -7), (6, 6, 8)
    if name == 'hub_MG92B_cover':
        S = D.SERVO['MG92B']; d0, d1, ct = D.stack(S); hexc = S['hex'] / np.cos(np.pi / 6); head = hexc + 4; w = -8.0
        add = [o(('cyl', (head - 0.2) / 2, w + 0.1, w + 1.6)), o(('hex', S['hex'], w + 1.6, d1))]
        cut = [o(('cyl', 1.2, w - 1, w + 29)), o(('cyl', 2.1, w - 1, w + 1.2)), o(('cyl', (S['horn'] + 0.3) / 2, d1 - 1, d1 + 1))]
        cut += [o(('cylat', 4.0 * np.cos(np.radians(a)), 4.0 * np.sin(np.radians(a)), 0.85, d1 - 4, d1 + 1)) for a in (0, 180)]
        return add, cut, (-head, -head, w - 1), (head, head, d1 + 1)
    S = D.SERVO['MG92B' if 'MG92B' in name else name.split('_')[1]]
    d0, d1, ct = D.stack(S); hexc = S['hex'] / np.cos(np.pi / 6); hubr = max(S['horn'] / 2, hexc / 2) + 1.2; L = S['hexlen']
    sc = {3: (3.4, 2.5, 6.0), 2: (2.4, 1.7, 4.2)}[S['scr']]; pr = S['horn'] * 0.34
    add = [o(('hex', S['hex'], -L, d0 + 0.01)), o(('cyl', hubr - 0.8, d0, d1))]
    cut = [o(('cyl', (sc[0] + 0.4) / 2, -L - 1, d1 + 1)), o(('cyl', (sc[2] + 0.4) / 2, -L - 1, d0 - 2)),
           o(('cyl', (S['horn'] + 0.3) / 2, d1 - 1, d1 + 1))]
    cut += [o(('cylat', pr * np.cos(np.radians(a)), pr * np.sin(np.radians(a)), sc[1] / 2, d1 - 4, d1 + 1)) for a in (45, 135, 225, 315)]
    cut += [o(('rad', a, -L / 2 - 0.5, 0, 30, sc[1])) for a in range(30, 360, 60)]
    R = hubr + 1
    return add, cut, (-R, -R, -L - 1), (R, R, d1 + 1)


def build_hardware(report, want):
    for name in ('hub_DS3235', 'hub_X10PRO', 'hub_MG92B_wristroll', 'hub_MG92B_cover', 'idler_pin'):
        if want and want not in name: continue
        add, cut, lo_b, hi_b = hub_ops(name)
        h = H_HW; lo, n, _ = grid(lo_b, hi_b, h)
        F = field_on_grid(lo, n, h, lambda P: np.maximum(eval_ops(add, P, min), -eval_ops(cut, P, min) if cut else -np.inf))
        write(F, lo, h, os.path.join('hardware', f'{name}.stl'), report)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    want = args[0] if args else ''
    sides = ['left', 'right']
    if '--side' in sys.argv: sides = [sys.argv[sys.argv.index('--side') + 1]]; want = '' if want in sides else want
    os.makedirs(os.path.join(OUT, 'hardware'), exist_ok=True)
    report = {}
    t0 = time.time()
    for side in sides:
        for link in D.LINKS:
            if want and want not in link and not any(want in f'{side}_{link}_{x}' for x in ('plug', 'cap')): continue
            build_link(link, side, report, want)
    for t, g in D.CART.items():
        build_cartridge(t, g, report, want, sides)
    build_hardware(report, want)
    json.dump(report, open(os.path.join(OUT, 'build_report.json'), 'w'), indent=1, default=float)
    print(f'{len(report)} parts, all single closed pieces, in {time.time() - t0:.0f} s -> {OUT}')


if __name__ == '__main__':
    main()
