"""Arm fixes: (1) horn pockets for real cross horns, (2) cable path at the spline end of every servo cartridge.
All geometry edits are CSG on the existing STLs (see meshedit.py). Run: python jobs.py [filter]"""
import sys, os, json, shutil, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from meshedit import *
from probe import ray_hits
from joints import parts_in_cart, xf
from stlio import load, save

SRC = os.environ.get('ARMS_SRC', os.path.join(HERE, 'original_arms')) + '/'
OUT = os.environ.get('ARMS_OUT', os.path.join(HERE, 'out')) + '/'

# ---------------------------------------------------------------- horn pocket shapes (mm)
# Pocket = round hub + 4 tapered arms. Arm slots run out through the rim of the horn plates,
# so long-arm / short-arm (asymmetric) cross horns both drop in; only width/hub/depth must fit.
HORN = {
    'std':   dict(hub_d=14.0, w_root=9.2, w_tip=7.2, depth=3.0),   # 25T standard-servo cross horn
    'micro': dict(hub_d=8.0,  w_root=6.0, w_tip=4.6, depth=2.2),   # SG90/MG90S cross horn
}
OPEN_REACH = 30.0       # arm slots run through the plate rim
ELBOW_REACH = 21.0      # elbow disc pocket is closed (disc face is visible), arms stop here

# ---------------------------------------------------------------- cable path sizes (mm)
WIRE = {'std': dict(w=6.0, h=3.05, groove=2.0), 'micro': dict(w=5.2, h=3.05, groove=1.6)}


def horn_arms(kind, reach, angle0=0.0):
    H = HORN[kind]
    return [(angle0 + k * 90.0, reach, H['w_root'], H['w_tip']) for k in range(4)]


def first_pos(h):
    h = h[h > 1e-6]
    return h[0] if len(h) else None


# ================================================================= horn plates
PLATES = [('shoulder_roll', 'hornplate', 'std'), ('shoulder_pitch', 'hornhub', 'std'),
          ('shoulder_yaw', 'hornplate', 'micro'), ('wrist_roll', 'hornplate', 'micro'),
          ('wrist_pitch', 'hornplate', 'micro'), ('wrist_yaw', 'hornplate', 'micro')]


def plate_job(side, joint, name, kind):
    f = f'{side}/actuators/{joint}/{side}_{joint}_{name}.stl'
    T = load(SRC + f)
    top = T[:, :, 2].max()
    floor = top + 5 - ray_hits(T, [6.0001, 0.00013, top + 5], [0, 0, -1])[0]
    zm = (top + floor) / 2
    old_reach = first_pos(ray_hits(T, [0, 0.0001, zm], [1, 0, 0]))
    old_reach = max(old_reach, first_pos(ray_hits(T, [0.0001, 0, zm], [0, 1, 0])))
    hole_r = first_pos(ray_hits(T, [0, 0, floor - 0.5], [1, 0.00013, 0]))
    H = HORN[kind]
    ops = [('fill', Cyl(0, 0, old_reach + 0.6, floor - 0.02, top, ri=hole_r + 0.02)),
           ('cut', Horn(H['hub_d'], horn_arms(kind, OPEN_REACH), top - H['depth'], top + 1.0))]
    meta = dict(top=round(top, 3), old_floor=round(floor, 3), new_floor=round(top - H['depth'], 3),
                old_reach=round(old_reach, 2), centre_hole_r=round(hole_r, 2))
    return f, T, ops, meta, 0.08


def elbow_link_job(side):
    f = f'{side}/links/{side}_elbow_link.stl'
    T = load(SRC + f)
    sg = 1.0 if side == 'left' else -1.0
    # disc inner face and old pocket floor along the joint axis (world y)
    face = sg * (ray_hits(T, [8.0001, -sg * 10, 0.00013], [0, sg, 0])[0] - 10)
    pfloor = sg * (ray_hits(T, [5.0001, -sg * 10, 5.0013], [0, sg, 0])[0] - 10)
    # local frame: origin on axis at the face, local z into the disc
    ez = np.array([0, sg, 0.]); ex = np.array([1., 0, 0]); ey = np.cross(ez, ex)
    M = np.eye(4); M[:3, 0] = ex; M[:3, 1] = ey; M[:3, 2] = ez; M[:3, 3] = [0, face, 0]
    depth = abs(pfloor - face)
    # old arms point at 34 deg in the world x-z plane -> local angle
    wdir = np.array([np.cos(np.radians(34)), 0, np.sin(np.radians(34))])
    a_loc = np.degrees(np.arctan2(wdir @ ey, wdir @ ex))
    H = HORN['std']
    ops = [('fill', Cyl(0, 0, 17.8 + 0.6, -0.0, depth + 0.02, M=M, ri=4.0 + 0.02)),
           ('cut', Horn(H['hub_d'], horn_arms('std', ELBOW_REACH, a_loc), -1.0, depth, M=M))]
    meta = dict(face_y=round(face, 3), pocket_depth=round(depth, 3), arm_angle_local=round(a_loc, 2))
    return f, T, ops, meta, 0.15


# ================================================================= cable paths (cartridge coordinates)
def cart_frame_ops(side, joint, part, cuts_in_cart):
    """re-express cartridge-frame cutters in the frame of another part of the same joint."""
    P = parts_in_cart(side, joint)
    if part == 'cartridge':
        return cuts_in_cart
    Mp = P[part][1]            # part -> cartridge
    Mi = np.linalg.inv(Mp)     # cartridge -> part
    out = []
    for mode, p in cuts_in_cart:
        p2 = _retarget(p, Mi)
        out.append((mode, p2))
    return out


def _retarget(p, Mi):
    import copy
    q = copy.copy(p)
    if isinstance(p, (Subtract,)):
        q.a = _retarget(p.a, Mi); q.b = _retarget(p.b, Mi); return q
    if isinstance(p, (Intersect,)):
        q.ps = [_retarget(x, Mi) for x in p.ps]; return q
    q.M = Mi @ p.M; q.Mi = np.linalg.inv(q.M); return q


def axis_of(side, joint):
    from asm import mods
    D = mods(side)
    for name, d in D.items():
        for pn, Pm in d['parts'].items():
            if pn != f'{side}_{joint}_cartridge': continue
            cands = [(np.zeros(3), d['v'])]
            if d['child']:
                c = D[d['child']]; cands.append((c['M'][:3, 3], c['M'][:3, :3] @ c['v']))
            R = Pm[:3, :3]; t = Pm[:3, 3]
            for o, dv in cands:
                oc = R.T @ (o - t); dc = R.T @ dv
                if abs(abs(dc[2]) - 1) < 1e-3: return oc[:2]
    raise ValueError(joint)


def yoke_cable(side, joint, kind, r_hole, r_bearing, band, entry=None, cap_box=None):
    """Yoke joints: servo bottom sits on the cap, right over the idler bearing bore.
    Channel: (entry through the spline-end wall, with 0.6 mm skin over the bore) -> arc round the bore
    inside the cartridge -> groove in the cap under the far end of the servo -> existing exit slot."""
    ax, ay = axis_of(side, joint)
    s = 1.0 if ay < 0 else -1.0       # arc goes round the side away from the axis offset
    W = WIRE[kind]
    r0, r1, a0, a1 = band
    pts = sector_pts(0, 0, r0, r1, a0, a1)
    pts = np.c_[ax + pts[:, 0], ay + s * pts[:, 1]]
    cart = [('cut', Poly(pts, -0.5, W['h']))]
    if entry is not None:
        x0, x1, hw = entry
        cart.append(('cut', Box([x0, ay - hw, 0.6], [x1, ay + hw, W['h']])))
    capc = []
    if cap_box is not None:
        x0, x1, y0, y1 = cap_box
        ylo, yhi = sorted([ay + s * y0, ay + s * y1])
        capc.append(('cut', Subtract(Box([x0, ylo, -W['groove']], [x1, yhi, 0.5]),
                                     Cyl(ax, ay, r_bearing + 0.25, -20, 20))))
    return cart, capc


def floor_cable(side, joint, kind, x_end_out, x_far, z_floor, notch_h, hw, depth):
    """Closed-floor cartridges (elbow, wrist roll): notch through the spline-end wall at the servo bottom
    plus a groove in the floor to the existing exit slot at the far end."""
    ax, ay = axis_of(side, joint)
    W = WIRE[kind]
    cart = [('cut', Box([x_end_out, ay - hw, z_floor - notch_h], [x_end_out + 2.4, ay + hw, z_floor + 0.1])),
            ('cut', Box([x_end_out, ay - hw, z_floor - 0.1], [x_far, ay + hw, z_floor + depth]))]
    return cart


def cable_jobs(side):
    J = {}
    # shoulder roll (standard servo, yoke): ear slot at the spline end is open to the cap
    c, k = yoke_cable(side, 'shoulder_roll', 'std', 11.3, 13.05, band=(12.2, 18.2, 178, 12),
                      cap_box=(3.0, 31.0, -3.5, 6.5))
    J['shoulder_roll'] = dict(cartridge=c, cap=k)
    # wrist pitch / wrist yaw (micro, yoke)
    for jn in ['wrist_pitch', 'wrist_yaw']:
        ax, ay = axis_of(side, jn)
        c, k = yoke_cable(side, jn, 'micro', 11.34, 13.05, band=(11.8, 16.0, 182, 18),
                          entry=(ax - 12.4, ax - 6.1, 2.6), cap_box=(9.0, 17.0, -1.5, 7.5))
        J[jn] = dict(cartridge=c, cap=k)
    # shoulder yaw (micro): servo bottom against the back cap; notch + back-cap groove
    ax, ay = axis_of(side, 'shoulder_yaw')
    W = WIRE['micro']; hw = W['w'] / 2
    J['shoulder_yaw'] = dict(
        cartridge=[('cut', Box([-8.2, ay - hw, 34.6], [-6.0, ay + hw, 37.8]))],
        backcap=[('cut', Box([-8.2, ay - hw, 37.4], [17.6, ay + hw, 37.6 + W['groove']]))])
    # elbow (standard): closed floor at z 38.6
    J['elbow'] = dict(cartridge=floor_cable(side, 'elbow', 'std', -22.6, 21.6, 38.6, 3.6, WIRE['std']['w'] / 2, 1.5))   # floor is 2.5 thick
    # wrist roll (micro): closed floor at z 23.6
    J['wrist_roll'] = dict(cartridge=floor_cable(side, 'wrist_roll', 'micro', -8.3, 18.4, 23.6, 3.0, WIRE['micro']['w'] / 2, 1.5))
    return J


def cable_part_jobs(side):
    out = []
    for joint, parts in cable_jobs(side).items():
        for part, cuts in parts.items():
            f = f'{side}/actuators/{joint}/{side}_{joint}_{part}.stl'
            T = load(SRC + f)
            ops = cart_frame_ops(side, joint, part, cuts)
            out.append((f, T, ops, dict(joint=joint, part=part), 0.1))
    return out


def all_jobs(flt=''):
    jobs = []
    for side in ['left', 'right']:
        for j, n, k in PLATES:
            jobs.append(('plate', side, lambda side=side, j=j, n=n, k=k: plate_job(side, j, n, k)))
        jobs.append(('elbowlink', side, lambda side=side: elbow_link_job(side)))
        jobs.append(('cable', side, lambda side=side: cable_part_jobs(side)))
    return jobs


def _disc(r, h, n=128):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False); P = np.c_[r * np.cos(a), r * np.sin(a)]; T = []
    for i in range(n):
        p, q = P[i], P[(i + 1) % n]
        T += [[[0, 0, 0], [q[0], q[1], 0], [p[0], p[1], 0]], [[0, 0, h], [p[0], p[1], h], [q[0], q[1], h]],
              [[p[0], p[1], 0], [q[0], q[1], 0], [q[0], q[1], h]], [[p[0], p[1], 0], [q[0], q[1], h], [p[0], p[1], h]]]
    return np.array(T, float)


def fit_tests():
    """small discs with just the horn pocket, to check your horns before printing the real parts"""
    for kind, r, h, hole in [('std', 22, 5.0, 3.75), ('micro', 15, 4.0, 2.5)]:
        H = HORN[kind]
        ops = [('cut', Cyl(0, 0, hole, -1, h + 1)),
               ('cut', Horn(H['hub_d'], horn_arms(kind, OPEN_REACH), h - H['depth'], h + 1))]
        R, info, G = run(_disc(r, h), ops, 0.08)
        os.makedirs(OUT + 'fit_tests', exist_ok=True)
        save(OUT + f'fit_tests/horn_pocket_test_{kind}.stl', R)
        print(f'fit_tests/horn_pocket_test_{kind}.stl', info)


def run_one(f, T, ops, meta, h, log):
    R, info, G = run(T, ops, h, max_err=0.02)
    os.makedirs(os.path.dirname(OUT + f), exist_ok=True)
    save(OUT + f, R)
    info.update(meta)
    print(f, info, flush=True)
    log[f] = {k: (float(v) if isinstance(v, (np.floating,)) else v) for k, v in info.items()}
    return R


if __name__ == '__main__':
    flt = sys.argv[1] if len(sys.argv) > 1 else ''
    os.makedirs(OUT, exist_ok=True)
    if flt in ('', 'fittest'):
        fit_tests()
    logf = os.path.join(OUT, 'joblog.json')
    log = json.load(open(logf)) if os.path.exists(logf) else {}
    for kind, side, fn in all_jobs():
        if flt and flt not in f'{kind}:{side}': continue
        res = fn()
        items = res if isinstance(res, list) else [res]
        for f, T, ops, meta, h in items:
            if len(sys.argv) > 2 and sys.argv[2] not in f: continue
            run_one(f, T, ops, meta, h, log)
            if f.endswith('_elbow_link.stl') and '/links/' in f:
                # the print-oriented copy in actuators/elbow/ is the same mesh, only translated
                side = f.split('/')[0]
                A = load(SRC + f).reshape(-1, 3)
                B = load(SRC + f'{side}/actuators/elbow/{side}_elbow_link.stl').reshape(-1, 3)
                t = (B - A).mean(0)
                save(OUT + f'{side}/actuators/elbow/{side}_elbow_link.stl', load(OUT + f) + t)
        json.dump(log, open(logf, 'w'), indent=1)
