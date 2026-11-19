"""
Integrate 6512 cycloidal actuators into the Unitree G1 left leg.

Joints done:  hip pitch (pelvis -> hip pitch link)
              hip roll  (hip pitch link -> hip roll link)
              knee      (hip yaw link / thigh -> knee link / shin)

Every joint uses the same pattern:
  Parent link (stator side): the motor shell becomes part of the link. Gear rings drop
      into a counterbore and clamp with 12x M3 into heat-set inserts. Stator floor copied
      from Motor_Shell. Access bore behind the floor for stator screws and wiring.
  Child link (output side): stock interface removed within LINK_GAP of the face, boss
      bolts to Output_Shaft_Front with 6x M3 on 36 PCD, access bores for a hex key.

Usage:  python integrate_leg.py <folder with your STLs> <output folder> [scale]
Needs:  pip install trimesh manifold3d numpy rtree
All in mm. G1 meshes are scaled about their own link origins, so multiply every URDF joint
origin by SCALE and the joint frames still line up.
"""
import sys, os, numpy as np, trimesh, manifold3d as m3d
from collections import Counter, defaultdict

IN  = sys.argv[1] if len(sys.argv) > 1 else "."
OUT = sys.argv[2] if len(sys.argv) > 2 else "./out"
SCALE = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0    # 1:1 with the G1 URDF

# ---------------- design parameters (mm) ----------------
RING_CLEAR   = 0.20    # radial clearance, gear rings in counterbore
BAY_R        = 29.25   # motor bay radius (original cage posts at r=29.0)
INSERT_D     = 4.0     # M3 heat-set insert hole
INSERT_DEPTH = 8.0
CAVITY_R     = 30.0    # access/wiring bore behind each stator floor
HATCH_HALF_H = 28.0    # pelvis rear hatch half-height
LINK_GAP     = 2.0     # gap between a child link and the stationary cap / parent face
BOSS_R       = 22.4    # child boss radius (output flange is r=22.5)
BOSS_INTO    = 6.0     # how far the boss runs into the child's existing plate
M3_CLEAR     = 3.4
CBORE_D      = 6.5
BOSS_T       = 4.0
WALL_MIN     = 3.0     # minimum wall around every actuator pocket (adds material where the stock link is thinner)
Z_RINGS, Z_FLOOR = -29.0, -61.0     # actuator frame: z out of the face, z=0 on the joint plane

# pelvis split: when the two hip pitch stator floors are closer than SPLIT_GAP the pelvis is made in
# two halves split on the centre plane so the stator screws can be reached from the split face
SPLIT_GAP    = 40.0
JOIN_BOLTS   = [(38, -80), (-38, -62), (20, -38), (-20, -38)]   # pelvis (x, z) at 1:1, scaled with SCALE
JOIN_DOWELS  = [(30, -50), (-30, -50)]
BOLT_GRIP    = 6.0     # right-half material under each joining bolt head (M3x12 into an insert)
DOWEL_HOLE   = 4.1     # for 4 mm dowel pins
DOWEL_DEPTH  = 6.0     # each side

# ---------------- mesh helpers ----------------
def fill_holes_fan(m):
    m = m.copy(); m.merge_vertices()
    c = Counter(map(tuple, m.edges_sorted))
    be = [tuple(x) for x in m.edges if c[tuple(sorted(x))] == 1]
    if not be: return m
    out = defaultdict(list)
    for a, b in be: out[a].append(b)
    used = set(); V = list(m.vertices); F = [list(f) for f in m.faces]
    for e0 in be:
        if e0 in used: continue
        loop = [e0]; used.add(e0); cur = e0[1]
        while cur != e0[0]:
            nb = [b for b in out[cur] if (cur, b) not in used]
            if not nb: break
            e = (cur, nb[0]); used.add(e); loop.append(e); cur = nb[0]
        # a hole boundary can pass through the same vertex twice (figure-8); split it into simple loops
        verts = [a for a, _ in loop]; stack, loops = [], []
        for v in verts:
            if v in stack:
                i = stack.index(v); loops.append(stack[i:]); stack = stack[:i + 1]
            else:
                stack.append(v)
        if len(stack) > 2: loops.append(stack)
        for lp in loops:
            if len(lp) < 3: continue
            ci = len(V); V.append(np.mean([m.vertices[a] for a in lp], axis=0))
            for a, b in zip(lp, lp[1:] + lp[:1]): F.append([b, a, ci])
    return trimesh.Trimesh(np.array(V), np.array(F), process=False)

def to_mf(t):
    t = t.copy(); t.merge_vertices()
    return m3d.Manifold(m3d.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))

def to_tm(mf):
    me = mf.to_mesh()
    return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=False)

def load_mf(name, g1=False):
    """Load an STL as one clean solid. The stock G1 meshes are several overlapping shells
    (plus some zero-volume scraps and one inside-out shell), so each shell is fixed and
    then unioned rather than used as-is."""
    t = trimesh.load(os.path.join(IN, name))
    if g1: t.apply_scale(1000.0 * SCALE)
    t = fill_holes_fan(t); t.merge_vertices()
    solid = None
    for c in t.split(only_watertight=False):
        if not c.is_watertight or abs(c.volume) < 1.0:
            continue                                   # scraps
        if c.volume < 0:
            c.invert()                                 # inside-out shell
        m = to_mf(c)
        if m.status() != m3d.Error.NoError or m.volume() <= 0:
            continue
        solid = m if solid is None else solid + m
    return solid if solid is not None else to_mf(t)

MIN_PIECE = 500.0   # mm3; detached boolean debris smaller than this is dropped from finished parts

def clean(mf):
    """Finished part as a trimesh with boolean debris removed (zero-thickness sheets left where two
    stock shells touched). Works on the vertex-merged mesh, the way a slicer reads the STL.
    Real separate bodies, like the shin's ankle push rods, are far bigger and are kept."""
    t = to_tm(mf); t.merge_vertices()
    keep = [c for c in t.split(only_watertight=False) if c.volume > MIN_PIECE]
    return trimesh.util.concatenate(keep)

def cyl(r, z0, z1, seg=128):
    return m3d.Manifold.cylinder(z1 - z0, r, r, seg).translate([0, 0, z0])

def box(lo, hi):
    lo, hi = np.array(lo, float), np.array(hi, float)
    return m3d.Manifold.cube(list(hi - lo)).translate(list(lo))

def xf(mf, T):
    return mf.transform(np.asarray(T)[:3, :])

def Tr(v):
    T = np.eye(4); T[:3, 3] = v; return T

def Ry(a):
    c, s = np.cos(a), np.sin(a); T = np.eye(4); T[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]; return T

def Rz_deg(a):
    c, s = np.cos(np.radians(a)), np.sin(np.radians(a)); T = np.eye(4); T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; return T

def urdf(xyz, pitch=0.0):
    return Tr(1000 * SCALE * np.array(xyz)) @ Ry(pitch)

def rot(cols):
    T = np.eye(4); T[:3, :3] = np.array(cols, float).T; return T

# actuator z onto a joint axis (right-handed)
AZ_TO_X = rot([[0, 1, 0], [0, 0, 1], [1, 0, 0]])     # act x->Jy, act y->Jz, act z->Jx
AZ_TO_Y = rot([[1, 0, 0], [0, 0, -1], [0, 1, 0]])    # act x->Jx, act y->-Jz, act z->Jy

# G1 23dof URDF joint frames (left leg)
J_PITCH = urdf([0, 0.064452, -0.1027])                       # in pelvis
J_ROLL  = urdf([0, 0.052, -0.030465], -0.1749)               # in hip pitch link
J_YAW   = urdf([0.025001, 0, -0.12412])                      # in hip roll link
J_KNEE  = urdf([-0.078273, 0.0021489, -0.17734], 0.1749)     # in hip yaw link

# ---------------- stator side ----------------
def stator_cut():
    c = cyl(35.0 + RING_CLEAR, Z_RINGS, 1.0) + cyl(BAY_R, Z_FLOOR - 0.01, Z_RINGS + 0.01)
    for k in range(12):
        a = np.radians(30 * k)
        c = c + cyl(INSERT_D / 2, Z_RINGS - INSERT_DEPTH, Z_RINGS + 0.01, 32).translate([32 * np.cos(a), 32 * np.sin(a), 0])
    return c

def insert_keepout():          # used to keep other bores away from the inserts and stator floor
    k = cyl(BAY_R + 1.0, Z_FLOOR - 1.0, Z_FLOOR + 7.0)
    for i in range(12):
        a = np.radians(30 * i)
        k = k + cyl(INSERT_D / 2 + 1.0, Z_RINGS - INSERT_DEPTH - 1.0, Z_RINGS + 0.01, 32).translate([32 * np.cos(a), 32 * np.sin(a), 0])
    return k

_FLOOR = None
def stator_floor():
    global _FLOOR
    if _FLOOR is None:
        shell = load_mf("Motor_Shell_-_Motor_Shell.stl")
        _FLOOR = (shell ^ cyl(BAY_R + 0.05, -32.01, -25.99)).translate([0, 0, Z_RINGS])
    return _FLOOR

def sleeve():
    return cyl(35.0 + RING_CLEAR + WALL_MIN, Z_RINGS, 0.0) + cyl(BAY_R + WALL_MIN, Z_FLOOR, Z_RINGS)

def add_stator(link, T_act):
    """Integrate an actuator's stationary side into a link. T_act: actuator frame -> link frame."""
    Ti = np.linalg.inv(T_act)
    a = xf(link, Ti) + sleeve()
    a = a - stator_cut() - cyl(CAVITY_R, -600, Z_FLOOR + 0.01)    # access bore out the back
    a = a + stator_floor()
    return xf(a, T_act)

# ---------------- output side ----------------
def axis_start(link_act, z0=LINK_GAP, z1=200):
    t = to_tm(link_act); zs = np.arange(z0, z1, 0.25)
    inside = t.contains(np.c_[np.zeros_like(zs), np.zeros_like(zs), zs])
    return zs[inside].min() if inside.any() else None

def bolt_xy(clock):
    return [(18 * np.cos(np.radians(clock + 60 * k)), 18 * np.sin(np.radians(clock + 60 * k))) for k in range(6)]

def bolt_cutter(clock):
    h = None
    for x, y in bolt_xy(clock):
        b = cyl(M3_CLEAR / 2, -1, BOSS_T + 0.01, 32).translate([x, y, 0]) + cyl(CBORE_D / 2, BOSS_T, 800, 48).translate([x, y, 0])
        h = b if h is None else h + b
    for k in range(6):
        a = np.radians(clock + 30 + 60 * k)
        h = h + cyl(3.5, -1, 1.0, 32).translate([17 * np.cos(a), 17 * np.sin(a), 0])
    return h

def access_lengths(link_act_tm, clock):
    L = []
    for x, y in bolt_xy(clock):
        zs = np.arange(BOSS_T, 800, 0.5)
        inside = link_act_tm.contains(np.c_[np.full_like(zs, x), np.full_like(zs, y), zs])
        L.append((zs[inside].max() if inside.any() else BOSS_T) - BOSS_T)
    return L

def add_output(link, T_act, r_clear, score=None):
    """Turn a child link's stock interface into a boss on Output_Shaft_Front.
    score(clock) -> number to minimise; default = longest access bore."""
    Ti = np.linalg.inv(T_act)
    a = xf(link, Ti)
    z_plate = axis_start(a)                                     # where the stock plate meets the axis
    a = a - cyl(r_clear, -600, LINK_GAP)                        # clear the stationary cap / parent face
    a = a + cyl(BOSS_R, 0, max(z_plate, LINK_GAP) + BOSS_INTO)
    ta = to_tm(a)
    if score is None:
        score = lambda c: max(access_lengths(ta, c))
    clock = min(range(0, 60, 5), key=score)
    a = a - bolt_cutter(clock)
    return xf(a, T_act), clock, access_lengths(ta, clock), z_plate

# ---------------- builds ----------------
def act_to_pelvis(side):
    T = np.eye(4)
    if side == "L":
        T[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]; T[:3, 3] = J_PITCH[:3, 3]
    else:
        T[:3, :3] = [[-1, 0, 0], [0, 0, -1], [0, -1, 0]]; T[:3, 3] = J_PITCH[:3, 3] * [1, -1, 1]
    return T

def ycyl(x, z, r, y0, y1):
    return m3d.Manifold.cylinder(y1 - y0, r, r, 48).rotate([-90, 0, 0]).translate([x, y0, z])

def build_pelvis():
    """Returns {"pelvis": solid} or {"pelvis_left_half": ..., "pelvis_right_half": ...}."""
    pel = load_mf("pelvis.STL", g1=True)
    for s in ("L", "R"):
        pel = pel + xf(sleeve(), act_to_pelvis(s))
    for s in ("L", "R"):
        pel = pel - xf(stator_cut(), act_to_pelvis(s))
    y_floor = J_PITCH[1, 3] + Z_FLOOR
    zc = J_PITCH[2, 3]
    pel = pel - ycyl(0, zc, CAVITY_R, -y_floor, y_floor) - box([-500, -y_floor, zc - HATCH_HALF_H], [0, y_floor, zc + HATCH_HALF_H])
    for s in ("L", "R"):
        pel = pel + xf(stator_floor(), act_to_pelvis(s))
    if 2 * y_floor >= SPLIT_GAP:
        return {"pelvis": pel}
    L = pel ^ box([-500, 0, -500], [500, 500, 500])
    R = pel ^ box([-500, -500, -500], [500, 0, 500])
    for x, z in JOIN_BOLTS:
        x, z = x * SCALE, z * SCALE
        R = R - ycyl(x, z, M3_CLEAR / 2, -BOLT_GRIP, 0.01) - ycyl(x, z, CBORE_D / 2, -600, -BOLT_GRIP)
        L = L - ycyl(x, z, INSERT_D / 2, -0.01, INSERT_DEPTH)
    for x, z in JOIN_DOWELS:
        d = ycyl(x * SCALE, z * SCALE, DOWEL_HOLE / 2, -DOWEL_DEPTH, DOWEL_DEPTH)
        L, R = L - d, R - d
    return {"pelvis_left_half": L, "pelvis_right_half": R}

def face_along(link_mf, J, axis, r_max):
    """Furthest extent of a parent link along a joint axis, near the axis (stock geometry)."""
    t = to_tm(xf(link_mf, np.linalg.inv(J)))
    v = t.vertices; ax = {"x": 0, "y": 1}[axis]
    r = np.linalg.norm(np.delete(v, ax, axis=1), axis=1)
    return v[r < r_max, ax].max()

def roll_act_T(clock, d_face):
    return J_ROLL @ Tr([d_face, 0, 0]) @ AZ_TO_X @ Rz_deg(clock)

def knee_act_T(clock, d_face):
    return J_KNEE @ Tr([0, d_face, 0]) @ AZ_TO_Y @ Rz_deg(clock)

def build_all():
    os.makedirs(OUT, exist_ok=True)
    res = {}
    # ---- pelvis (hip pitch stator x2)
    res["pelvis_parts"] = build_pelvis()

    # ---- hip pitch link: pitch output + roll stator
    pit0 = load_mf("left_hip_pitch_link.STL", g1=True)
    d_roll = face_along(pit0, J_ROLL, "x", 45 * SCALE)
    T_pitch_out = AZ_TO_Y                         # pitch link frame = pitch joint frame, output face at y=0
    # choose the roll actuator clocking and pitch bolt clocking together so the pitch link's
    # hex-key bores stay clear of the roll inserts and roll stator floor
    best = None
    for rc in range(0, 90, 5):
        T_ra = roll_act_T(rc, d_roll)
        keep = xf(insert_keepout(), np.linalg.inv(T_pitch_out) @ T_ra)
        for pc in range(0, 60, 5):
            hit = (bolt_cutter(pc) ^ keep).volume()
            if best is None or hit < best[0] - 1e-6:
                best = (hit, rc, pc)
        if best[0] < 1e-6: break
    hit, roll_clock, pitch_clock = best
    T_ra = roll_act_T(roll_clock, d_roll)
    pit = add_stator(pit0, T_ra)
    a = xf(pit, np.linalg.inv(T_pitch_out))
    z_plate = axis_start(xf(pit0, np.linalg.inv(T_pitch_out)))
    a = a - box([-500, -500, -600], [500, 500, LINK_GAP])
    a = a + cyl(BOSS_R, 0, z_plate + BOSS_INTO)
    pitch_len = access_lengths(to_tm(xf(pit0, np.linalg.inv(T_pitch_out))), pitch_clock)
    a = a - bolt_cutter(pitch_clock)
    pit = xf(a, T_pitch_out)
    res["pitch"] = pit
    res["info_pitch"] = dict(roll_face=d_roll, roll_clock=roll_clock, pitch_clock=pitch_clock, bore_hits=hit, pitch_bore_len=pitch_len)

    # ---- hip roll link: roll output
    rol0 = load_mf("left_hip_roll_link.STL", g1=True)
    T_ra_child = Tr([d_roll, 0, 0]) @ AZ_TO_X
    rol, rclk, rlen, rplate = add_output(rol0, T_ra_child, r_clear=48 * SCALE)
    res["roll"] = rol; res["info_roll"] = dict(clock=rclk, bore_len=rlen, plate=rplate)

    # ---- thigh (hip yaw link): knee stator
    yaw0 = load_mf("left_hip_yaw_link.STL", g1=True)
    d_knee = face_along(yaw0, J_KNEE, "y", 30 * SCALE)
    T_ka = knee_act_T(0, d_knee)
    res["yaw"] = add_stator(yaw0, T_ka)
    res["info_knee"] = dict(knee_face=d_knee)

    # ---- knee link (shin): knee output
    kne0 = load_mf("left_knee_link.STL", g1=True)
    T_ka_child = Tr([0, d_knee, 0]) @ AZ_TO_Y
    kne, kclk, klen, kplate = add_output(kne0, T_ka_child, r_clear=41 * SCALE)
    res["knee"] = kne; res["info_knee"].update(clock=kclk, bore_len=klen, plate=kplate)
    res["T"] = dict(roll=T_ra, knee=T_ka)
    return res

if __name__ == "__main__":
    r = build_all()
    names = {"pitch": "left_hip_pitch_link_integrated.stl", "roll": "left_hip_roll_link_integrated.stl",
             "yaw": "left_hip_yaw_link_knee_integrated.stl", "knee": "left_knee_link_integrated.stl"}
    parts = {k + "_integrated": v for k, v in r["pelvis_parts"].items()}
    parts.update({names[k][:-4]: r[k] for k in names})
    for n, m in parts.items():
        t = clean(m); t.export(os.path.join(OUT, n + ".stl"))
        print(f"{n + '.stl':42s} {t.volume/1000:6.0f} cm3 (solid), {len(t.split(only_watertight=False))} bodies, watertight {t.is_watertight}")
    for k in ("info_pitch", "info_roll", "info_knee"):
        print(k, {a: (np.round(b, 1) if not isinstance(b, int) else b) for a, b in r[k].items()})
