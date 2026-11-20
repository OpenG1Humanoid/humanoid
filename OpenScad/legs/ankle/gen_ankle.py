"""Ankle pitch + roll driven by two 6512 actuators in the shin, G1-style parallel linkage.

    python gen_ankle.py            (from this folder; a few minutes)

Reads  STL/legs/left_knee_link_integrated.stl      (shin from integrate_leg.py)
       STL/legs/feet/left_ankle_roll_link.STL      (G1 foot, metres)
Writes STL/legs/left_knee_link_ankle.stl           shin with two actuator pockets (calf)
       STL/legs/left_ankle_roll_link_ears.stl      G1 foot + two rod-end ears at the heel
       STL/legs/ankle_crank.stl                    crank arm, bolts to the actuator output (print 4)
       ankle_linkage.json, ../../lib/ankle_linkage.scad   linkage geometry + linear map for the sim
Run mirror_right.py afterwards for the right leg.

Layout (left shin = knee_link frame, mm; same G1 ankle joints):
  * actuator A (upper) at (x, z) = (XM, ZU), output face at y = +Y_FACE, facing out (+y)
    actuator B (lower) at (XM, ZL), output face at y = -Y_FACE, facing out (-y)
    each pocket is the same as the waist: ring counterbore + 12 x M3 inserts, bay, drop-in stator
    carrier (print waist_stator_carrier.stl), access/wire bore out through the opposite side.
  * a crank on each output (6 x M3 on the 36 mm PCD), arm RC long, pointing back (-x).
  * a push rod (M4 rod ends, M4 threaded rod or 6 mm tube) from each crank down to an ear on the
    heel, on the same side. Both cranks turning the same way pitch the foot; opposite ways roll it.
"""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'waist'))
from meshsdf import load, mesh_sdf, zcyl, ycyl, box
sys.path.insert(0, os.path.join(HERE, '..', '..', 'pyparts'))
from solid import finalize
def to_stl(F, lo, h, path, bridge_mm=1.5, drop_loose_cm3=0.0):
    r = finalize(F, lo, h, path, bridge_mm=bridge_mm, drop_loose_cm3=drop_loose_cm3)
    print('   ', r, flush=True)
    return r['triangles'], r['volume_cm3'] * 1000
ROOT = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
STL = os.path.join(ROOT, 'STL', 'legs')

# ---------------- parameters (mm) ----------------
XM, ZU, ZL = 3.0, -60.0, -135.0        # actuator centres in the shin (x, z)
Y_FACE = 36.0                           # output faces on the shin sides
RC = 44.0                               # crank radius (swing <= 46 deg, crank-rod angle >= 44 deg over the G1 range)
CRANK_Z = (2.0, 10.0)                   # crank plate, actuator frame (LINK_GAP 2 from the output face)
BALL_ZA = 14.0                          # rod-end ball centre, actuator frame (outside the crank)
EAR = dict(x=-37.0, z=26.0, y_in=29.0, y_out=33.5, half_x=8.0, r_top=8.0)   # heel ears, roll-link frame
BALL_Y_FOOT = 38.5                      # rod-end ball centre on the ear bolt
M4_HOLE = 4.3
J_APITCH = np.array([0, -0.094445, -300.01]); J_AROLL = np.array([0, 0, -17.558])
Q_PITCH = (-0.87267, 0.5236); Q_ROLL = (-0.2618, 0.2618)    # G1 ankle ranges (rad)

# actuator-cut parameters, as gen_waist.py (output face at z = 0 in the actuator frame)
RING_R, BAY_R, Z_RINGS, Z_FLOOR = 35.2, 29.25, -29.0, -61.0
INSERT_R, INSERT_D, INSERT_DEPTH, WALL_MIN = 32.0, 4.0, 8.0, 3.0
CARRIER_SCREW_R, CARRIER_SCREW_ANG = 25.5, [45, 135, 225, 315]
CAVITY_R, CAVITY_D = 21.0, 5.0
M3_CLEAR, CBORE_D, OUT_PCD_R, BOSS_R = 3.4, 6.5, 18.0, 22.4

def U(*a): return np.minimum.reduce(a)
def grid(lo, hi, h):
    lo = np.array(lo, float); n = tuple(int(np.ceil((b - a) / h)) for a, b in zip(lo, hi))
    ax = [lo[i] + h * (np.arange(n[i]) + 0.5) for i in range(3)]
    return lo, n, ax[0][:, None, None], ax[1][None, :, None], ax[2][None, None, :]

def to_act(X, Y, Z, which):
    """shin coords -> actuator frame (z out of the output face, x = shin x)"""
    if which == 'A': return X - XM, -(Z - ZU), Y - Y_FACE
    return X - XM, Z - ZL, -Y - Y_FACE

def act_matrix(which):
    """4x4 actuator frame -> shin frame"""
    M = np.eye(4)
    if which == 'A': M[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]; M[:3, 3] = [XM, Y_FACE, ZU]
    else:            M[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]; M[:3, 3] = [XM, -Y_FACE, ZL]
    return M

def pocket(x, y, z):
    zr, zf = Z_RINGS, Z_FLOOR
    c = [zcyl(x, y, z, RING_R, zr, 1.0), zcyl(x, y, z, BAY_R, zf - 0.01, zr + 0.01),
         zcyl(x, y, z, CAVITY_R, zf - 200, zf + 0.01)]                 # access + wire bore out the far side
    for k in range(12):
        a = np.radians(30 * k); c.append(zcyl(x, y, z, INSERT_D / 2, zr - INSERT_DEPTH, zr + 0.01, INSERT_R * np.cos(a), INSERT_R * np.sin(a)))
    for a in CARRIER_SCREW_ANG:
        a = np.radians(a); c.append(zcyl(x, y, z, INSERT_D / 2, zf - 6.0, zf + 0.01, CARRIER_SCREW_R * np.cos(a), CARRIER_SCREW_R * np.sin(a)))
    return U(*c)

def sleeve(x, y, z):
    return U(zcyl(x, y, z, RING_R + WALL_MIN, Z_RINGS, 0.0), zcyl(x, y, z, BAY_R + WALL_MIN, Z_FLOOR - CAVITY_D - WALL_MIN, Z_RINGS))

# ---------------- linkage kinematics ----------------
def Ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def Rx(a): c, s = np.cos(a), np.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def foot_point(which, qp, qr):
    p = np.array([EAR['x'], BALL_Y_FOOT if which == 'A' else -BALL_Y_FOOT, EAR['z']])
    return J_APITCH + Ry(qp) @ (J_AROLL + Rx(qr) @ p)
def crank_centre(which):
    M = act_matrix(which); return M[:3, :3] @ np.array([0, 0, BALL_ZA]) + M[:3, 3]
def crank_tip(which, phi):
    """phi = crank angle about the actuator z axis (output direction), 0 = pointing back (-x)"""
    M = act_matrix(which)
    return M[:3, :3] @ np.array([-RC * np.cos(phi), -RC * np.sin(phi), BALL_ZA]) + M[:3, 3]
Q_MID = (sum(Q_PITCH) / 2, 0.0)          # cranks point straight back (phi = 0) mid-range, not at zero pitch
ROD = {w: float(np.linalg.norm(crank_tip(w, 0) - foot_point(w, *Q_MID))) for w in 'AB'}
def solve_phi(which, qp, qr, guess=None):
    """crank angle on the working branch (the root of the closure equation nearest the linear guess)"""
    f = foot_point(which, qp, qr)
    g = lambda ph: np.linalg.norm(crank_tip(which, ph) - f) - ROD[which]
    if guess is not None:                         # fast path: Newton from the linear estimate
        ph = guess
        for _ in range(30):
            v = g(ph); d = (g(ph + 1e-6) - v) / 1e-6
            if abs(d) < 1e-9: break
            ph -= v / d
            if abs(v) < 1e-9: break
        if abs(g(ph)) < 1e-6 and abs(ph - guess) < 0.5: return ph
    ph = np.linspace(-np.pi, np.pi, 721); v = np.array([g(p) for p in ph])
    roots = []
    for i in np.where(np.sign(v[:-1]) != np.sign(v[1:]))[0]:
        a, b = ph[i], ph[i + 1]
        for _ in range(60):
            m = (a + b) / 2
            if np.sign(g(m)) == np.sign(g(a)): a = m
            else: b = m
        roots.append((a + b) / 2)
    if not roots: return np.nan                   # rod too short/long: the pose is unreachable
    ref = 0.0 if guess is None else guess
    return min(roots, key=lambda r: abs(r - ref))
def transmission(which, phi, qp, qr):
    """angle between the crank arm and the rod, deg (90 = best, < 30 = poor)"""
    c = crank_centre(which); t = crank_tip(which, phi); f = foot_point(which, qp, qr)
    a, b = t - c, f - t
    return np.degrees(np.arccos(abs(a @ b) / np.linalg.norm(a) / np.linalg.norm(b)))

def linkage_report():
    J = np.zeros((2, 2))
    for i, w in enumerate('AB'):
        for j, (dp, dr) in enumerate(((1e-5, 0), (0, 1e-5))):
            J[i, j] = (solve_phi(w, dp, dr) - solve_phi(w, -dp, -dr)) / 2e-5
    worst = dict(phi=0.0, trans=90.0, unreachable=0)
    for qp in np.linspace(*Q_PITCH, 15):
        for qr in np.linspace(*Q_ROLL, 9):
            for i, w in enumerate('AB'):
                ph = solve_phi(w, qp, qr, guess=J[i] @ [qp, qr])
                if np.isnan(ph): worst['unreachable'] += 1; continue
                worst['phi'] = max(worst['phi'], abs(np.degrees(ph)))
                worst['trans'] = min(worst['trans'], transmission(w, ph, qp, qr))
    return J, worst

if __name__ == '__main__':
    J, worst = linkage_report()
    PHI0 = [solve_phi(w, 0, 0) for w in 'AB']
    rep = dict(phi_at_zero_rad=[round(p, 4) for p in PHI0], rod_length_mm={w: round(ROD[w], 1) for w in 'AB'}, jacobian_dphi_dq=J.round(4).tolist(),
               max_crank_angle_deg=round(worst['phi'], 1), min_transmission_deg=round(worst['trans'], 1),
               unreachable_poses=worst['unreachable'])
    print('linkage', rep, flush=True)

    # ---------------- shin ----------------
    H = 0.6
    T = load(os.path.join(STL, 'left_knee_link_integrated.stl'))
    lo, n, X, Y, Z = grid((-48, -42, -322), (68, 42, 46), H)
    F, _ = mesh_sdf(T, lo, n, H)
    for w in 'AB':
        x, y, z = to_act(X, Y, Z, w)
        F = np.minimum(F, sleeve(x, y, z))
    for w in 'AB':
        x, y, z = to_act(X, Y, Z, w)
        F = np.maximum(F, -pocket(x, y, z))
    # Unitree's shin has two lower-front cover plates as separate bodies, 1.7 mm off the shin (screwed on
    # in the G1). Printed, they are part of the shin: finalize() fills that gap solid.
    nt, vol = to_stl(F, lo, H, os.path.join(STL, 'left_knee_link_ankle.stl'), bridge_mm=2.5)
    rep['shin'] = dict(triangles=nt, volume_cm3=round(vol / 1000, 1)); print('shin', rep['shin'], flush=True)
    del F, X, Y, Z

    # ---------------- foot with heel ears (roll-link frame) ----------------
    H = 0.5
    T = load(os.path.join(STL, 'feet', 'left_ankle_roll_link.STL'), scale=1000.0)
    lo, n, X, Y, Z = grid((-70, -45, -40), (146, 45, 40), H)
    F, _ = mesh_sdf(T, lo, n, H)
    e = EAR
    for s in (1, -1):
        yy = s * Y
        ear = box(X, yy, Z, (e['x'] - e['half_x'], e['y_in'], 0.0), (e['x'] + e['half_x'], e['y_out'], e['z']))   # plate up to the hole
        root = box(X, yy, Z, (e['x'] - e['half_x'] - 2, 18.0, -8.0), (e['x'] + e['half_x'] + 2, e['y_out'], 12.0))   # root block down into the heel
        ear = np.minimum(ear, root)
        top = ycyl(X, yy, Z, e['x'], e['z'], e['r_top'], e['y_in'], e['y_out'])                      # round top
        F = np.minimum(F, U(ear, top))
        F = np.maximum(F, -ycyl(X, yy, Z, e['x'], e['z'], M4_HOLE / 2, e['y_in'] - 6, e['y_out'] + 1))
    # Unitree's foot mesh has a ~1 cm3 block floating 4 mm inside the toe end with no contact: deleted.
    nt, vol = to_stl(F, lo, H, os.path.join(STL, 'left_ankle_roll_link_ears.stl'), drop_loose_cm3=1.2)
    rep['foot'] = dict(triangles=nt, volume_cm3=round(vol / 1000, 1)); print('foot', rep['foot'], flush=True)
    del F, X, Y, Z

    # ---------------- crank (actuator frame, arm along -x) ----------------
    H = 0.25
    lo, n, X, Y, Z = grid((-RC - 9, -BOSS_R - 1, CRANK_Z[0] - 1), (BOSS_R + 1, BOSS_R + 1, CRANK_Z[1] + 1), H)
    z0, z1 = CRANK_Z
    F = U(zcyl(X, Y, Z, BOSS_R, z0, z1), box(X, Y, Z, (-RC, -7, z0), (0, 7, z1)), zcyl(X, Y, Z, 8.0, z0, z1, -RC, 0))
    F = np.maximum(F, -zcyl(X, Y, Z, M4_HOLE / 2, z0 - 1, z1 + 1, -RC, 0))           # rod-end bolt
    F = np.maximum(F, -zcyl(X, Y, Z, 6.0, z0 - 1, z1 + 1))                          # centre relief
    for k in range(6):
        a = np.radians(60 * k); cx, cy = OUT_PCD_R * np.cos(a), OUT_PCD_R * np.sin(a)
        F = np.maximum(F, -zcyl(X, Y, Z, M3_CLEAR / 2, z0 - 1, z1 + 1, cx, cy))
        F = np.maximum(F, -zcyl(X, Y, Z, CBORE_D / 2, z1 - 3.0, z1 + 1, cx, cy))       # M3 head counterbore
    nt, vol = to_stl(F, lo, H, os.path.join(STL, 'ankle_crank.stl'))
    rep['crank'] = dict(triangles=nt, volume_cm3=round(vol / 1000, 2)); print('crank', rep['crank'])

    # ---------------- outputs for OpenSCAD and the simulator ----------------
    rep['geometry'] = dict(XM=XM, ZU=ZU, ZL=ZL, Y_FACE=Y_FACE, RC=RC, BALL_ZA=BALL_ZA, EAR=EAR, BALL_Y_FOOT=BALL_Y_FOOT,
                           act_A=act_matrix('A').tolist(), act_B=act_matrix('B').tolist())
    json.dump(rep, open(os.path.join(HERE, 'ankle_linkage.json'), 'w'), indent=1)
    m = lambda M: '[' + ', '.join('[' + ', '.join(f'{v:.4f}' for v in r) + ']' for r in M) + ']'
    open(os.path.join(ROOT, 'OpenScad', 'lib', 'ankle_linkage.scad'), 'w').write(f'''// generated by OpenScad/legs/ankle/gen_ankle.py - left-leg values (shin = knee_link frame, mm)
ANK_ACT_A = {m(act_matrix('A'))};   // actuator frames in the shin
ANK_ACT_B = {m(act_matrix('B'))};
ANK_RC = {RC}; ANK_BALL_ZA = {BALL_ZA};
ANK_FOOT_BALL = [{EAR['x']}, {BALL_Y_FOOT}, {EAR['z']}];   // A side (+y); B side is y negated (roll-link frame)
ANK_PHI0 = [{PHI0[0]:.4f}, {PHI0[1]:.4f}];   // crank angles at ankle pitch = roll = 0 (rad)
ANK_J = {m(J)};   // d(crank angle)/d[ankle pitch, ankle roll], at zero: crank = ANK_PHI0 + ANK_J * q
''')
    print('done')
