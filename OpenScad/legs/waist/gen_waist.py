"""Waist yaw joint with the same 6512 cycloidal actuator as the legs.

    python gen_waist.py            (from this folder; about 5 minutes)

Reads  STL/legs/pelvis_{left,right}_half_integrated.stl  (the leg build, before the waist),
       STL/legs/actuator/Motor_Shell_-_Motor_Shell.stl, STL/legs/stock/waist_yaw_link_rev_1_0.stl
Writes STL/legs/pelvis_{left,right}_half_waist.stl, STL/legs/waist_stator_carrier.stl,
       STL/legs/waist_yaw_link_integrated.stl, waist_report.json

Layout (pelvis frame, mm; the G1 waist yaw axis is the pelvis z axis):
  * stator side = pelvis top disc. Ring counterbore + 12 heat-set inserts like every leg joint,
    output face flush with the pelvis top (z = Z_FACE), output facing up.
  * the stator floor is a separate drop-in STATOR CARRIER (copy of your motor-shell floor plus an
    outer ring). Screw the stator to it on the bench, drop it in, 4 x M3 countersunk into inserts.
    (Inside the solid pelvis there is no way to reach screws behind a fixed floor.)
  * wires leave the cavity under the carrier through a channel to the front of the pelvis.
  * child = waist yaw link: the G1 waist yaw disc (roll/pitch lugs removed: the 23-dof G1 locks
    them) with a riser up to the torso bottom plate. Boss + 6 x M3 on the 36 mm PCD to the output,
    hex-key bores from the riser top; 6 x M4 heat-set inserts in the riser top for the torso plate.
  * two pelvis joining bolts and both dowels moved out of the pocket; old holes filled.
"""
import json, os, sys
import numpy as np
from scipy import ndimage
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from meshsdf import load, mesh_sdf, vox_sdf, zcyl, ycyl, box, to_stl
STL = os.path.normpath(os.path.join(HERE, '..', '..', '..', 'STL', 'legs'))

# ---------------- parameters ----------------
Z_FACE = 2.2          # output face = pelvis top face
RING_R = 35 + 0.20    # ring counterbore (RING_CLEAR 0.2)
BAY_R = 29.25
Z_RINGS, Z_FLOOR = -29.0, -61.0           # actuator frame, as the legs
INSERT_R, INSERT_D, INSERT_DEPTH = 32.0, 4.0, 8.0
WALL_MIN = 3.0
CARRIER_R = 29.0      # drop-in stator carrier (bay is 29.25)
CARRIER_RING_IN = 25.6
CARRIER_SCREW_R = 25.5; CARRIER_SCREW_ANG = [45, 135, 225, 315]   # on the floor's spokes
M3_CLEAR, M3_CSK, CBORE_D = 3.4, 6.6, 6.5
CAVITY_R, CAVITY_D = 21.0, 5.0            # wire space under the carrier
CHANNEL_HALF_W = 6.0                       # wire channel to the front, 12 x 5 mm
OLD_JOIN_BOLTS = [[20, -38], [-20, -38]]; OLD_DOWELS = [[30, -50], [-30, -50]]
JOIN_BOLTS = [[38, -80], [-38, -62], [40, -40], [-40, -40]]          # first two unchanged
JOIN_DOWELS = [[41, -52], [-41, -52]]
BOLT_GRIP, DOWEL_HOLE, DOWEL_DEPTH = 6.0, 4.1, 6.0
LINK_GAP, BOSS_R, BOSS_T = 2.0, 22.4, 4.0
OUT_PCD_R, OUT_CLOCK = 18.0, 0.0
RISER_R, RISER_HOLE_R = 35.0, 12.0
TORSO_PLATE_Z = 29.0 + 44.0                 # torso bottom plate underside (layout Z_PLATE[0]) in the pelvis frame, zero waist
PLATE_PCD_R, PLATE_N, PLATE_INSERT_D, PLATE_INSERT_L = 25.0, 6, 5.6, 8.0   # M4 heat-set
z_ring_bot = Z_FACE + Z_RINGS; z_floor = Z_FACE + Z_FLOOR                 # -26.8, -58.8

def grid(lo, hi, h):
    lo = np.array(lo, float); n = tuple(int(np.ceil((b-a)/h)) for a, b in zip(lo, hi))
    ax = [lo[i]+h*(np.arange(n[i])+0.5) for i in range(3)]
    return lo, n, ax[0][:, None, None], ax[1][None, :, None], ax[2][None, None, :]

def U(*a): return np.minimum.reduce(a)
def stator_cuts(X, Y, Z):
    c = [zcyl(X, Y, Z, RING_R, z_ring_bot, Z_FACE+1), zcyl(X, Y, Z, BAY_R, z_floor-0.01, z_ring_bot+0.01),
         zcyl(X, Y, Z, CAVITY_R, z_floor-CAVITY_D, z_floor+0.01),
         box(X, Y, Z, (0, -CHANNEL_HALF_W, z_floor-CAVITY_D), (120, CHANNEL_HALF_W, z_floor))]
    for k in range(12):
        a = np.radians(30*k); c.append(zcyl(X, Y, Z, INSERT_D/2, z_ring_bot-INSERT_DEPTH, z_ring_bot+0.01, INSERT_R*np.cos(a), INSERT_R*np.sin(a)))
    for a in CARRIER_SCREW_ANG:
        a = np.radians(a); c.append(zcyl(X, Y, Z, INSERT_D/2, z_floor-6.0, z_floor+0.01, CARRIER_SCREW_R*np.cos(a), CARRIER_SCREW_R*np.sin(a)))
    return U(*c)
def sleeve(X, Y, Z):
    return U(zcyl(X, Y, Z, RING_R+WALL_MIN, z_ring_bot, Z_FACE), zcyl(X, Y, Z, BAY_R+WALL_MIN, z_floor-CAVITY_D-WALL_MIN, z_ring_bot))

def join_holes(X, Y, Z, side, bolts, dowels, grow=0.0):
    c = []
    for x, z in bolts:
        if side > 0: c.append(ycyl(X, Y, Z, x, z, INSERT_D/2+grow, -0.01, INSERT_DEPTH))
        else: c += [ycyl(X, Y, Z, x, z, M3_CLEAR/2+grow, -BOLT_GRIP, 0.01), ycyl(X, Y, Z, x, z, CBORE_D/2+grow, -600, -BOLT_GRIP)]
    for x, z in dowels: c.append(ycyl(X, Y, Z, x, z, DOWEL_HOLE/2+grow, -DOWEL_DEPTH, DOWEL_DEPTH))
    return U(*c)

rep = {}
H = 0.6
# ---------------- pelvis halves ----------------
for side, nm in ((1, 'left'), (-1, 'right')):
    T = load(os.path.join(STL, f'pelvis_{nm}_half_integrated.stl'))
    lo, n, X, Y, Z = grid((-52, -3 if side > 0 else -70, -152), (52, 70 if side > 0 else 3, 6), H)   # 3 mm past the split so the cut face is exact
    F, occ = mesh_sdf(T, lo, n, H)
    # fill the old joining holes that the pocket would leave as open bores (inside the closed envelope)
    env = ndimage.binary_closing(occ, iterations=10) | occ
    fill = np.maximum(join_holes(X, Y, Z, side, OLD_JOIN_BOLTS, OLD_DOWELS, grow=0.3), vox_sdf(env, H))
    F = np.minimum(F, fill)
    F = np.minimum(F, np.maximum(sleeve(X, Y, Z), -side*Y))                 # minimum wall round the pocket
    F = np.maximum(F, -stator_cuts(X, Y, Z))
    F = np.maximum(F, -join_holes(X, Y, Z, side, JOIN_BOLTS, JOIN_DOWELS))
    F = np.maximum(F, -side*Y)                                              # stay on this side of the split
    nt, vol = to_stl(F, lo, H, os.path.join(STL, f'pelvis_{nm}_half_waist.stl'))
    rep[f'pelvis_{nm}'] = dict(triangles=nt, volume_cm3=round(vol/1000, 1))
    print(nm, rep[f'pelvis_{nm}'], flush=True)
    del F, occ, env, fill

# ---------------- stator carrier (actuator frame -> pelvis) ----------------
Hc = 0.25
T = load(os.path.join(STL, 'actuator', 'Motor_Shell_-_Motor_Shell.stl'))
lo, n, X, Y, Z = grid((-31, -31, -33), (31, 31, -25), Hc)
F, _ = mesh_sdf(T, lo, n, Hc, band=1.0)
zt, zb = -26.0, -32.0                                           # floor in the motor shell frame
F = np.maximum(F, zcyl(X, Y, Z, CARRIER_R, zb, zt))
F = np.minimum(F, np.maximum(zcyl(X, Y, Z, CARRIER_R, zb, zt), -zcyl(X, Y, Z, CARRIER_RING_IN, zb-1, zt+1)))
for a in CARRIER_SCREW_ANG:
    a = np.radians(a); cx, cy = CARRIER_SCREW_R*np.cos(a), CARRIER_SCREW_R*np.sin(a)
    r = np.hypot(X-cx, Y-cy)
    csk = np.maximum(r-np.maximum(M3_CLEAR/2, M3_CSK/2-(zt-Z)), np.maximum(zb-1-Z, Z-zt-1))   # 90 deg countersink from the top
    F = np.maximum(F, -csk)
lo_p = lo+np.array([0, 0, Z_RINGS+Z_FACE])                       # shell z -32..-26 -> pelvis z_floor..z_floor+6
nt, vol = to_stl(F, lo_p, Hc, os.path.join(STL, 'waist_stator_carrier.stl'))
rep['waist_stator_carrier'] = dict(triangles=nt, volume_cm3=round(vol/1000, 2)); print(rep['waist_stator_carrier'])

# ---------------- waist yaw link ----------------
T = load(os.path.join(STL, 'stock', 'waist_yaw_link_rev_1_0.stl'))
lo, n, X, Y, Z = grid((-46, -46, -2), (46, 46, TORSO_PLATE_Z+2), H)
F, occ = mesh_sdf(T, lo, n, H)
# disc top: highest z of the full-radius disc
zs = Z.ravel(); col = occ[int((40-lo[0])/H), int((0-lo[1])/H), :]; ZT = float(zs[np.where(col)[0].max()]+H/2)
F = np.maximum(F, Z-ZT)                                            # drop the roll/pitch lugs (locked waist)
F = np.minimum(F, zcyl(X, Y, Z, RISER_R, ZT-1, TORSO_PLATE_Z))      # riser up to the torso bottom plate
F = np.maximum(F, -zcyl(X, Y, Z, RISER_HOLE_R, ZT+3, TORSO_PLATE_Z+1))
F = np.maximum(F, Z_FACE+LINK_GAP-Z)                               # clear of the stationary cap
F = np.minimum(F, zcyl(X, Y, Z, BOSS_R, Z_FACE, Z_FACE+LINK_GAP+1))  # boss on the output flange
for k in range(6):
    a = np.radians(OUT_CLOCK+60*k); cx, cy = OUT_PCD_R*np.cos(a), OUT_PCD_R*np.sin(a)
    F = np.maximum(F, -zcyl(X, Y, Z, M3_CLEAR/2, Z_FACE-1, Z_FACE+BOSS_T+0.01, cx, cy))
    F = np.maximum(F, -zcyl(X, Y, Z, CBORE_D/2, Z_FACE+BOSS_T, TORSO_PLATE_Z+1, cx, cy))       # hex-key bore from the riser top
    a2 = np.radians(OUT_CLOCK+30+60*k)
    F = np.maximum(F, -zcyl(X, Y, Z, 3.5, Z_FACE-1, Z_FACE+1, 17*np.cos(a2), 17*np.sin(a2)))  # pin bolt head relief
for k in range(PLATE_N):
    a = np.radians(30+360/PLATE_N*k)
    F = np.maximum(F, -zcyl(X, Y, Z, PLATE_INSERT_D/2, TORSO_PLATE_Z-PLATE_INSERT_L, TORSO_PLATE_Z+1, PLATE_PCD_R*np.cos(a), PLATE_PCD_R*np.sin(a)))
nt, vol = to_stl(F, lo, H, os.path.join(STL, 'waist_yaw_link_integrated.stl'))
rep['waist_yaw_link'] = dict(triangles=nt, volume_cm3=round(vol/1000, 1), disc_top_z=ZT); print(rep['waist_yaw_link'])
json.dump(rep, open(os.path.join(HERE, 'waist_report.json'), 'w'), indent=1)
