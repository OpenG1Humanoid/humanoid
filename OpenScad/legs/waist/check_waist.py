"""Interference checks for the waist yaw joint, on the finished STLs (0.4 mm voxels), plus a section image."""
import os, sys, json
import numpy as np
from meshsdf import load, occupancy
HERE = os.path.dirname(os.path.abspath(__file__)); R = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
S = os.path.join(R, 'STL', 'legs'); A = os.path.join(S, 'actuator')
h = 0.4; lo = np.array([-52., -52, -70]); n = (260, 260, 400)
def place(T, M): return T@M[:3, :3].T+M[:3, 3]
def Rz(a): c, s = np.cos(np.radians(a)), np.sin(np.radians(a)); M = np.eye(4); M[:2, :2] = [[c, -s], [s, c]]; return M
def Tz(z): M = np.eye(4); M[2, 3] = z; return M
def vox(T): return occupancy(T, lo, n, h)
pel = vox(np.concatenate([load(os.path.join(S, 'pelvis_left_half_waist.stl')), load(os.path.join(S, 'pelvis_right_half_waist.stl'))]))
car = vox(load(os.path.join(S, 'waist_stator_carrier.stl')))
W = Tz(2.2)
hous = vox(np.concatenate([place(load(os.path.join(A, f'Housing_-_Bambu_-_Housing_{k}.stl')), W) for k in ('Front', 'Middle', 'Rear')]))
outp = vox(np.concatenate([place(load(os.path.join(A, f'Output_-_Bambu_-_Output_Shaft_{k}.stl')), W) for k in ('Front', 'Rear')]))
link0 = load(os.path.join(S, 'waist_yaw_link_integrated.stl'))
plate = vox(load(os.path.join(R, 'STL', 'torso_frame', 'bottom_plate.stl'))+np.array([-3.9635, 0, 44]))
res = {}
def ov(a, b): return round(float((a & b).sum()*h**3), 1)
res['pelvis_vs_actuator_housing_mm3'] = ov(pel, hous)
res['pelvis_vs_actuator_output_mm3'] = ov(pel, outp)
res['pelvis_vs_carrier_mm3'] = ov(pel, car)
res['carrier_vs_housing_mm3'] = ov(car, hous)
zs = lo[2]+h*(np.arange(n[2])+.5)
kc = np.where(car.any((0, 1)))[0]; kp = zs[kc.min()]
res['carrier_bottom_z'] = round(float(zs[kc.min()]-h/2), 2)
for yaw in (0, 45, 90, 150, -150):
    L = vox(place(link0, Rz(yaw)))
    res[f'link_yaw{yaw}'] = dict(vs_pelvis_mm3=ov(L, pel), vs_housing_mm3=ov(L, hous), vs_carrier_mm3=ov(L, car), vs_output_mm3=ov(L, outp))
    if yaw == 0: L0 = L
res['link_vs_torso_plate_mm3'] = ov(L0, plate)
kl = np.where(L0.any((0, 1)))[0]; kpl = np.where(plate.any((0, 1)))[0]
res['gap_riser_top_to_plate_mm'] = round(float(zs[kpl.min()]-zs[kl.max()]-h), 2)
# boss face vs output face
res['link_lowest_z'] = round(float(zs[kl.min()]-h/2), 2)
print(json.dumps(res, indent=1)); json.dump(res, open(os.path.join(HERE, 'waist_checks.json'), 'w'), indent=1)
# section image through y = 0
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
j = n[1]//2 + 3   # y = +1.4 mm (just off the split face)
img = np.ones((n[2], n[0], 3))
for m, c in ((pel, (.72, .72, .72)), (car, (.88, .75, .38)), (hous, (.88, .48, .22)), (outp, (.36, .55, .35)), (L0, (.62, .71, .78)), (plate, (.25, .55, .85))):
    img[m[:, j, :].T] = c
fig, ax = plt.subplots(figsize=(9, 12)); ax.imshow(img[::-1], extent=[lo[0], lo[0]+n[0]*h, lo[2], lo[2]+n[2]*h])
ax.set_title('section y = 1.4 mm (pelvis frame): pelvis grey, carrier gold, housing orange,\noutput green, waist yaw link blue-grey, torso plate blue', fontsize=10)
ax.set_xlabel('x mm'); ax.set_ylabel('z mm'); fig.tight_layout(); fig.savefig(os.path.join(R, 'Assets', 'waist_section.png'), dpi=80)
