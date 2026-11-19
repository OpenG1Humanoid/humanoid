"""Generate the framed torso parts from the ORIGINAL torso_link_rev_1_0 outer surface.

python gen_torso.py            -> parts_<h>.npz (voxel parts) + report
Shell = filled original torso, minus the stock head collar (+0.5 mm), hollowed to SKIN.
"""
import sys, json, os
import numpy as np
from scipy import ndimage
import layout as L
from prims import D, prim_mask, WORK, head_stl
from voxmesh import load_stl_mm, voxelize

HEAD_STL = head_stl()
d = np.load(os.path.join(WORK, 'E_0.5.npz')); E = d['E']; lo = d['lo'].astype(float); h = float(d['h'])
n = E.shape
xs, ys, zs = (lo[i] + h*np.arange(n[i]) for i in range(3))
X = xs[:, None, None]; Y = ys[None, :, None]; Z = zs[None, None, :]
def ix(v, a): return int(np.clip(round((v - lo[a])/h), 0, n[a]-1))
def sub(x0, x1, y0, y1, z0, z1):
    return (slice(ix(x0, 0), ix(x1, 0)+1), slice(ix(y0, 1), ix(y1, 1)+1), slice(ix(z0, 2), ix(z1, 2)+1))
def subpts(sl):
    return np.meshgrid(xs[sl[0]], ys[sl[1]], zs[sl[2]], indexing='ij')
log = {}

# ---------------------------------------------------------------- solid S and distance to outside
class G: pass
g = G(); g.lo = lo; g.n = n; g.res = h
Hd = voxelize(load_stl_mm(HEAD_STL) + np.array(L.HEAD_OFFSET), g)
Hd = ndimage.binary_dilation(Hd)                     # 0.5 mm clearance round the head collar
S = E & ~Hd; del Hd
S[:, :, zs < 0] = False                              # nothing below the waist plane
Dout = (ndimage.distance_transform_edt(S)*h).astype(np.float32)
shell3 = S & (Dout <= L.SKIN)
shell4 = S & (Dout <= L.BLOCK_SKIN)
# open bottom: in the lowest slices keep only the ring (no floor)
for k in np.where(zs < 8)[0]:
    ring = ndimage.distance_transform_edt(S[:, :, k])*h <= L.SKIN
    shell3[:, :, k] &= ring; shell4[:, :, k] &= ring
print('shell done', shell3.sum()*h**3/1000, 'cm3')

# ---------------------------------------------------------------- aluminium (with clearance) + servo features
alu = np.zeros(n, bool)
for nm, b, ax_ in L.members():
    x0, x1, y0, y1, z0, z1 = b; g_ = L.GAP
    alu[sub(x0-g_, x1+g_, y0-g_, y1+g_, z0-g_, z1+g_)] = True
tops = [o for o in D.OPS if o['link'] == 'torso_link']
scut = np.zeros(n, bool); plugs = {}
reg = sub(-72, 90, -112, 112, 200, 316); P = np.stack([a.ravel() for a in subpts(reg)], 1)
shp = (reg[0].stop-reg[0].start, reg[1].stop-reg[1].start, reg[2].stop-reg[2].start)
for o in tops:
    m = prim_mask(P, o['M'], o['prim']).reshape(shp)
    if o['kind'] == 'cut': scut[reg] |= m
    if o['kind'] == 'plug':
        plugs.setdefault(o['tag'], np.zeros(n, bool))[reg] |= m
del P
anyplug = np.zeros(n, bool)
for m in plugs.values(): anyplug |= m

# ---------------------------------------------------------------- helpers for fixings
def cyl(axis, c, r, t0, t1, sign=1):
    """mask of an axis-aligned cylinder. axis 0/1/2; c = point on axis (3); t measured from c along sign*axis"""
    a = axis; o = [i for i in range(3) if i != a]
    lo_ = [c[i]-r-1 for i in range(3)]; hi_ = [c[i]+r+1 for i in range(3)]
    ta, tb = sorted((c[a]+sign*t0, c[a]+sign*t1)); lo_[a], hi_[a] = ta, tb
    sl = sub(lo_[0], hi_[0], lo_[1], hi_[1], lo_[2], hi_[2]); Q = subpts(sl)
    t = (Q[a]-c[a])*sign; rr = np.hypot(Q[o[0]]-c[o[0]], Q[o[1]]-c[o[1]])
    m = np.zeros(n, bool); m[sl] = (rr <= r) & (t >= min(t0, t1)) & (t <= max(t0, t1)); return m
def cone(axis, c, sign, t_top, r_top, r_bot, depth):
    """countersink: radius r_top at t_top, shrinking 1:1 inwards (towards -t) to r_bot"""
    a = axis; o = [i for i in range(3) if i != a]; r = r_top
    lo_ = [c[i]-r-1 for i in range(3)]; hi_ = [c[i]+r+1 for i in range(3)]
    ta, tb = sorted((c[a]+sign*(t_top-depth), c[a]+sign*(t_top+3))); lo_[a], hi_[a] = ta, tb
    sl = sub(lo_[0], hi_[0], lo_[1], hi_[1], lo_[2], hi_[2]); Q = subpts(sl)
    t = (Q[a]-c[a])*sign; rr = np.hypot(Q[o[0]]-c[o[0]], Q[o[1]]-c[o[1]])
    rad = np.maximum(r_bot, r_top - (t_top - t))
    m = np.zeros(n, bool); m[sl] = (rr <= rad) & (t >= t_top-depth); return m
def exit_t(axis, c, sign, tmax=80):
    """distance along the axis from c to where the solid S ends (outer surface)"""
    last = None
    for t in np.arange(0, tmax, h/2):
        p = list(c); p[axis] += sign*t
        if S[ix(p[0], 0), ix(p[1], 1), ix(p[2], 2)]: last = t
        elif last is not None and t - last > 3: break
    return last

# ---------------------------------------------------------------- regions
gp = L.GAP/2
R_front = (X > L.X_SPLIT+gp) & (Z < L.Z_SPLIT-gp)
R_back = (X < L.X_SPLIT-gp) & (Z < L.Z_SPLIT-gp)
R_top = (np.abs(Y) < L.Y_SPLIT-gp) & (Z > L.Z_SPLIT+gp)
R_blkL = (Y > L.Y_SPLIT+gp) & (Z > L.Z_SPLIT+gp)
R_blkR = (Y < -L.Y_SPLIT-gp) & (Z > L.Z_SPLIT+gp)
parts = {}
parts['front_panel'] = shell3 & R_front
parts['back_panel'] = shell3 & R_back
parts['top_panel'] = shell3 & R_top

# panel screws ---------------------------------------------------------------------------------------
screws = []
for part, p, ax3, size in L.PANEL_SCREWS:
    a = int(np.argmax(np.abs(ax3))); sg = int(np.sign(ax3[a])); F = L.M5 if size == 'M5' else L.M3
    to = exit_t(a, p, sg)
    reg_ = R_front if part == 'front_panel' else R_back
    boss = cyl(a, p, F['boss']/2, L.GAP, to + 1, sg) & S & reg_
    # pick a standard length that reaches 3-6 mm past the aluminium face (T-nut / insert), recess the head if needed
    STD = {'M5': [10, 12, 14, 16, 18, 20, 22, 25, 30, 35, 40, 45], 'M3': [6, 8, 10, 12, 14, 16, 18, 20, 25]}[size]
    grip0 = to - L.GAP
    Ls = max(l for l in STD if l <= grip0 + 6.0); recess = max(0.0, grip0 + 3.5 - Ls)   # 3.5..6 mm past the face
    tt = to - recess                                        # countersink top
    hole = cyl(a, p, F['clr']/2, -1, to + 3, sg) | cone(a, p, sg, tt, F['head']/2, F['clr']/2, (F['head']-F['clr'])/2 + 0.5)
    if recess > 0: hole |= cyl(a, p, F['head']/2, tt, to + 3, sg)
    parts[part] = (parts[part] | boss) & ~hole
    screws.append(dict(part=part, at=p, axis=ax3, size=size, boss_len=round(to, 1), grip=round(grip0 - recess, 1),
                       screw=f"{size}x{Ls} countersunk", recess=round(recess, 1), past_face=round(Ls - (grip0 - recess), 1)))
log['panel_screws'] = screws

# top panel: pillars down to the crossbars, head screws, neck cable hole ------------------------------
top = parts['top_panel']
zb = L.Z_TOPBAR[1]
for (x, y) in L.TOP_BOLTS:
    c = (x, y, zb)
    pil = cyl(2, c, 7.0, L.GAP, 120, 1) & S & R_top
    top |= pil
    top &= ~cyl(2, c, L.M5['clr']/2, -1, 11, 1)          # M5 x 16 through a 10 mm base into the T-nut
    top &= ~cyl(2, c, 10.5/2, 10.5, 140, 1)              # counterbore / access from above (under the head)
for (x, y) in L.HEAD_SCREWS:
    # socket floor top at this (x, y)
    col = S[ix(x, 0), ix(y, 1), :]; ktop = np.where(col & (zs > 240))[0].max(); zt = zs[ktop]
    top |= cyl(2, (x, y, zt), L.M3['boss']/2, 0, 10, -1) & S
    top &= ~cyl(2, (x, y, zt), L.M3['clr']/2, -2, 12, -1)
    log.setdefault('head_screws', []).append(dict(at=(x, y), floor_top_z=round(float(zt), 1)))
xn, yn, dn = L.NECK_HOLE
top &= ~cyl(2, (xn, yn, 230), dn/2, 0, 100, 1)
parts['top_panel'] = top

# shoulder blocks + hatches ---------------------------------------------------------------------------
for side, s, R in (('L', 1, R_blkL), ('R', -1, R_blkR)):
    tag = 'pitch' + side
    src = (scut | plugs[tag]) & R
    cradle = np.zeros(n, bool)
    cradle[reg] = ndimage.binary_dilation(src[reg], iterations=10) & S[reg]       # 5 mm round the servo
    blk = (shell4 | cradle) & R
    # bar pads (8 mm) run from the servo cradle out to the front / back skin so they tie into the block
    for (x0, x1, y1) in ((22.0, 120.0, L.Y_BAR_F), (-120.0, -22.0, L.Y_BAR_B)):
        ya, yb = sorted((s*(L.Y_SPLIT+gp), s*y1))
        pad = np.zeros(n, bool); pad[sub(x0, x1, ya, yb, zb+L.GAP, zb+8)] = True
        blk |= pad & S
    for (x, y) in L.BLOCK_BOLTS:
        c = (x, s*y, zb)
        blk &= ~cyl(2, c, L.M5['clr']/2, -1, 9, 1)        # M5 x 14 through the 8 mm pad
        blk &= ~cyl(2, c, 10.5/2, 8, 30, 1)               # head clearance above the pad
    blk &= ~scut & ~anyplug
    parts['shoulder_block_' + side] = blk
    parts['hatch_' + side] = S & plugs[tag] & ~scut & R

# remove aluminium clearance from all printed parts, keep the main connected piece --------------------
for k in list(parts):
    m = parts[k] & ~alu
    lab, nl = ndimage.label(m)
    if nl > 1:
        sizes = ndimage.sum(m, lab, range(1, nl+1)); keep = np.argmax(sizes) + 1
        small = sizes.sum() - sizes.max()
        log.setdefault('dropped_bits_cm3', {})[k] = round(float(small*h**3/1000), 2)
        for j in range(nl):
            if j+1 != keep and sizes[j]*h**3/1000 > 0.3:
                sl = ndimage.find_objects((lab == j+1).astype(np.int8))[0]
                log.setdefault('dropped_bits', []).append((k, round(float(sizes[j]*h**3/1000), 2),
                    [round(float(lo[a]+sl[a].start*h), 1) for a in range(3)], [round(float(lo[a]+sl[a].stop*h), 1) for a in range(3)]))
        m = lab == keep
    parts[k] = m

# ---------------------------------------------------------------- bottom plate + trays (simple printed parts)
def rrect2d(x0, x1, y0, y1, r):
    cx = np.clip(X, x0+r, x1-r); cy = np.clip(Y, y0+r, y1-r)
    return np.hypot(X-cx, Y-cy) <= r
x0, x1, y0, y1, r = L.PLATE
plate = rrect2d(x0, x1, y0, y1, r) & (Z >= L.Z_PLATE[0]) & (Z <= L.Z_PLATE[1])
for xc in (L.XF, L.XB):
    for s in (1, -1):
        c = (xc, s*L.YP, L.Z_PLATE[0])
        plate &= ~cyl(2, c, L.M5['clr']/2, -1, 10, 1)
        plate &= ~cone(2, c, -1, 0, L.M5['head']/2, L.M5['clr']/2, 3.0)     # countersunk from below into the tapped post ends
for i in range(L.WAIST_N):
    a = 2*np.pi*i/L.WAIST_N + np.pi/L.WAIST_N
    plate &= ~cyl(2, (L.WAIST_PCD/2*np.cos(a), L.WAIST_PCD/2*np.sin(a), L.Z_PLATE[0]), L.WAIST_HOLE/2, -1, 10, 1)
plate &= ~cyl(2, (0, 0, L.Z_PLATE[0]), L.PLATE_CABLE_HOLE/2, -1, 10, 1)
zp = sum(L.Z_PLATE)/2
for part, p, ax3, size in L.PANEL_SCREWS:
    if size == 'M3':
        a = int(np.argmax(np.abs(ax3))); sg = int(np.sign(ax3[a]))
        plate &= ~cyl(a, p, L.M3['insert']/2, 0, L.M3['insert_l']+1, -sg)     # heat-set insert in the plate edge
parts['bottom_plate'] = plate

def tray(xa, xb, slots):
    t = (X >= xa) & (X <= xb) & (np.abs(Y) <= L.TRAY_Y) & (Z >= L.TRAY_Z[0]) & (Z <= L.TRAY_Z[1])
    for s in (1, -1):
        for z in L.TRAY_SCREWS_Z:
            t &= ~((np.hypot(Y-s*L.YP, Z-z) <= L.M5['clr']/2))
    return t & slots
zc = sum(L.TRAY_Z)/2
# compute tray: horizontal slots |y| 18..38 at z = zc +-29 (Raspberry Pi 5: 58 x 49) and zc +-43 (86 mm boards);
# vent windows between
cs = np.ones(n, bool)
for dz in (-43, -29, 29, 43):
    cs &= ~((np.abs(Y) >= 18) & (np.abs(Y) <= 38) & (np.abs(Z-(zc+dz)) <= 1.45))
cs &= ~((np.abs(Y) <= 12) & (np.abs(Z-zc) <= 24))                              # cable / airflow window
parts['compute_tray'] = tray(L.COMPUTE_X, L.COMPUTE_X+L.TRAY_T, cs)
ds = np.ones(n, bool)
gy = np.arange(-35, 36, 10.0); gz = np.arange(zc-50, zc+51, 10.0)
for yy in gy:
    for zz in gz:
        ds &= ~(np.hypot(Y-yy, Z-zz) <= 1.45)                                  # 10 mm grid of 2.9 mm holes (M3 self-tap / M2.5)
parts['driver_tray'] = tray(L.DRIVER_X-L.TRAY_T, L.DRIVER_X, ds)

# ---------------------------------------------------------------- report
BED = (220, 220, 250)
rep = {}
for k, m in parts.items():
    idx = np.where(m.any((1, 2)))[0], np.where(m.any((0, 2)))[0], np.where(m.any((0, 1)))[0]
    bb = [float((v.max()-v.min()+1)*h) for v in idx]
    import itertools
    fits = any(all(b <= B for b, B in zip(perm, BED)) for perm in itertools.permutations(bb))
    rep[k] = dict(vol_cm3=round(float(m.sum()*h**3/1000), 1), bbox_mm=[round(b, 1) for b in bb], fits_220x220x250=fits)
log['parts'] = rep
for k, v in rep.items(): print(f"{k:18s} {v}")
json.dump(log, open(os.path.join(WORK, 'report.json'), 'w'), indent=1, default=float)
np.savez_compressed(os.path.join(WORK, f'parts_{h}.npz'), lo=lo, h=h, **parts)
np.savez_compressed(os.path.join(WORK, 'aux.npz'), alu=alu, scut=scut, anyplug=anyplug)
