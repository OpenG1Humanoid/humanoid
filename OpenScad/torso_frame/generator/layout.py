"""Open G1 framed torso: layout (mm, torso_link frame). Single source for the generator, checks and SCAD.

Frame: 2020 aluminium, slot 6, M5 drop-in T-nuts.
  4 posts (front x=XF, back x=XB, y=+-YP), z = top of bottom plate .. underside of crossbars
  2 crossbars along y on the post tops (front shorter: the chest narrows there)
  2 top side rails + 2 lower side rails along x between front and back posts
Printed: front panel, back panel (below Z_SPLIT, split at X_SPLIT), top panel (|y| < Y_SPLIT),
  2 shoulder blocks (|y| > Y_SPLIT, carry the DS3235 shoulder-pitch servos) + their hatches,
  bottom (waist) plate, compute tray (back), driver tray (front).
"""
SKIN = 3.0                 # panel wall
BLOCK_SKIN = 4.0           # shoulder block wall
GAP = 0.5                  # seam gap between printed parts / clearance to aluminium
PROF = 20.0                # 2020
XF, XB = 44.0, -39.0       # front / back post + crossbar centre x
YP = 52.0                  # post centre |y|
Z_PLATE = (29.0, 36.0)     # bottom (waist) plate, 7 mm
Z_TOPBAR = (200.0, 220.0)  # crossbars + top side rails
Z_LOWRAIL = (60.0, 80.0)   # lower side rails
Y_BAR_F, Y_BAR_B = 70.0, 85.0   # crossbar half lengths
Y_SPLIT = 45.0
X_SPLIT = 4.0
Z_SPLIT = Z_TOPBAR[1]
PLATE = (-50.0, 57.0, -63.0, 63.0, 18.0)   # x0,x1,y0,y1,corner r

# fixings -------------------------------------------------------------------------------------------
M5 = dict(clr=5.5, head=10.4, boss=12.0)   # ISO 10642 countersunk M5
M3 = dict(clr=3.4, head=6.6, boss=9.0, insert=4.0, insert_l=5.7)
# panel -> frame screws: (part, point on the aluminium face (x,y,z), outward axis, size)
h = PROF/2
PANEL_SCREWS = []
for s in (1, -1):
    for z in (50.0, 180.0):
        PANEL_SCREWS.append(("front_panel", (XF+h, s*YP, z), (1, 0, 0), "M5"))
        PANEL_SCREWS.append(("back_panel", (XB-h, s*YP, z), (-1, 0, 0), "M5"))
    zr = sum(Z_LOWRAIL)/2
    PANEL_SCREWS.append(("front_panel", (20.0, s*(YP+h), zr), (0, s, 0), "M5"))
    PANEL_SCREWS.append(("back_panel", (-14.0, s*(YP+h), zr), (0, s, 0), "M5"))
    zp = sum(Z_PLATE)/2
    PANEL_SCREWS.append(("front_panel", (PLATE[1], s*30.0, zp), (1, 0, 0), "M3"))   # into heat-set inserts in the plate edge
    PANEL_SCREWS.append(("back_panel", (PLATE[0], s*30.0, zp), (-1, 0, 0), "M3"))
# top parts -> crossbars, vertical M5 from above (x, y)
TOP_BOLTS = [(XF, s*25.0) for s in (1, -1)] + [(XB, s*25.0) for s in (1, -1)]
BLOCK_BOLTS = [(XF, 52.0), (XF, 64.0), (XB, 55.0), (XB, 75.0)]     # left block; right = mirror
# head -> top panel: M3 screws from below into heat-set inserts in the head collar (torso x, y)
HEAD_SCREWS = [(-18.0, 22.0), (-18.0, -22.0), (22.0, 22.0), (22.0, -22.0)]
HEAD_OFFSET = (3.9635, 0.0, -44.0)     # torso <- head_link (mm)
NECK_HOLE = (-1.04, 0.0, 20.0)         # cable hole under the head bore (x, y, dia)
# electronics trays (printed, 4 mm) screwed to the post faces
TRAY_T = 4.0
TRAY_Y = 62.0
TRAY_Z = (84.0, 198.0)
COMPUTE_X = XB + h                     # tray back face on the back posts' front faces
DRIVER_X = XF - h                      # tray front face on the front posts' back faces
TRAY_SCREWS_Z = (92.0, 190.0)
# waist interface: the plate sits on the waist yaw link riser (OpenScad/legs/waist). 6 x M4 into heat-set
# inserts in the riser top, on the G1 waist yaw axis (pelvis z axis = torso x +3.9635)
WAIST_X = 3.9635
WAIST_PCD, WAIST_N, WAIST_HOLE = 50.0, 6, 4.5
PLATE_CABLE_HOLE = 24.0                  # on the axis, over the riser's centre hole
PLATE_WIRE_HOLES = [(WAIST_X, 48.0), (WAIST_X, -48.0)]   # torso <-> pelvis wiring, outside the riser
PLATE_WIRE_HOLE = 14.0


def members():
    """(name, (x0,x1,y0,y1,z0,z1), axis) for each 2020 piece"""
    M = []
    for nm, xc, yb in (("front", XF, Y_BAR_F), ("back", XB, Y_BAR_B)):
        for s, side in ((1, "L"), (-1, "R")):
            M.append((f"post_{nm}_{side}", (xc-h, xc+h, s*YP-h, s*YP+h, Z_PLATE[1], Z_TOPBAR[0]), "z"))
        M.append((f"crossbar_{nm}", (xc-h, xc+h, -yb, yb, *Z_TOPBAR), "y"))
    for s, side in ((1, "L"), (-1, "R")):
        for nm, zz in (("top", Z_TOPBAR), ("low", Z_LOWRAIL)):
            M.append((f"rail_{nm}_{side}", (XB+h, XF-h, s*YP-h, s*YP+h, *zz), "x"))
    return M
