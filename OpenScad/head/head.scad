// =====================================================================
//  Unitree G1 dome head, Intel RealSense D455, removable side panels
// =====================================================================
//  Units: millimetres. Frame: the G1 `head_link` frame (+x forward,
//  +y left, +z up), same origin as Unitree's meshes/head_link.STL,
//  so the stock collar below CUT_Z drops onto the torso unchanged.
//
//  PART selects what is rendered:
//    "assembly"  frame + all panels (preview)
//    "exploded"  panels pushed outwards (preview)
//    "frame"     printable frame (collar, neck, hub, spokes, ring,
//                ribs, crown and camera face)
//    "panel"     one side panel, chosen by PANEL (0..N_PANELS-1)
//    "brow"      one eyebrow slider, BROW_SIDE = 1 (left) or -1 (right)
//
//  Panels are screwed on from the inside: each panel has 4 bosses
//  for M3 heat-set inserts. M3 x 10 screws go through the rib
//  flanges from inside the head (reach in through the open
//  underside) into the inserts.
//
//  Tested geometry: the same model was generated in Python and checked
//  for panel/frame interference, wall thickness and camera clearance.
//  Needs OpenSCAD 2024+ with the Manifold backend for sensible render
//  times (Preferences > Features/Advanced > Manifold).
// =====================================================================

PART    = "assembly";   // "assembly" | "exploded" | "frame" | "panel" | "brow"
PANEL   = 0;            // which panel when PART = "panel"
BROW_SIDE = 1;          // which brow when PART = "brow" (1 left, -1 right)
BROW_POP  = [0, 0];     // preview: [left, right] travel in mm (0 = flush, BROW_TRAVEL = fully out)
SHOW_SERVOS = true;     // ghost SG90s in assembly previews
EXPLODE = 40;           // mm, for "exploded"

STOCK_STL = "stock/head_link.STL";   // Unitree's head_link.STL (metres)

$fn = 160;              // drop to ~64 while editing

// ---------------- dome ----------------
C       = [-5, 0];      // dome centre in x,y (over the neck)
Z0      = 400;          // equator (brim) height
R       = 125;          // dome radius -> 250 mm diameter
H       = 85;           // dome height above the equator
BELLY_H = 14;           // rounded underside of the brim
WALL    = 3;            // shell / panel thickness
BAND_R  = 1.8;          // decorative bead round the brim

// ---------------- frame layout ----------------
CUT_Z     = 372;        // keep stock collar below this height
Z_CROWN   = 452;        // crown (fixed top cap) starts here
RING_W    = 14;         // brim ring radial width
FACE_A    = 40;         // camera face spans -FACE_A..+FACE_A degrees
N_PANELS  = 5;          // removable panels round the back and sides
RIB_W     = 10;         // visible rib between panels (tangential)
FLANGE_W  = 38;         // rib flange behind panel edges
FL_OUT    = 10;         // flange outer surface: ellipsoid inset by this
FL_IN     = 13;         // flange inner surface
GAP       = 0.4;        // panel clearance all round
FIT       = 0.2;        // boss-to-flange clearance

// panel z range (sits on the brim ring, tucks under the crown)
PZ0 = Z0 + 3 + GAP;
PZ1 = Z_CROWN - GAP;

// ---------------- panel fixings (M3) ----------------
BOSS_D    = 9;
BOSS_V    = RIB_W/2 + GAP + 7.1;     // boss centre from rib centreline
BOSS_Z    = [418, 438];
INSERT_D  = 4.0;                     // M3 heat-set insert hole
INSERT_L  = 6;
SCREW_CLR = 3.4;

// ---------------- D455 (124 x 29 x 26 mm) ----------------
CAM_FRONT = 95;                      // flat facet + camera glass plane
CAM_Z     = 402;
CAM_HALF  = [62.4, 14.8];            // pocket half size y,z (124.8 x 29.6)
CAM_RND   = 10;
CAM_DEPTH = 26.5;
HOUSING_X0 = 63;                     // back of the camera housing
BEZ_HALF  = [66, 23.25]; BEZ_CZ = 397.25; BEZ_RND = 13;
TROUGH_Z  = [377.5, 388];            // cable trough under the camera
TROUGH_Y  = [8, 60];                 // (D455 USB-C is on its underside)
MANIFOLD_X = [66, CAM_FRONT - 22];
TRIPOD_X  = CAM_FRONT - 13;          // 1/4-20 tripod thread, camera centre
CH_A = [69, 0, 382.5];  CH_B = [-5, 0, 384];   // cable channel to neck bore
CH_R = 6.5; SLEEVE_R = 10;
BORE_R = 9;

// ---------------- neck / hub ----------------
HUB_R  = 40;
HUB_Z  = [386, Z0];
SPOKE_W = 10;

// ---------------- eyebrows: sliding bars driven by SG90 micro servos -------------
// Each brow is a bar that sits flush in a slot in the face above the camera and
// slides straight out (+x) up to BROW_TRAVEL mm. A tab on its back end is linked
// to the servo horn with a ~1 mm wire pushrod (Z-bends). The two servos sit back to
// back on the centreline, shafts pointing outboard, horns in the x-z plane.
BROW_Y      = [26, 54];          // |y| span of each brow
BROW_Z      = [422, 428];        // z span (6 mm tall)
BROW_RND    = 3;                 // end rounding (stadium ends)
BROW_LEN    = 20;                // bar length front-to-back
BROW_TRAVEL = 11;                // hard stop: tab hits the end of the guide opening
BROW_CLR    = 0.3;               // slide clearance
GUIDE_X     = [80, CAM_FRONT - 3];   // guide sleeve behind the face plate
GUIDE_WALL  = 2;
TAB_Y       = [29, 32];          // drive tab on top of the brow's back end
TAB_X       = 4;                 // tab length along x
TAB_TOP     = 433;
TAB_HOLE_D  = 1.3;               // for 1 mm wire
// SG90 micro servo (TowerPro-style): body 22.8 x 12.2 x 22.7, tabs 32.3 long, 2.5 thick
SV_L = 22.8; SV_W = 12.2; SV_H = 22.7; SV_SHAFT_OFF = 5.9;
SV_TAB_L = 32.3; SV_TAB_T = 2.5; SV_TAB_Z = 15.9; SV_HOLE_SP = 27.8;
SV_SHAFT = [78, 440];            // shaft axis (x, z); axis runs along y
SV_Y0    = 1.5;                  // servo bottom face distance from centreline
SV_PLATE = 3;                    // mounting plate thickness (tabs screw to its outboard face)
SV_CLR   = 0.3;
SV_PILOT = 1.6;                  // pilot hole for the servo's own M2 self-tappers
HORN_R   = 9;                    // trim the single-arm horn to this radius

// ---------------- neck fixing to the framed torso (open3dhumanoid torso, top panel) ----------------
// 4 x M3 heat-set inserts in the underside of the stock collar. M3 x 16 screws come up from inside
// the torso top panel. Positions are in this head_link frame (torso x - 3.9635, torso y).
NECK_INSERTS = [[-21.96, 22, 332.8], [-21.96, -22, 332.8], [18.04, 22, 331.7], [18.04, -22, 331.7]];  // x, y, underside z
NECK_INSERT_D = 4.0;     // M3 x 5.7 heat-set insert
NECK_INSERT_L = 7;

RIB_ANGLES = [for (i = [0:N_PANELS]) FACE_A + i*(360 - 2*FACE_A)/N_PANELS];

// =====================================================================
//  helpers
// =====================================================================
function rk(k, z) = (R - k) * sqrt(max(0, 1 - pow((z - Z0)/(H - k), 2)));

module at_c() translate([C[0], C[1], 0]) children();

module ell(k = 0) at_c() translate([0, 0, Z0]) scale([R - k, R - k, H - k]) sphere(1);

module zslab(z0, z1) translate([-400, -400, z0]) cube([800, 800, z1 - z0]);

// full outer solid: upper dome + shallow belly
module outer_solid() {
    intersection() { ell(0); zslab(Z0, Z0 + H + 1); }
    intersection() {
        at_c() translate([0, 0, Z0]) scale([R, R, BELLY_H]) sphere(1);
        zslab(Z0 - BELLY_H - 1, Z0);
    }
}

module shell(k_out = 0, k_in = WALL) difference() { ell(k_out); ell(k_in); }

module wedge(a1, a2, z0, z1, n = 48)
    at_c() translate([0, 0, z0]) linear_extrude(z1 - z0)
        polygon(concat([[0, 0]], [for (i = [0:n]) let(a = a1 + (a2 - a1)*i/n) 400*[cos(a), sin(a)]]));

module rib_slab(a, w, z0, z1, r0 = 0, r1 = 400)
    at_c() rotate([0, 0, a]) translate([r0, -w/2, z0]) cube([r1 - r0, w, z1 - z0]);

// cylinder along the local radial (u) axis of a rib frame
module radial_cyl(a, v, z, u0, u1, d)
    at_c() rotate([0, 0, a]) translate([u0, v, z]) rotate([0, 90, 0]) cylinder(d = d, h = u1 - u0, $fn = 32);

// rounded rectangle in the y-z plane, extruded along x
module rrect_x(yh, zh, r, cz, x0, x1)
    translate([x0, 0, cz]) rotate([0, 90, 0]) linear_extrude(x1 - x0)
        offset(r = r) square([2*(zh - r), 2*(yh - r)], center = true);

module capsule(a, b, r) hull() { translate(a) sphere(r, $fn = 32); translate(b) sphere(r, $fn = 32); }

// =====================================================================
//  frame
// =====================================================================
module stock_collar()
    intersection() {
        scale(1000) import(STOCK_STL, convexity = 10);
        zslab(250, CUT_Z);
    }

module neck() {
    translate([-4.7, 0, CUT_Z - 8]) scale([27, 29.5, 1]) cylinder(r = 1, h = Z0 - 2 - (CUT_Z - 8));
    at_c() translate([0, 0, 376]) cylinder(r1 = 29, r2 = HUB_R, h = HUB_Z[0] - 376);   // flare
    at_c() translate([0, 0, HUB_Z[0]]) cylinder(r = HUB_R, h = HUB_Z[1] - HUB_Z[0]);    // hub
}

module spokes()
    for (a = RIB_ANGLES) intersection() {
        rib_slab(a, SPOKE_W, 388, Z0, HUB_R - 5, R - RING_W + 4);
        outer_solid();
    }

module brim_ring() {
    intersection() {
        outer_solid();
        at_c() difference() {
            translate([0, 0, Z0 - BELLY_H - 1]) cylinder(r = R + 5, h = BELLY_H + 4);
            translate([0, 0, Z0 - BELLY_H - 2]) cylinder(r = R - RING_W, h = BELLY_H + 6);
        }
        zslab(Z0 - BELLY_H - 1, Z0 + 3);
    }
    at_c() translate([0, 0, Z0]) rotate_extrude() translate([R, 0]) circle(BAND_R, $fn = 24);
}

module ribs()
    for (a = RIB_ANGLES) {
        intersection() { rib_slab(a, RIB_W, Z0, Z_CROWN + 3); difference() { ell(0); ell(FL_IN); } }
        difference() {   // flange behind the panel edges (none needed on the camera-face side)
            intersection() { rib_slab(a, FLANGE_W, PZ0, PZ1); difference() { ell(FL_OUT); ell(FL_IN); } }
            wedge(-FACE_A, FACE_A, PZ0 - 1, PZ1 + 1);
        }
    }

module crown() {
    intersection() { shell(0, WALL); zslab(Z_CROWN, Z0 + H + 1); }
    intersection() { difference() { ell(WALL); ell(12); } zslab(Z_CROWN, Z_CROWN + 6); }   // stiffening ring
}

module face_shell() {
    intersection() {
        union() {
            intersection() { shell(0, WALL); wedge(-FACE_A, FACE_A, Z0, Z_CROWN + 1); }
            intersection() { outer_solid(); translate([CAM_FRONT - WALL, -200, Z0]) cube([WALL, 400, H]); }  // facet plate
        }
        translate([-400, -400, 0]) cube([400 + CAM_FRONT, 800, 600]);     // flat facet
    }
}

module camera_housing() {
    rrect_x(BEZ_HALF[0], BEZ_HALF[1], BEZ_RND, BEZ_CZ, HOUSING_X0, CAM_FRONT);
    capsule(CH_A, CH_B, SLEEVE_R);
}

module camera_cuts() {
    rrect_x(CAM_HALF[0], CAM_HALF[1], CAM_RND, CAM_Z, CAM_FRONT - CAM_DEPTH, CAM_FRONT + 30);       // pocket
    for (s = [1, -1]) translate([CAM_FRONT - 23, s > 0 ? TROUGH_Y[0] : -TROUGH_Y[1], TROUGH_Z[0]])
        cube([50, TROUGH_Y[1] - TROUGH_Y[0], TROUGH_Z[1] - TROUGH_Z[0]]);                               // USB troughs
    translate([MANIFOLD_X[0], -TROUGH_Y[1], TROUGH_Z[0]])
        cube([MANIFOLD_X[1] - MANIFOLD_X[0] + 1, 2*TROUGH_Y[1], TROUGH_Z[1] - TROUGH_Z[0]]);         // manifold
    translate([TRIPOD_X, 0, 360]) cylinder(r = 3.3, h = 35, $fn = 32);                               // 1/4-20 clearance
    translate([TRIPOD_X, 0, 360]) cylinder(r = 6.2, h = 382 - 360, $fn = 48);                        // counterbore
    capsule(CH_A, CH_B, CH_R);                                                                       // cable channel
    at_c() translate([0, 0, 300]) cylinder(r = BORE_R, h = Z0 + 5 - 300, $fn = 48);                 // neck bore
}

// ---------------- eyebrow guides and servo mounts (frame) ----------------
module stadium_x(y0, y1, z0, z1, r, x0, x1)          // rounded-end bar profile in y-z, along x
    hull() for (y = [y0 + r, y1 - r]) translate([x0, y, (z0 + z1)/2])
        rotate([0, 90, 0]) scale([(z1 - z0)/(2*r), 1, 1]) cylinder(r = r, h = x1 - x0, $fn = 32);

function sv_body_x() = [SV_SHAFT[0] - (SV_L - SV_SHAFT_OFF), SV_SHAFT[0] + SV_SHAFT_OFF];
function sv_cx()     = SV_SHAFT[0] - (SV_L/2 - SV_SHAFT_OFF);

module brow_guides()
    for (m = [1, -1]) scale([1, m, 1]) {
        // sleeve round the slot, behind the face plate
        stadium_x(BROW_Y[0] - BROW_CLR - GUIDE_WALL, BROW_Y[1] + BROW_CLR + GUIDE_WALL,
                  BROW_Z[0] - BROW_CLR - GUIDE_WALL, BROW_Z[1] + BROW_CLR + GUIDE_WALL,
                  BROW_RND + GUIDE_WALL, GUIDE_X[0], GUIDE_X[1] + 1);
        // servo mounting plate (tabs screw to its outboard face)
        intersection() {
            translate([sv_body_x()[0] - 8, SV_Y0 + SV_TAB_Z - SV_PLATE, BROW_Z[1]])
                cube([CAM_FRONT - (sv_body_x()[0] - 8), SV_PLATE, 40]);
            ell(1);
        }
    }

module brow_cuts()
    for (m = [1, -1]) scale([1, m, 1]) {
        // slide channel through face plate and sleeve
        stadium_x(BROW_Y[0] - BROW_CLR, BROW_Y[1] + BROW_CLR, BROW_Z[0] - BROW_CLR, BROW_Z[1] + BROW_CLR,
                  BROW_RND, GUIDE_X[0] - 10, CAM_FRONT + 5);
        // opening along the top of the sleeve for the drive tab; its front end is the travel stop
        translate([GUIDE_X[0] - 1, TAB_Y[0] - BROW_CLR, BROW_Z[1]])
            cube([(CAM_FRONT - BROW_LEN + TAB_X + BROW_TRAVEL) - (GUIDE_X[0] - 1), TAB_Y[1] - TAB_Y[0] + 2*BROW_CLR, 10]);
        // servo body cut-out and pilot holes in the mounting plate
        translate([sv_body_x()[0] - SV_CLR, SV_Y0 + SV_TAB_Z - SV_PLATE - 1, SV_SHAFT[1] - SV_W/2 - SV_CLR])
            cube([SV_L + 2*SV_CLR, SV_PLATE + 2, SV_W + 2*SV_CLR]);
        for (dx = [-SV_HOLE_SP/2, SV_HOLE_SP/2])
            translate([sv_cx() + dx, SV_Y0 + SV_TAB_Z - SV_PLATE - 1, SV_SHAFT[1]]) rotate([-90, 0, 0])
                cylinder(d = SV_PILOT, h = SV_PLATE + 2, $fn = 16);
    }

// ---------------- eyebrow slider (separate print) ----------------
// built for the left side (+y); travel moves it +x
module brow(side = 1, travel = 0)
    scale([1, side > 0 ? 1 : -1, 1]) translate([travel, 0, 0]) difference() {
        union() {
            stadium_x(BROW_Y[0], BROW_Y[1], BROW_Z[0], BROW_Z[1], BROW_RND, CAM_FRONT - BROW_LEN, CAM_FRONT);
            translate([CAM_FRONT - BROW_LEN, TAB_Y[0], BROW_Z[1] - 1]) cube([TAB_X, TAB_Y[1] - TAB_Y[0], TAB_TOP - BROW_Z[1] + 1]);
        }
        translate([CAM_FRONT - BROW_LEN + TAB_X/2, TAB_Y[0] - 1, TAB_TOP - 2]) rotate([-90, 0, 0])
            cylinder(d = TAB_HOLE_D, h = TAB_Y[1] - TAB_Y[0] + 2, $fn = 16);
    }

// SG90 ghost for previews
module sg90(side = 1)
    scale([1, side > 0 ? 1 : -1, 1]) {
        translate([sv_body_x()[0], SV_Y0, SV_SHAFT[1] - SV_W/2]) cube([SV_L, SV_H, SV_W]);
        translate([sv_cx() - SV_TAB_L/2, SV_Y0 + SV_TAB_Z, SV_SHAFT[1] - SV_W/2]) cube([SV_TAB_L, SV_TAB_T, SV_W]);
        translate([SV_SHAFT[0], SV_Y0 + SV_H, SV_SHAFT[1]]) rotate([-90, 0, 0]) cylinder(r = 2.4, h = 4, $fn = 24);
        translate([SV_SHAFT[0], SV_Y0 + SV_H + 2.5, SV_SHAFT[1]]) rotate([-90, 0, 0]) cylinder(r = HORN_R, h = 1.5, $fn = 32);
    }

module neck_inserts()
    for (p = NECK_INSERTS) translate([p[0], p[1], p[2] - 2]) cylinder(d = NECK_INSERT_D, h = NECK_INSERT_L + 2, $fn = 24);

module flange_holes()
    for (i = [0:N_PANELS - 1], side = [0, 1], z = BOSS_Z) {
        a = RIB_ANGLES[i + side];
        v = side == 0 ? BOSS_V : -BOSS_V;
        radial_cyl(a, v, z, sqrt(pow(rk(FL_IN, z), 2) - v*v) - 2, sqrt(pow(rk(FL_OUT, z), 2) - v*v) + 1, SCREW_CLR);
    }

module frame()
    difference() {
        intersection() {
          union() {
            stock_collar();
            neck();
            spokes();
            brim_ring();
            ribs();
            crown();
            face_shell();
            camera_housing();
            brow_guides();
          }
          translate([-400, -400, 0]) cube([400 + CAM_FRONT, 800, 600]);   // nothing ahead of the camera face
        }
        camera_cuts();
        brow_cuts();
        flange_holes();
        neck_inserts();
    }

// =====================================================================
//  panels
// =====================================================================
module panel(i) {
    a1 = RIB_ANGLES[i]; a2 = RIB_ANGLES[i + 1];
    difference() {
        union() {
            intersection() { shell(0, WALL); wedge(a1, a2, PZ0, PZ1); }
            for (side = [0, 1], z = BOSS_Z) {
                a = side == 0 ? a1 : a2;
                v = side == 0 ? BOSS_V : -BOSS_V;
                // boss end follows the curved flange face, FIT clear of it
                difference() {
                    radial_cyl(a, v, z, sqrt(pow(rk(FL_OUT, z), 2) - v*v) - 6,
                                        sqrt(pow(rk(WALL, z), 2) - v*v) + 1, BOSS_D);
                    ell(FL_OUT - FIT);
                }
            }
        }
        rib_slab(a1, RIB_W + 2*GAP, PZ0 - 5, PZ1 + 5);
        rib_slab(a2, RIB_W + 2*GAP, PZ0 - 5, PZ1 + 5);
        for (side = [0, 1], z = BOSS_Z) {
            a = side == 0 ? a1 : a2;
            v = side == 0 ? BOSS_V : -BOSS_V;
            u0 = sqrt(pow(rk(FL_OUT, z), 2) - v*v);
            radial_cyl(a, v, z, u0 - 6, u0 + INSERT_L, INSERT_D);
        }
    }
}

function panel_mid(i) = (RIB_ANGLES[i] + RIB_ANGLES[i + 1]) / 2;

// =====================================================================
//  output
// =====================================================================
module brows_and_servos() {
    color("Gold") { brow(1, BROW_POP[0]); brow(-1, BROW_POP[1]); }
    if (SHOW_SERVOS) color("RoyalBlue", 0.6) { sg90(1); sg90(-1); }
}

if (PART == "frame") frame();
else if (PART == "panel") panel(PANEL);
else if (PART == "brow") brow(BROW_SIDE, 0);
else if (PART == "exploded") {
    color("DimGray") frame();
    for (i = [0:N_PANELS - 1]) let(m = panel_mid(i))
        color("SteelBlue") translate(EXPLODE*[cos(m), sin(m), 0]) panel(i);
    brows_and_servos();
}
else {
    color("DimGray") frame();
    for (i = [0:N_PANELS - 1]) color("SteelBlue") panel(i);
    brows_and_servos();
}
