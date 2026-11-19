// G1 left leg with integrated 6512 cycloidal actuators
//
// Hip pitch, hip roll, hip yaw and knee each get the actuator built into the links:
//   parent link (stator side): motor bay, stator floor and gear ring counterbore with
//     12 heat-set insert holes, plus at least WALL_MIN of wall around the pocket
//   child link (output side): cut back LINK_GAP from the joint face, boss that bolts to
//     Output_Shaft_Front with 6x M3 on a 36 mm PCD, plus hex-key access
//
// Files this needs (paths are relative to this file, in the repo's STL/legs/ folder):
//   STL/legs/stock/*.stl                     cleaned G1 links, one solid each, in mm (pelvis,
//                                            left_hip_pitch_link, left_hip_roll_link, left_hip_yaw_link,
//                                            left_knee_link): copy your stock/ folder there
//   STL/legs/actuator/Motor_Shell_-_Motor_Shell.stl   the stator floor is copied from this
//   STL/legs/actuator/Housing_*.stl, Output_*.stl     only for show_actuators in the assembly view
//
// side = "right" exports the right-leg parts as the mirror image of the left ones (the G1 right leg
// is the left leg mirrored in y). The pelvis halves already come as a pair.
//
// Any OpenSCAD version works. The 2021.01 release rendered a pelvis half in about 85 s and
// the hip roll link in about 25 s on a modest machine. A development snapshot with the
// Manifold backend switched on (Preferences > Advanced > 3D Rendering > Backend) is much
// faster, which matters most for a full render (F6) of the whole assembly.
//
// The numbers in "Worked out by integrate_leg.py" came from the Python build at 1:1.
// Everything else can be changed here. To change SCALE, rerun integrate_leg.py at the new
// scale and copy its params.json values into that block.

part = "assembly"; // [assembly, pelvis_left_half, pelvis_right_half, hip_pitch_link, hip_roll_link, thigh, shin, waist_yaw_link, waist_stator_carrier]
side = "left";     // [left, right] for hip_pitch_link, hip_roll_link, thigh and shin
show_actuators = true;   // assembly view only: place the actuator housings and outputs
both_legs = true;        // assembly view: add the right leg (mirror of the left)
STOCK_DIR    = "../../STL/legs/stock/";
ACTUATOR_DIR = "../../STL/legs/actuator/";

// ---------------- design parameters (mm) ----------------
SCALE        = 1.0;      // G1 scale. The baked block below is for 1.0
RING_CLEAR   = 0.20;     // radial clearance, gear rings in the counterbore
BAY_R        = 29.25;    // motor bay radius (original cage posts at r = 29.0)
WALL_MIN     = 3.0;      // minimum wall around every actuator pocket
INSERT_D     = 4.0;      // M3 heat-set insert hole
INSERT_DEPTH = 8.0;
CAVITY_R     = 30.0;     // access opening behind each stator floor
HATCH_HALF_H = 28.0;     // pelvis wiring slot half height
LINK_GAP     = 2.0;      // gap between a child link and the stationary cap
BOSS_R       = 22.4;     // child boss radius (the output flange is 22.5)
M3_CLEAR     = 3.4;
CBORE_D      = 6.5;      // counterbore and hex-key bore
BOSS_T       = 4.0;      // boss material under the bolt heads
WINDOW_H     = 20.0;     // thigh access window chamber height
WINDOW_R     = 18 + CBORE_D / 2 + 1.5;
JOIN_BOLTS   = [[38, -80], [-38, -62], [40, -40], [-40, -40]];   // pelvis halves, (x, z) at 1:1
JOIN_DOWELS  = [[41, -52], [-41, -52]];   // the old [+-20,-38] bolts and [+-30,-50] dowels sat in the waist pocket
BOLT_GRIP    = 6.0;      // right-half material under each joining bolt head (M3x12 into an insert)
DOWEL_HOLE   = 4.1;      // for 4 mm dowel pins
DOWEL_DEPTH  = 6.0;      // each side
Z_RINGS      = -29;      // actuator frame: z out of the face, z = 0 on the output face
Z_FLOOR      = -61;

FN_BIG = 128; FN_MID = 48; FN_SMALL = 32;   // circle resolution

// ---------------- worked out by integrate_leg.py (params.json, scale 1.0) ----------------
D_ROLL         = 33.5014;   // roll output face, forward of the roll joint
D_YAW          = 0.0041;    // yaw output face, below the yaw joint
D_KNEE         = 15.3733;   // knee output face, outboard of the knee joint
ROLL_CLOCK     = 0;         // roll actuator rotation (keeps the pitch bolt bores off its inserts)
PITCH_CLOCK    = 10;        // bolt pattern rotations
ROLL_OUT_CLOCK = 0;
YAW_OUT_CLOCK  = 0;
KNEE_OUT_CLOCK = 0;
PITCH_ZBOSS    = 16.25;     // boss heights
ROLL_ZBOSS     = 8;
YAW_ZBOSS      = 8;
KNEE_ZBOSS     = 8;
R_CLEAR_ROLL   = 48 * SCALE;   // how far out each child link is cut back at the joint face
R_CLEAR_YAW    = 56 * SCALE;
R_CLEAR_KNEE   = 41 * SCALE;

// ---------------- waist yaw (same 6512 actuator, stator in the pelvis top) ----------------
WAIST_FACE      = 2.2;     // output face = pelvis top face, output facing +z
WAIST_CAVITY_R  = 21;      // wire space under the stator carrier
WAIST_CAVITY_D  = 5;
WAIST_CHANNEL_W = 12;      // wire channel from that space out to the front of the pelvis
CARRIER_R       = 29.0;    // drop-in stator carrier (motor-shell floor + outer ring)
CARRIER_RING_IN = 25.6;
CARRIER_SCREW_R = 25.5;    // 4 x M3 countersunk on the floor spokes (45, 135, 225, 315 deg)
M3_CSK          = 6.6;
WAIST_DISC_TOP  = 15.4;    // top of the G1 waist yaw disc (roll/pitch lugs above it are dropped)
WAIST_RISER_R   = 35;      // riser from the disc up to the torso bottom plate
WAIST_RISER_HOLE_R = 12;
WAIST_PLATE_Z   = 73.0;    // torso bottom plate underside, pelvis frame, zero waist angles
WAIST_PLATE_PCD_R  = 25;   // 6 x M4 heat-set inserts for the torso plate, at 30 + 60k deg
WAIST_PLATE_INSERT_D = 5.6;
WAIST_PLATE_INSERT_L = 8;

// ---------------- frames ----------------
function T(v) = [[1, 0, 0, v[0]], [0, 1, 0, v[1]], [0, 0, 1, v[2]], [0, 0, 0, 1]];
function Ry(a) = [[cos(a), 0, sin(a), 0], [0, 1, 0, 0], [-sin(a), 0, cos(a), 0], [0, 0, 0, 1]];
function Rz(a) = [[cos(a), -sin(a), 0, 0], [sin(a), cos(a), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
function S(v) = v * 1000 * SCALE;      // URDF metres -> mm
RAD = 180 / PI;

// G1 23dof URDF joint frames, left leg
J_PITCH = T(S([0, 0.064452, -0.1027]));
J_ROLL  = T(S([0, 0.052, -0.030465])) * Ry(-0.1749 * RAD);
J_YAW   = T(S([0.025001, 0, -0.12412]));
J_KNEE  = T(S([-0.078273, 0.0021489, -0.17734])) * Ry(0.1749 * RAD);

// actuator z onto a joint axis
AZ_TO_X  = [[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]];    // output faces +x (forward)
AZ_TO_Y  = [[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]];   // output faces +y (outboard)
AZ_TO_NZ = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]];  // output faces -z (down)

// actuator frames in their parent link
PEL_L    = [[1, 0, 0, 0], [0, 0, 1, J_PITCH[1][3]], [0, -1, 0, J_PITCH[2][3]], [0, 0, 0, 1]];
PEL_R    = [[-1, 0, 0, 0], [0, 0, -1, -J_PITCH[1][3]], [0, -1, 0, J_PITCH[2][3]], [0, 0, 0, 1]];
ROLL_ACT = J_ROLL * T([D_ROLL, 0, 0]) * AZ_TO_X * Rz(ROLL_CLOCK);
YAW_ACT  = J_YAW * T([0, 0, -D_YAW]) * AZ_TO_NZ;
KNEE_ACT = J_KNEE * T([0, D_KNEE, 0]) * AZ_TO_Y;
WAIST_ACT = T([0, 0, WAIST_FACE]);

// ---------------- primitives (actuator frame) ----------------
module zcyl(r, z0, z1, fn = FN_BIG) { translate([0, 0, z0]) cylinder(r = r, h = z1 - z0, $fn = fn); }
module ycyl(x, z, r, y0, y1, fn = FN_MID) { translate([x, y0, z]) rotate([-90, 0, 0]) cylinder(r = r, h = y1 - y0, $fn = fn); }
module box(lo, hi) { translate(lo) cube(hi - lo); }

module sleeve() {                       // guarantees WALL_MIN around the pocket
    zcyl(35 + RING_CLEAR + WALL_MIN, Z_RINGS, 0);
    zcyl(BAY_R + WALL_MIN, Z_FLOOR, Z_RINGS);
}

module stator_cut(cavity_len = 600) {   // ring counterbore, motor bay, insert holes, access opening
    zcyl(35 + RING_CLEAR, Z_RINGS, 1);
    zcyl(BAY_R, Z_FLOOR - 0.01, Z_RINGS + 0.01);
    for (k = [0 : 11]) rotate([0, 0, 30 * k]) translate([32, 0, 0])
        zcyl(INSERT_D / 2, Z_RINGS - INSERT_DEPTH, Z_RINGS + 0.01, FN_SMALL);
    if (cavity_len > 0) zcyl(CAVITY_R, Z_FLOOR - cavity_len, Z_FLOOR + 0.01);
}

module stator_floor() {                 // motor shell floor: stator bolt pattern, centre bore, wire slots
    translate([0, 0, Z_RINGS]) intersection() {
        import(str(ACTUATOR_DIR, "Motor_Shell_-_Motor_Shell.stl"));
        zcyl(BAY_R + 0.05, -32.01, -25.99);
    }
}

module bolt_cutter(clock, zboss, window = false) {
    for (k = [0 : 5]) rotate([0, 0, clock + 60 * k]) translate([18, 0, 0]) {
        zcyl(M3_CLEAR / 2, -1, BOSS_T + 0.01, FN_SMALL);
        zcyl(CBORE_D / 2, BOSS_T, window ? zboss + 0.5 : 800, FN_MID);
    }
    for (k = [0 : 5]) rotate([0, 0, clock + 30 + 60 * k]) translate([17, 0, 0])
        zcyl(3.5, -1, 1, FN_SMALL);    // relief over the pin bolt heads
    if (window) {                       // chamber under the boss, open towards +x
        zcyl(WINDOW_R, zboss, zboss + WINDOW_H);
        box([0, -WINDOW_R, zboss], [400, WINDOW_R, zboss + WINDOW_H]);
    }
}

// ---------------- link operations (act on children) ----------------
module with_stator(M, cavity_len = 600) {
    union() {
        difference() {
            union() { children(); multmatrix(M) sleeve(); }
            multmatrix(M) stator_cut(cavity_len);
        }
        multmatrix(M) stator_floor();
    }
}

module with_output(M, r_clear, zboss, clock, window = false, slab = false) {
    difference() {
        union() {
            difference() {
                children();
                multmatrix(M) if (slab) box([-500, -500, -600], [500, 500, LINK_GAP]);
                              else zcyl(r_clear, -600, LINK_GAP);
            }
            multmatrix(M) zcyl(BOSS_R, 0, zboss);
        }
        multmatrix(M) bolt_cutter(clock, zboss, window);
    }
}

module stock(name) { scale(SCALE) import(str(STOCK_DIR, name, ".stl")); }

module waist_cut() {                    // actuator frame: rings, bay, inserts, carrier seat, wire channel
    stator_cut(0);
    zcyl(WAIST_CAVITY_R, Z_FLOOR - WAIST_CAVITY_D, Z_FLOOR + 0.01);
    for (a = [45 : 90 : 315]) rotate([0, 0, a]) translate([CARRIER_SCREW_R, 0, 0])
        zcyl(INSERT_D / 2, Z_FLOOR - 6, Z_FLOOR + 0.01, FN_SMALL);
    box([0, -WAIST_CHANNEL_W / 2, Z_FLOOR - WAIST_CAVITY_D], [120, WAIST_CHANNEL_W / 2, Z_FLOOR]);
}
module waist_sleeve() {
    zcyl(35 + RING_CLEAR + WALL_MIN, Z_RINGS, 0);
    zcyl(BAY_R + WALL_MIN, Z_FLOOR - WAIST_CAVITY_D - WALL_MIN, Z_RINGS);
}

module waist_stator_carrier() {         // pelvis frame. Screw the stator to it on the bench, drop it in
    multmatrix(WAIST_ACT) translate([0, 0, Z_RINGS]) difference() {
        union() {
            intersection() { import(str(ACTUATOR_DIR, "Motor_Shell_-_Motor_Shell.stl")); zcyl(CARRIER_R, -32.01, -25.99); }
            difference() { zcyl(CARRIER_R, -32, -26); zcyl(CARRIER_RING_IN, -33, -25); }
        }
        for (a = [45 : 90 : 315]) rotate([0, 0, a]) translate([CARRIER_SCREW_R, 0, 0]) {
            zcyl(M3_CLEAR / 2, -33, -25, FN_SMALL);
            translate([0, 0, -26 - (M3_CSK - M3_CLEAR) / 2])
                cylinder(r1 = M3_CLEAR / 2, r2 = M3_CSK / 2, h = (M3_CSK - M3_CLEAR) / 2 + 0.01, $fn = FN_SMALL);
            zcyl(M3_CSK / 2, -26, -25, FN_SMALL);
        }
    }
}

module waist_yaw_link() {               // pelvis frame at zero yaw: bolts to the waist output, carries the torso
    difference() {
        union() {
            difference() {
                union() {
                    intersection() { stock("waist_yaw_link_rev_1_0"); zcyl(50, -10, WAIST_DISC_TOP); }
                    zcyl(WAIST_RISER_R, WAIST_DISC_TOP - 1, WAIST_PLATE_Z);
                }
                zcyl(200, -50, WAIST_FACE + LINK_GAP);
                zcyl(WAIST_RISER_HOLE_R, WAIST_DISC_TOP + 3, WAIST_PLATE_Z + 1);
            }
            multmatrix(WAIST_ACT) zcyl(BOSS_R, 0, LINK_GAP + 1);
        }
        multmatrix(WAIST_ACT) bolt_cutter(0, 0);        // 6 x M3 to the output, hex-key bores up through the riser
        for (k = [0 : 5]) rotate([0, 0, 30 + 60 * k]) translate([WAIST_PLATE_PCD_R, 0, 0])
            zcyl(WAIST_PLATE_INSERT_D / 2, WAIST_PLATE_Z - WAIST_PLATE_INSERT_L, WAIST_PLATE_Z + 1, FN_SMALL);
    }
}

// ---------------- parts ----------------
Y_FLOOR = J_PITCH[1][3] + Z_FLOOR;     // back face of each hip pitch stator floor
ZC      = J_PITCH[2][3];

module pelvis_body() {
    union() {
        difference() {
            union() { stock("pelvis"); multmatrix(PEL_L) sleeve(); multmatrix(PEL_R) sleeve(); multmatrix(WAIST_ACT) waist_sleeve(); }
            multmatrix(PEL_L) stator_cut(0);
            multmatrix(PEL_R) stator_cut(0);
            multmatrix(WAIST_ACT) waist_cut();
            ycyl(0, ZC, CAVITY_R, -Y_FLOOR, Y_FLOOR);                                        // recess behind both floors
            box([-500, -Y_FLOOR, ZC - HATCH_HALF_H], [0, Y_FLOOR, ZC + HATCH_HALF_H]);       // wiring slot out the back
        }
        multmatrix(PEL_L) stator_floor();
        multmatrix(PEL_R) stator_floor();
    }
}

module pelvis_left_half() {
    difference() {
        intersection() { pelvis_body(); box([-500, 0, -500], [500, 500, 500]); }
        for (b = JOIN_BOLTS)  ycyl(b[0] * SCALE, b[1] * SCALE, INSERT_D / 2, -0.01, INSERT_DEPTH);
        for (d = JOIN_DOWELS) ycyl(d[0] * SCALE, d[1] * SCALE, DOWEL_HOLE / 2, -DOWEL_DEPTH, DOWEL_DEPTH);
    }
}

module pelvis_right_half() {
    difference() {
        intersection() { pelvis_body(); box([-500, -500, -500], [500, 0, 500]); }
        for (b = JOIN_BOLTS) {
            ycyl(b[0] * SCALE, b[1] * SCALE, M3_CLEAR / 2, -BOLT_GRIP, 0.01);
            ycyl(b[0] * SCALE, b[1] * SCALE, CBORE_D / 2, -600, -BOLT_GRIP);
        }
        for (d = JOIN_DOWELS) ycyl(d[0] * SCALE, d[1] * SCALE, DOWEL_HOLE / 2, -DOWEL_DEPTH, DOWEL_DEPTH);
    }
}

module hip_pitch_link() {     // bolts to the hip pitch output, houses the hip roll actuator
    with_output(AZ_TO_Y, 0, PITCH_ZBOSS, PITCH_CLOCK, slab = true)
        with_stator(ROLL_ACT) stock("left_hip_pitch_link");
}

module hip_roll_link() {      // bolts to the hip roll output, houses the hip yaw actuator
    with_output(T([D_ROLL, 0, 0]) * AZ_TO_X, R_CLEAR_ROLL, ROLL_ZBOSS, ROLL_OUT_CLOCK)
        with_stator(YAW_ACT, cavity_len = 2) stock("left_hip_roll_link");
}

module thigh() {              // bolts to the hip yaw output through a front window, houses the knee actuator
    with_output(T([0, 0, -D_YAW]) * AZ_TO_NZ, R_CLEAR_YAW, YAW_ZBOSS, YAW_OUT_CLOCK, window = true)
        with_stator(KNEE_ACT) stock("left_hip_yaw_link");
}

module shin() {               // bolts to the knee output
    with_output(T([0, D_KNEE, 0]) * AZ_TO_Y, R_CLEAR_KNEE, KNEE_ZBOSS, KNEE_OUT_CLOCK) stock("left_knee_link");
}

module sided() { if (side == "right") mirror([0, 1, 0]) children(); else children(); }

// ---------------- assembly (zero pose) ----------------
module actuator() {
    color("#e07b39") for (f = ["Housing_-_Bambu_-_Housing_Front.stl", "Housing_-_Bambu_-_Housing_Middle.stl",
                               "Housing_-_Bambu_-_Housing_Rear.stl"]) import(str(ACTUATOR_DIR, f));
    color("#5b8c5a") for (f = ["Output_-_Bambu_-_Output_Shaft_Front.stl", "Output_-_Bambu_-_Output_Shaft_Rear.stl"]) import(str(ACTUATOR_DIR, f));
}

module assembly() {
    color("#b8b8b8") pelvis_left_half();
    color("#8f9bb0") pelvis_right_half();
    left_leg_assembly();
    if (both_legs) mirror([0, 1, 0]) left_leg_assembly();   // right leg = mirror of the left (zero pose)
    color("#e0c060") waist_stator_carrier();
    color("#9fb4c8") waist_yaw_link();
    if (show_actuators) {
        multmatrix(PEL_L) actuator();
        multmatrix(PEL_R) actuator();
        multmatrix(WAIST_ACT) actuator();
    }
}

module left_leg_assembly() {
    multmatrix(J_PITCH) color("#d4b632") hip_pitch_link();
    multmatrix(J_PITCH * J_ROLL) color("#6fae6a") hip_roll_link();
    multmatrix(J_PITCH * J_ROLL * J_YAW) color("#c8c4d4") thigh();
    multmatrix(J_PITCH * J_ROLL * J_YAW * J_KNEE) color("#5b7fc9") shin();
    if (show_actuators) {
        multmatrix(J_PITCH * ROLL_ACT) actuator();
        multmatrix(J_PITCH * J_ROLL * YAW_ACT) actuator();
        multmatrix(J_PITCH * J_ROLL * J_YAW * KNEE_ACT) actuator();
    }
}

if (part == "assembly") assembly();
else if (part == "pelvis_left_half") pelvis_left_half();
else if (part == "pelvis_right_half") pelvis_right_half();
else if (part == "hip_pitch_link") sided() hip_pitch_link();
else if (part == "hip_roll_link") sided() hip_roll_link();
else if (part == "thigh") sided() thigh();
else if (part == "shin") sided() shin();
else if (part == "waist_yaw_link") waist_yaw_link();
else if (part == "waist_stator_carrier") waist_stator_carrier();
