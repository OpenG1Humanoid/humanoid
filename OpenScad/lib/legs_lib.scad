// =====================================================================
//  legs_lib.scad - Open G1 pelvis + both legs with the 6512 cycloidal actuators (library)
// =====================================================================
//  Modules only. Open legs/legs.scad to view the legs, or assembly/assembly.scad for the robot.
//  Parts are the finished STLs in STL/legs/ (left from integrate_leg.py, right = mirror_right.py),
//  each in its own G1 link frame (mm). Joint frames are the G1 URDF's; joint angles in degrees,
//  same sign convention as the URDF for both legs: [hip_pitch, hip_roll, hip_yaw, knee].
// =====================================================================
legs_stl_dir = "../../STL/legs/";   // relative to the file you open (one folder below OpenScad/)
show_leg_actuators = true;          // actuator housings + outputs at hip pitch, hip roll and knee

LEG_RAD = 180 / PI;
function leg_T(v)  = [[1, 0, 0, v[0]], [0, 1, 0, v[1]], [0, 0, 1, v[2]], [0, 0, 0, 1]];
function leg_Rx(a) = [[1, 0, 0, 0], [0, cos(a), -sin(a), 0], [0, sin(a), cos(a), 0], [0, 0, 0, 1]];
function leg_Ry(a) = [[cos(a), 0, sin(a), 0], [0, 1, 0, 0], [-sin(a), 0, cos(a), 0], [0, 0, 0, 1]];
function leg_Rz(a) = [[cos(a), -sin(a), 0, 0], [sin(a), cos(a), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
LEG_MY = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
function leg_side(M, s) = s > 0 ? M : LEG_MY * M * LEG_MY;     // left frame -> right frame

// G1 URDF joint frames (s = +1 left, -1 right)
function LEG_J_PITCH(s) = leg_T([0, s * 64.452, -102.7]);
function LEG_J_ROLL(s)  = leg_T([0, s * 52, -30.465]) * leg_Ry(-0.1749 * LEG_RAD);
function LEG_J_YAW(s)   = leg_T([25.001, 0, -124.12]);
function LEG_J_KNEE(s)  = leg_T([-78.273, s * 2.1489, -177.34]) * leg_Ry(0.1749 * LEG_RAD);

// actuator frames (z out of the joint face, z = 0 on the output face), from g1_leg_actuators.scad
LEG_D_ROLL = 33.5014; LEG_D_KNEE = 15.3733; LEG_ROLL_CLOCK = 0;
LEG_AZ_TO_X = [[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]];
LEG_AZ_TO_Y = [[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]];
LEG_PEL_L = [[1, 0, 0, 0], [0, 0, 1, 64.452], [0, -1, 0, -102.7], [0, 0, 0, 1]];
LEG_PEL_R = [[-1, 0, 0, 0], [0, 0, -1, -64.452], [0, -1, 0, -102.7], [0, 0, 0, 1]];
function LEG_ROLL_ACT(s) = leg_side(LEG_J_ROLL(1) * leg_T([LEG_D_ROLL, 0, 0]) * LEG_AZ_TO_X * leg_Rz(LEG_ROLL_CLOCK), s);
function LEG_KNEE_ACT(s) = leg_side(LEG_J_KNEE(1) * leg_T([0, LEG_D_KNEE, 0]) * LEG_AZ_TO_Y, s);

module leg_stl(name) import(str(legs_stl_dir, name, ".stl"), convexity = 10);

module leg_actuator() {
    color("#e07b39") for (f = ["Housing_-_Bambu_-_Housing_Front", "Housing_-_Bambu_-_Housing_Middle",
                               "Housing_-_Bambu_-_Housing_Rear"]) leg_stl(str("actuator/", f));
    color("#5b8c5a") for (f = ["Output_-_Bambu_-_Output_Shaft_Front", "Output_-_Bambu_-_Output_Shaft_Rear"]) leg_stl(str("actuator/", f));
}

// waist yaw: the same actuator in the pelvis top, output face up at z = 2.2 (pelvis frame)
LEG_WAIST_ACT = leg_T([0, 0, 2.2]);

module pelvis() {
    color("#b8b8b8") leg_stl("pelvis_left_half_waist");
    color("#8f9bb0") leg_stl("pelvis_right_half_waist");
    color("#e0c060") leg_stl("waist_stator_carrier");
    if (show_leg_actuators) {
        multmatrix(LEG_PEL_L) leg_actuator(); multmatrix(LEG_PEL_R) leg_actuator();
        multmatrix(LEG_WAIST_ACT) leg_actuator();
    }
}

// waist yaw link (turns with the torso), in the pelvis frame at zero yaw
module waist_yaw_link() color("#9fb4c8") leg_stl("waist_yaw_link_integrated");

// one leg, in the pelvis frame. side = "left" | "right", q = [hip_pitch, hip_roll, hip_yaw, knee] (deg)
module leg(side, q = [0, 0, 0, 0]) {
    s = side == "left" ? 1 : -1;
    multmatrix(LEG_J_PITCH(s) * leg_Ry(q[0])) {
        color("#d4b632") leg_stl(str(side, "_hip_pitch_link_integrated"));
        if (show_leg_actuators) multmatrix(LEG_ROLL_ACT(s)) leg_actuator();
        multmatrix(LEG_J_ROLL(s) * leg_Rx(q[1])) {
            color("#6fae6a") leg_stl(str(side, "_hip_roll_link_integrated"));
            multmatrix(LEG_J_YAW(s) * leg_Rz(q[2])) {
                color("#c8c4d4") leg_stl(str(side, "_hip_yaw_link_knee_integrated"));
                if (show_leg_actuators) multmatrix(LEG_KNEE_ACT(s)) leg_actuator();
                multmatrix(LEG_J_KNEE(s) * leg_Ry(q[3])) color("#5b7fc9") leg_stl(str(side, "_knee_link_integrated"));
            }
        }
    }
}

module legs(qLeft = [0, 0, 0, 0], qRight = [0, 0, 0, 0]) { pelvis(); leg("left", qLeft); leg("right", qRight); }

// pelvis frame inside the torso_link frame at zero waist angles (G1 rev 1.0 waist)
LEG_PELVIS_IN_TORSO = leg_T([3.9635, 0, -44]);

// everything below the torso, drawn in the torso_link frame. qWaist = waist yaw (deg, G1 sign):
// the torso turns by +qWaist about the pelvis z axis, so the pelvis turns by -qWaist under it.
module lower_body(qLeft = [0, 0, 0, 0], qRight = [0, 0, 0, 0], qWaist = 0) {
    multmatrix(LEG_PELVIS_IN_TORSO) {
        waist_yaw_link();
        multmatrix(leg_Rz(-qWaist)) legs(qLeft, qRight);
    }
}
