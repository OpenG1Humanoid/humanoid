// =====================================================================
//  assembly.scad - the whole robot: pelvis + legs, torso, both 7-DoF servo arms (generated)
//  F5 (preview) is quick; F6 (full render) of everything is slow - use the
//  Manifold backend (Preferences > Features > manifold) for F6.
//  Joint angles in degrees: arms [shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw],
//  legs [hip pitch, hip roll, hip yaw, knee] (G1 URDF convention, both sides), waist yaw.
//  Frame: torso_link.
// =====================================================================
include <../lib/g1_servo_arm.scad>
include <../lib/torso_frame_lib.scad>   // framed torso (2020 aluminium + printed panels)
include <../lib/legs_lib.scad>          // pelvis + legs with the 6512 cycloidal actuators
part = "none";                      // stop g1_servo_arm.scad drawing a single part
qL = [0, 0, 0, 0, 0, 0, 0];         // left arm
qR = [0, 0, 0, 0, 0, 0, 0];         // right arm
qLegL = [0, 0, 0, 0, 0, 0];         // left leg: hip pitch, hip roll, hip yaw, knee, ankle pitch, ankle roll
qLegR = [0, 0, 0, 0, 0, 0];         // right leg
qWaist = 0;                         // waist yaw (deg)
show_legs     = true;
show_torso    = true;
torso_style   = "framed";           // "framed": 2020 frame + printable panels | "solid": one-piece printed torso
show_left     = true;
show_right    = true;
show_head     = true;               // printed dome head (STL/head)
show_hardware = true;               // servo cartridges, hubs, bearings and idler pins

module head_parts() {
    translate([3.9635, 0, -44]) {
        color("#5a5d66") import("../../STL/head/frame_mm.stl", convexity = 10);
        for (i = [0:4]) color("#6d7380") import(str("../../STL/head/panel_", i, "_mm.stl"), convexity = 10);
        color("#c9a227") { import("../../STL/head/brow_left_mm.stl", convexity = 10); import("../../STL/head/brow_right_mm.stl", convexity = 10); }
    }
}

module sided() { if ($side == "left") children(); else mirror([0,1,0]) children(); }

module arm(q) {
    s = ($side == "left") ? 1 : -1;      // right arm: URDF offsets mirrored in Y
    rotate([0, q[0], 0]) {
        color("SteelBlue") sided() { shoulder_pitch_link_main(); } if (show_hardware) color("Gold") sided() { multmatrix([[0.0000,0.0000,-1.0000,30.4400],[-1.0000,0.0000,0.0000,38.0000],[0.0000,1.0000,0.0000,-13.8310],[0.0000,0.0000,0.0000,1.0000]]) { hub_DS3235(); cartridge_roll(); cartridge_cap_roll(); cartridge_bearings_roll(); } }
        translate([0, s*38, -13.831]) rotate([s*-0.27925*180/PI, 0, 0]) rotate([q[1], 0, 0]) {
            color("DarkOrange") sided() { shoulder_roll_link_main(); } if (show_hardware) color("Gold") sided() { multmatrix([[1.0000,0.0000,0.0000,0.0000],[0.0000,1.0000,0.0000,6.2400],[0.0000,0.0000,1.0000,-103.2000],[0.0000,0.0000,0.0000,1.0000]]) { hub_X10PRO(); cartridge_yaw(); cartridge_cap_yaw(); cartridge_bearings_yaw(); } }
            translate([0, s*6.24, -103.2]) rotate([0, 0, q[2]]) {
                color("ForestGreen") sided() { shoulder_yaw_link_main(); } if (show_hardware) color("Gold") sided() { multmatrix([[1.0000,0.0000,0.0000,15.7830],[0.0000,0.0000,-1.0000,20.4800],[0.0000,1.0000,0.0000,-80.5180],[0.0000,0.0000,0.0000,1.0000]]) { hub_X10PRO(); cartridge_elbow(); cartridge_cap_elbow(); cartridge_bearings_elbow(); } }
                translate([15.783, 0, -80.518]) rotate([0, q[3], 0]) {
                    color("FireBrick") sided() { elbow_link_main(); } if (show_hardware) color("Gold") sided() { multmatrix([[0.0000,0.0000,-1.0000,99.5000],[1.0000,0.0000,0.0000,1.8880],[0.0000,-1.0000,0.0000,-10.0000],[0.0000,0.0000,0.0000,1.0000]]) { hub_MG92B_wristroll(); cartridge_wroll(); cartridge_cap_wroll(); cartridge_bearings_wroll(); } }
                    translate([100, s*1.888, -10]) rotate([q[4], 0, 0]) {
                        color("MediumPurple") sided() { wrist_roll_link_main(); wrist_roll_link_cap_wpitch(); wrist_roll_link_cap_wpitch_idler(); } if (show_hardware) color("Gold") sided() {  }
                        translate([38, 0, 0]) rotate([0, q[5], 0]) {
                            color("SaddleBrown") sided() { wrist_pitch_link_main(); wrist_pitch_link_plug_wpitch(); wrist_pitch_link_plug_wyaw(); } if (show_hardware) color("Gold") sided() {  multmatrix([[0.0000,1.0000,0.0000,0.0000],[0.0000,0.0000,-1.0000,20.5000],[-1.0000,0.0000,0.0000,0.0000],[0.0000,0.0000,0.0000,1.0000]]) hub_MG92B_cover(); multmatrix([[-0.0000,-1.0000,0.0000,46.0000],[-1.0000,0.0000,0.0000,0.0000],[-0.0000,0.0000,-1.0000,20.5000],[0.0000,0.0000,0.0000,1.0000]]) hub_MG92B_cover(); multmatrix([[0.0000,1.0000,0.0000,0.0000],[0.0000,0.0000,1.0000,-22.5000],[1.0000,0.0000,0.0000,0.0000],[0.0000,0.0000,0.0000,1.0000]]) idler_pin(); multmatrix([[0.0000,-1.0000,0.0000,46.0000],[1.0000,0.0000,0.0000,0.0000],[-0.0000,0.0000,1.0000,-22.5000],[0.0000,0.0000,0.0000,1.0000]]) idler_pin(); }
                            translate([46, 0, 0]) rotate([0, 0, q[6]]) {
                                color("HotPink") sided() { wrist_yaw_link_main(); wrist_yaw_link_cap_wyaw(); wrist_yaw_link_cap_wyaw_idler(); } if (show_hardware) color("Gold") sided() {  }
                            }
                        }
                    }
                }
            }
        }
    }
}

if (show_torso) {
    if (torso_style == "framed") torso_framed();
    else color("Gainsboro") { torso_link_main(); torso_link_plug_pitchL(); torso_link_plug_pitchR(); }
    if (show_hardware) color("Gold") { multmatrix([[-0.0000,-1.0000,-0.0002,3.9562],[-0.2757,0.0002,-0.9612,99.6433],[0.9612,0.0001,-0.2757,247.6146],[0.0000,0.0000,0.0000,1.0000]]) { hub_DS3235(); bearing_6807(); } multmatrix([[0.0000,-1.0000,-0.0002,3.9562],[-0.2757,-0.0002,0.9612,-99.6433],[-0.9612,0.0001,-0.2757,247.6146],[0.0000,0.0000,0.0000,1.0000]]) { hub_DS3235(); bearing_6807(); } }
}
if (show_left)  multmatrix([[1.0000,0.0002,-0.0000,3.9563],[-0.0002,0.9612,-0.2757,100.2200],[-0.0001,0.2757,0.9612,247.7800],[0.0000,0.0000,0.0000,1.0000]]) { $side = "left";  arm(qL); }
if (show_right) multmatrix([[1.0000,-0.0002,-0.0000,3.9563],[0.0002,0.9612,0.2757,-100.2200],[-0.0001,-0.2757,0.9612,247.7800],[0.0000,0.0000,0.0000,1.0000]]) { $side = "right"; arm(qR); }
if (show_head) head_parts();       // dome head: frame, 5 panels, 2 brows (head_link frame = torso + [3.9635, 0, -44])
if (show_legs)  lower_body(qLegL, qLegR, qWaist);   // pelvis, legs, waist yaw link
