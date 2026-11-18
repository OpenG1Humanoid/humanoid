// =====================================================================
//  assembly.scad - torso + both 7-DoF servo arms, all in one (generated)
//  F5 (preview) is quick; F6 (full render) of everything is slow - use the
//  Manifold backend (Preferences > Features > manifold) for F6.
//  Joint angles in degrees: [shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw]
// =====================================================================
include <g1_servo_arm.scad>
part = "none";                      // stop g1_servo_arm.scad drawing a single part
qL = [0, 0, 0, 0, 0, 0, 0];         // left arm
qR = [0, 0, 0, 0, 0, 0, 0];         // right arm
show_torso    = true;
show_left     = true;
show_right    = true;
show_hardware = true;               // printed hubs and idler pins (gold)

module sided() { if ($side == "left") children(); else mirror([0,1,0]) children(); }

module arm(q) {
    s = ($side == "left") ? 1 : -1;      // right arm: URDF offsets mirrored in Y
    rotate([0, q[0], 0]) {
        color("SteelBlue") sided() { shoulder_pitch_link_main(); shoulder_pitch_link_plug_roll(); } if (show_hardware) color("Gold") sided() { multmatrix([[0.0000,0.0000,-1.0000,30.4400],[-1.0000,0.0000,0.0000,38.0000],[0.0000,1.0000,0.0000,-13.8310],[0.0000,0.0000,0.0000,1.0000]]) hub_DS3235(); }
        translate([0, s*38, -13.831]) rotate([s*-0.27925*180/PI, 0, 0]) rotate([q[1], 0, 0]) {
            color("DarkOrange") sided() { shoulder_roll_link_main(); shoulder_roll_link_plug_yaw(); } if (show_hardware) color("Gold") sided() { multmatrix([[1.0000,0.0000,0.0000,0.0000],[0.0000,1.0000,0.0000,6.2400],[0.0000,0.0000,1.0000,-103.2000],[0.0000,0.0000,0.0000,1.0000]]) hub_X10PRO(); }
            translate([0, s*6.24, -103.2]) rotate([0, 0, q[2]]) {
                color("ForestGreen") sided() { shoulder_yaw_link_main(); shoulder_yaw_link_plug_elbow(); } if (show_hardware) color("Gold") sided() { multmatrix([[1.0000,0.0000,0.0000,15.7830],[0.0000,0.0000,-1.0000,20.4800],[0.0000,1.0000,0.0000,-80.5180],[0.0000,0.0000,0.0000,1.0000]]) hub_X10PRO(); }
                translate([15.783, 0, -80.518]) rotate([0, q[3], 0]) {
                    color("FireBrick") sided() { elbow_link_main(); elbow_link_plug_wroll(); } if (show_hardware) color("Gold") sided() { multmatrix([[0.0000,0.0000,-1.0000,99.5000],[1.0000,0.0000,0.0000,1.8880],[0.0000,-1.0000,0.0000,-10.0000],[0.0000,0.0000,0.0000,1.0000]]) hub_MG92B_wristroll(); }
                    translate([100, s*1.888, -10]) rotate([q[4], 0, 0]) {
                        color("MediumPurple") sided() { wrist_roll_link_main(); wrist_roll_link_cap_wpitch(); wrist_roll_link_cap_wpitch_idler(); } if (show_hardware) color("Gold") sided() {  }
                        translate([38, 0, 0]) rotate([0, q[5], 0]) {
                            color("SaddleBrown") sided() { wrist_pitch_link_main(); wrist_pitch_link_plug_wpitch(); wrist_pitch_link_plug_wyaw(); } if (show_hardware) color("Gold") sided() { multmatrix([[0.0000,1.0000,0.0000,0.0000],[0.0000,0.0000,-1.0000,20.5000],[-1.0000,0.0000,0.0000,0.0000],[0.0000,0.0000,0.0000,1.0000]]) hub_MG92B_cover(); multmatrix([[-0.0000,-1.0000,0.0000,46.0000],[-1.0000,0.0000,0.0000,0.0000],[-0.0000,0.0000,-1.0000,20.5000],[0.0000,0.0000,0.0000,1.0000]]) hub_MG92B_cover(); multmatrix([[0.0000,1.0000,0.0000,0.0000],[0.0000,0.0000,1.0000,-22.5000],[1.0000,0.0000,0.0000,0.0000],[0.0000,0.0000,0.0000,1.0000]]) idler_pin(); multmatrix([[0.0000,-1.0000,0.0000,46.0000],[1.0000,0.0000,0.0000,0.0000],[-0.0000,0.0000,1.0000,-22.5000],[0.0000,0.0000,0.0000,1.0000]]) idler_pin(); }
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
    color("Gainsboro") { torso_link_main(); torso_link_plug_pitchL(); torso_link_plug_pitchR(); }
    if (show_hardware) color("Gold") { multmatrix([[-0.0000,-1.0000,-0.0002,3.9562],[-0.2757,0.0002,-0.9612,99.6433],[0.9612,0.0001,-0.2757,247.6146],[0.0000,0.0000,0.0000,1.0000]]) hub_DS3235(); multmatrix([[0.0000,-1.0000,-0.0002,3.9562],[-0.2757,-0.0002,0.9612,-99.6433],[-0.9612,0.0001,-0.2757,247.6146],[0.0000,0.0000,0.0000,1.0000]]) hub_DS3235(); }
}
if (show_left)  multmatrix([[1.0000,0.0002,-0.0000,3.9563],[-0.0002,0.9612,-0.2757,100.2200],[-0.0001,0.2757,0.9612,247.7800],[0.0000,0.0000,0.0000,1.0000]]) { $side = "left";  arm(qL); }
if (show_right) multmatrix([[1.0000,-0.0002,-0.0000,3.9563],[0.0002,0.9612,0.2757,-100.2200],[-0.0001,-0.2757,0.9612,247.7800],[0.0000,0.0000,0.0000,1.0000]]) { $side = "right"; arm(qR); }
