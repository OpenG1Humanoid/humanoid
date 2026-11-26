 // OpenG1 servo arm assembly - open in OpenSCAD (F5 preview). Keep this file in the zip root.
// Joint angles in degrees (G1 zero pose = all 0).
/* [Left arm] */
l_shoulder_pitch = 0; // [-177:1:153]
l_shoulder_roll = 0; // [-91:1:129]
l_shoulder_yaw = 0; // [-150:1:150]
l_elbow = 0; // [-60:1:120]
l_wrist_roll = 0; // [-113:1:113]
l_wrist_pitch = 0; // [-92:1:92]
l_wrist_yaw = 0; // [-92:1:92]
/* [Right arm] */
r_shoulder_pitch = 0; // [-177:1:153]
r_shoulder_roll = 0; // [-91:1:129]
r_shoulder_yaw = 0; // [-150:1:150]
r_elbow = 0; // [-60:1:120]
r_wrist_roll = 0; // [-113:1:113]
r_wrist_pitch = 0; // [-92:1:92]
r_wrist_yaw = 0; // [-92:1:92]
/* [Display] */
show_left = true;
show_right = false;
show_links = false;
show_actuators = true;
links_transparent = false;
/* [Hidden] */
module link_col(){ if (links_transparent) color([0.8,0.8,0.85,0.35]) children(); else color([0.82,0.83,0.86]) children(); }
module left_wrist_yaw_link(){
  multmatrix([[1.000000,0.000000,0.000000,46.000000],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,0.000000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_wrist_yaw, v=[0,0,1]) {
    if (show_links) link_col() import("left/links/left_wrist_yaw_link.stl", convexity=10);
    if (show_actuators) {
      color([0.3,0.8,0.3]) multmatrix([[-1.000000,0.000000,0.000000,-0.001055],[0.000000,1.000000,0.000000,0.002086],[0.000000,0.000000,-1.000000,20.600972],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/wrist_yaw/left_wrist_yaw_hornplate.stl", convexity=10);
    }
  }
}
module left_wrist_pitch_link(){
  multmatrix([[1.000000,0.000000,0.000000,38.000000],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,0.000000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_wrist_pitch, v=[0,1,0]) {
    if (show_links) link_col() import("left/links/left_wrist_pitch_link.stl", convexity=10);
    if (show_actuators) {
      color([0.2,0.6,0.9]) multmatrix([[-1.000000,0.000000,0.000000,-2.247583],[0.000000,0.000000,1.000000,-22.492571],[0.000000,1.000000,0.000000,3.019777],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/wrist_pitch/left_wrist_pitch_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[-1.000000,0.000000,0.000000,-2.244570],[0.000000,0.000000,1.000000,-15.502569],[0.000000,1.000000,0.000000,3.022455],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/wrist_pitch/left_wrist_pitch_cartridge.stl", convexity=10);
      color([0.8,0.2,0.6]) multmatrix([[-1.000000,0.000000,0.000000,-0.003685],[0.000000,0.000000,1.000000,-30.040098],[0.000000,1.000000,0.000000,0.002256],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/wrist_pitch/left_wrist_pitch_idler_pin.stl", convexity=10);
      color([0.2,0.6,0.9]) multmatrix([[-1.000000,0.000000,0.000000,43.622897],[0.000000,-1.000000,0.000000,-3.019826],[0.000000,0.000000,1.000000,-22.493307],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/wrist_yaw/left_wrist_yaw_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[-1.000000,0.000000,0.000000,43.622898],[0.000000,-1.000000,0.000000,-3.022470],[0.000000,0.000000,1.000000,-15.503290],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/wrist_yaw/left_wrist_yaw_cartridge.stl", convexity=10);
      color([0.8,0.2,0.6]) multmatrix([[-1.000000,0.000000,0.000000,45.996315],[0.000000,-1.000000,0.000000,-0.002256],[0.000000,0.000000,1.000000,-30.040098],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/wrist_yaw/left_wrist_yaw_idler_pin.stl", convexity=10);
    }
    left_wrist_yaw_link();
  }
}
module left_wrist_roll_link(){
  multmatrix([[1.000000,0.000000,0.000000,100.000000],[0.000000,1.000000,0.000000,1.887910],[0.000000,0.000000,1.000000,-10.000000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_wrist_roll, v=[1,0,0]) {
    if (show_links) link_col() import("left/links/left_wrist_roll_link.stl", convexity=10);
    if (show_actuators) {
      color([0.3,0.8,0.3]) multmatrix([[0.000000,0.000000,-1.000000,0.000352],[0.000000,1.000000,0.000000,0.000807],[1.000000,0.000000,0.000000,-0.002433],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/wrist_roll/left_wrist_roll_hornplate.stl", convexity=10);
      color([0.3,0.8,0.3]) multmatrix([[-1.000000,0.000000,0.000000,37.998945],[0.000000,0.000000,-1.000000,20.600972],[0.000000,-1.000000,0.000000,-0.002086],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/wrist_pitch/left_wrist_pitch_hornplate.stl", convexity=10);
    }
    left_wrist_pitch_link();
  }
}
module left_elbow_link(){
  multmatrix([[1.000000,0.000000,0.000000,15.783000],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,-80.518000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_elbow, v=[0,1,0]) {
    if (show_links) link_col() import("left/links/left_elbow_link.stl", convexity=10);
    if (show_actuators) {
      color([0.2,0.6,0.9]) multmatrix([[0.000000,0.000000,-1.000000,99.570101],[0.000000,1.000000,0.000000,5.162960],[1.000000,0.000000,0.000000,-9.895393],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/wrist_roll/left_wrist_roll_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[0.000000,0.000000,-1.000000,85.972095],[0.000000,1.000000,0.000000,5.163204],[1.000000,0.000000,0.000000,-9.895439],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/wrist_roll/left_wrist_roll_cartridge.stl", convexity=10);
    }
    left_wrist_roll_link();
  }
}
module left_shoulder_yaw_link(){
  multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,1.000000,0.000000,6.240000],[0.000000,0.000000,1.000000,-103.200000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_shoulder_yaw, v=[0,0,1]) {
    if (show_links) link_col() import("left/links/left_shoulder_yaw_link.stl", convexity=10);
    if (show_actuators) {
      color([0.95,0.55,0.15]) multmatrix([[-0.707107,-0.707107,0.000000,-0.000212],[-0.707107,0.707107,0.000000,-0.000212],[0.000000,0.000000,-1.000000,-0.250000],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/shoulder_yaw/left_shoulder_yaw_cartridge.stl", convexity=10);
      color([0.2,0.6,0.9]) multmatrix([[-0.559193,-0.829038,0.000000,10.051072],[0.000000,0.000000,-1.000000,20.041660],[0.829038,-0.559193,0.000000,-72.019813],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/elbow/left_elbow_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[-0.559193,-0.829038,0.000000,10.050037],[0.000000,0.000000,-1.000000,18.040907],[0.829038,-0.559193,0.000000,-72.021069],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/elbow/left_elbow_cartridge.stl", convexity=10);
    }
    left_elbow_link();
  }
}
module left_shoulder_roll_link(){
  multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,0.961262,0.275635,38.000000],[0.000000,-0.275635,0.961262,-13.831000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_shoulder_roll, v=[1,0,0]) {
    if (show_links) link_col() import("left/links/left_shoulder_roll_link.stl", convexity=10);
    if (show_actuators) {
      color([0.3,0.8,0.3]) multmatrix([[0.000000,0.000000,-1.000000,30.602095],[-0.615659,0.788012,0.000000,0.000000],[0.788012,0.615659,0.000000,0.000000],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/shoulder_roll/left_shoulder_roll_hornplate.stl", convexity=10);
      color([0.3,0.8,0.3]) multmatrix([[-0.707107,-0.707107,0.000000,0.000707],[-0.707107,0.707107,0.000000,6.240707],[0.000000,0.000000,-1.000000,-96.499300],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/shoulder_yaw/left_shoulder_yaw_hornplate.stl", convexity=10);
    }
    left_shoulder_yaw_link();
  }
}
module left_shoulder_pitch_link(){
  multmatrix([[1.000000,0.000000,0.000000,3.956300],[0.000000,0.961246,-0.275692,100.220000],[0.000000,0.275692,0.961246,247.780000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_shoulder_pitch, v=[0,1,0]) {
    if (show_links) link_col() import("left/links/left_shoulder_pitch_link.stl", convexity=10);
    if (show_actuators) {
      color([0.2,0.6,0.9]) multmatrix([[0.000000,0.000000,1.000000,-30.250099],[-0.374607,-0.927184,0.000000,34.375683],[0.927184,-0.374607,0.000000,-5.858687],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/shoulder_roll/left_shoulder_roll_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[0.000000,0.000000,1.000000,-21.500114],[-0.374607,-0.927184,0.000000,34.376503],[0.927184,-0.374607,0.000000,-5.859738],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/shoulder_roll/left_shoulder_roll_cartridge.stl", convexity=10);
      color([0.8,0.2,0.6]) multmatrix([[0.000000,0.000000,1.000000,-39.900101],[-0.374607,-0.927184,0.000000,38.000874],[0.927184,-0.374607,0.000000,-13.833279],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/shoulder_roll/left_shoulder_roll_idler_pin.stl", convexity=10);
      color([0.3,0.8,0.3]) multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,0.000000,-1.000000,8.500289],[0.000000,1.000000,0.000000,0.003860],[0.000000,0.000000,0.000000,1.000000]]) import("left/actuators/shoulder_pitch/left_shoulder_pitch_hornhub.stl", convexity=10);
    }
    left_shoulder_roll_link();
  }
}
module right_wrist_yaw_link(){
  multmatrix([[1.000000,0.000000,0.000000,46.000000],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,0.000000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_wrist_yaw, v=[0,0,1]) {
    if (show_links) link_col() import("right/links/right_wrist_yaw_link.stl", convexity=10);
    if (show_actuators) {
      color([0.3,0.8,0.3]) multmatrix([[-1.000000,0.000000,0.000000,-0.001055],[0.000000,1.000000,0.000000,-0.002086],[0.000000,0.000000,-1.000000,20.600972],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/wrist_yaw/right_wrist_yaw_hornplate.stl", convexity=10);
    }
  }
}
module right_wrist_pitch_link(){
  multmatrix([[1.000000,0.000000,0.000000,38.000000],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,0.000000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_wrist_pitch, v=[0,1,0]) {
    if (show_links) link_col() import("right/links/right_wrist_pitch_link.stl", convexity=10);
    if (show_actuators) {
      color([0.2,0.6,0.9]) multmatrix([[-1.000000,0.000000,0.000000,-2.244756],[0.000000,0.000000,-1.000000,22.492571],[0.000000,-1.000000,0.000000,3.019777],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/wrist_pitch/right_wrist_pitch_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[-1.000000,0.000000,0.000000,-2.244570],[0.000000,0.000000,-1.000000,15.502569],[0.000000,-1.000000,0.000000,3.022455],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/wrist_pitch/right_wrist_pitch_cartridge.stl", convexity=10);
      color([0.8,0.2,0.6]) multmatrix([[-1.000000,0.000000,0.000000,-0.003685],[0.000000,0.000000,-1.000000,30.040098],[0.000000,-1.000000,0.000000,0.002256],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/wrist_pitch/right_wrist_pitch_idler_pin.stl", convexity=10);
      color([0.2,0.6,0.9]) multmatrix([[-1.000000,0.000000,0.000000,43.622897],[0.000000,-1.000000,0.000000,3.019826],[0.000000,0.000000,1.000000,-22.493307],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/wrist_yaw/right_wrist_yaw_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[-1.000000,0.000000,0.000000,43.622898],[0.000000,-1.000000,0.000000,3.022470],[0.000000,0.000000,1.000000,-15.503290],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/wrist_yaw/right_wrist_yaw_cartridge.stl", convexity=10);
      color([0.8,0.2,0.6]) multmatrix([[-1.000000,0.000000,0.000000,45.996315],[0.000000,-1.000000,0.000000,0.002256],[0.000000,0.000000,1.000000,-30.040098],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/wrist_yaw/right_wrist_yaw_idler_pin.stl", convexity=10);
    }
    right_wrist_yaw_link();
  }
}
module right_wrist_roll_link(){
  multmatrix([[1.000000,0.000000,0.000000,100.000000],[0.000000,1.000000,0.000000,-1.887910],[0.000000,0.000000,1.000000,-10.000000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_wrist_roll, v=[1,0,0]) {
    if (show_links) link_col() import("right/links/right_wrist_roll_link.stl", convexity=10);
    if (show_actuators) {
      color([0.3,0.8,0.3]) multmatrix([[0.000000,0.000000,-1.000000,0.000352],[0.000000,1.000000,0.000000,-0.000807],[1.000000,0.000000,0.000000,-0.002433],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/wrist_roll/right_wrist_roll_hornplate.stl", convexity=10);
      color([0.3,0.8,0.3]) multmatrix([[-1.000000,0.000000,0.000000,37.998945],[0.000000,0.000000,1.000000,-20.600972],[0.000000,1.000000,0.000000,-0.002086],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/wrist_pitch/right_wrist_pitch_hornplate.stl", convexity=10);
    }
    right_wrist_pitch_link();
  }
}
module right_elbow_link(){
  multmatrix([[1.000000,0.000000,0.000000,15.783000],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,-80.518000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_elbow, v=[0,1,0]) {
    if (show_links) link_col() import("right/links/right_elbow_link.stl", convexity=10);
    if (show_actuators) {
      color([0.2,0.6,0.9]) multmatrix([[0.000000,0.000000,-1.000000,99.570101],[0.000000,1.000000,0.000000,-5.162960],[1.000000,0.000000,0.000000,-9.895632],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/wrist_roll/right_wrist_roll_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[0.000000,0.000000,-1.000000,85.972095],[0.000000,1.000000,0.000000,-5.163212],[1.000000,0.000000,0.000000,-9.895439],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/wrist_roll/right_wrist_roll_cartridge.stl", convexity=10);
    }
    right_wrist_roll_link();
  }
}
module right_shoulder_yaw_link(){
  multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,1.000000,0.000000,-6.240000],[0.000000,0.000000,1.000000,-103.200000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_shoulder_yaw, v=[0,0,1]) {
    if (show_links) link_col() import("right/links/right_shoulder_yaw_link.stl", convexity=10);
    if (show_actuators) {
      color([0.95,0.55,0.15]) multmatrix([[-0.707107,0.707107,0.000000,-0.000212],[0.707107,0.707107,0.000000,0.000212],[0.000000,0.000000,-1.000000,-0.250000],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/shoulder_yaw/right_shoulder_yaw_cartridge.stl", convexity=10);
      color([0.2,0.6,0.9]) multmatrix([[-0.559193,0.829038,0.000000,10.051387],[0.000000,0.000000,1.000000,-20.041660],[0.829038,0.559193,0.000000,-72.019600],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/elbow/right_elbow_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[-0.559193,0.829038,0.000000,10.050037],[0.000000,0.000000,1.000000,-18.040907],[0.829038,0.559193,0.000000,-72.021069],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/elbow/right_elbow_cartridge.stl", convexity=10);
    }
    right_elbow_link();
  }
}
module right_shoulder_roll_link(){
  multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,0.961262,-0.275635,-38.000000],[0.000000,0.275635,0.961262,-13.831000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_shoulder_roll, v=[1,0,0]) {
    if (show_links) link_col() import("right/links/right_shoulder_roll_link.stl", convexity=10);
    if (show_actuators) {
      color([0.3,0.8,0.3]) multmatrix([[0.000000,0.000000,-1.000000,30.602095],[0.615659,0.788012,0.000000,0.000000],[0.788012,-0.615659,0.000000,0.000000],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/shoulder_roll/right_shoulder_roll_hornplate.stl", convexity=10);
      color([0.3,0.8,0.3]) multmatrix([[-0.707107,0.707107,0.000000,0.000707],[0.707107,0.707107,0.000000,-6.240707],[0.000000,0.000000,-1.000000,-96.499300],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/shoulder_yaw/right_shoulder_yaw_hornplate.stl", convexity=10);
    }
    right_shoulder_yaw_link();
  }
}
module right_shoulder_pitch_link(){
  multmatrix([[1.000000,0.000000,0.000000,3.956300],[0.000000,0.961246,0.275692,-100.220000],[0.000000,-0.275692,0.961246,247.780000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_shoulder_pitch, v=[0,1,0]) {
    if (show_links) link_col() import("right/links/right_shoulder_pitch_link.stl", convexity=10);
    if (show_actuators) {
      color([0.2,0.6,0.9]) multmatrix([[0.000000,0.000000,1.000000,-30.250099],[0.374607,-0.927184,0.000000,-34.375859],[0.927184,0.374607,0.000000,-5.858676],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/shoulder_roll/right_shoulder_roll_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[0.000000,0.000000,1.000000,-21.500099],[0.374607,-0.927184,0.000000,-34.376508],[0.927184,0.374607,0.000000,-5.859750],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/shoulder_roll/right_shoulder_roll_cartridge.stl", convexity=10);
      color([0.8,0.2,0.6]) multmatrix([[0.000000,0.000000,1.000000,-39.900101],[0.374607,-0.927184,0.000000,-38.000874],[0.927184,0.374607,0.000000,-13.833279],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/shoulder_roll/right_shoulder_roll_idler_pin.stl", convexity=10);
      color([0.3,0.8,0.3]) multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,-8.500289],[0.000000,-1.000000,0.000000,0.003860],[0.000000,0.000000,0.000000,1.000000]]) import("right/actuators/shoulder_pitch/right_shoulder_pitch_hornhub.stl", convexity=10);
    }
    right_shoulder_roll_link();
  }
}
if (show_left) left_shoulder_pitch_link();
if (show_right) right_shoulder_pitch_link();
// torso_link frame origin = G1 torso_link (waist yaw). Torso/legs not included.
