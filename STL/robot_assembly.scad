// OpenG1 full robot assembly. Put this file in STL/ (next to arms/, legs/, torso_frame/, head/). F5 to preview.
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
show_right = true;
show_legs = true;
show_torso = true;
show_head = true;
knee_variant = "ankle"; // [ankle, integrated]
pelvis_variant = "waist"; // [waist, integrated]
show_links = true;
show_actuators = true;
links_transparent = false;
/* [Hidden] */
module link_col(){ if (links_transparent) color([0.8,0.8,0.85,0.35]) children(); else color([0.82,0.83,0.86]) children(); }
module left_wrist_yaw_link(){
  multmatrix([[1.000000,0.000000,0.000000,46.000000],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,0.000000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_wrist_yaw, v=[0,0,1]) {
    if (show_links) link_col() import("arms/left/links/left_wrist_yaw_link.stl", convexity=10);
    if (show_actuators) {
      color([0.3,0.8,0.3]) multmatrix([[-1.000000,0.000000,0.000000,-0.001055],[0.000000,1.000000,0.000000,0.002086],[0.000000,0.000000,-1.000000,20.600972],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/wrist_yaw/left_wrist_yaw_hornplate.stl", convexity=10);
    }
  }
}
module left_wrist_pitch_link(){
  multmatrix([[1.000000,0.000000,0.000000,38.000000],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,0.000000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_wrist_pitch, v=[0,1,0]) {
    if (show_links) link_col() import("arms/left/links/left_wrist_pitch_link.stl", convexity=10);
    if (show_actuators) {
      color([0.2,0.6,0.9]) multmatrix([[-1.000000,0.000000,0.000000,-2.247583],[0.000000,0.000000,1.000000,-22.492571],[0.000000,1.000000,0.000000,3.019777],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/wrist_pitch/left_wrist_pitch_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[-1.000000,0.000000,0.000000,-2.244570],[0.000000,0.000000,1.000000,-15.502569],[0.000000,1.000000,0.000000,3.022455],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/wrist_pitch/left_wrist_pitch_cartridge.stl", convexity=10);
      color([0.8,0.2,0.6]) multmatrix([[-1.000000,0.000000,0.000000,-0.003685],[0.000000,0.000000,1.000000,-30.040098],[0.000000,1.000000,0.000000,0.002256],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/wrist_pitch/left_wrist_pitch_idler_pin.stl", convexity=10);
      color([0.2,0.6,0.9]) multmatrix([[-1.000000,0.000000,0.000000,43.622897],[0.000000,-1.000000,0.000000,-3.019826],[0.000000,0.000000,1.000000,-22.493307],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/wrist_yaw/left_wrist_yaw_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[-1.000000,0.000000,0.000000,43.622898],[0.000000,-1.000000,0.000000,-3.022470],[0.000000,0.000000,1.000000,-15.503290],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/wrist_yaw/left_wrist_yaw_cartridge.stl", convexity=10);
      color([0.8,0.2,0.6]) multmatrix([[-1.000000,0.000000,0.000000,45.996315],[0.000000,-1.000000,0.000000,-0.002256],[0.000000,0.000000,1.000000,-30.040098],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/wrist_yaw/left_wrist_yaw_idler_pin.stl", convexity=10);
    }
    left_wrist_yaw_link();
  }
}
module left_wrist_roll_link(){
  multmatrix([[1.000000,0.000000,0.000000,100.000000],[0.000000,1.000000,0.000000,1.887910],[0.000000,0.000000,1.000000,-10.000000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_wrist_roll, v=[1,0,0]) {
    if (show_links) link_col() import("arms/left/links/left_wrist_roll_link.stl", convexity=10);
    if (show_actuators) {
      color([0.3,0.8,0.3]) multmatrix([[0.000000,0.000000,-1.000000,0.000352],[0.000000,1.000000,0.000000,0.000807],[1.000000,0.000000,0.000000,-0.002433],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/wrist_roll/left_wrist_roll_hornplate.stl", convexity=10);
      color([0.3,0.8,0.3]) multmatrix([[-1.000000,0.000000,0.000000,37.998945],[0.000000,0.000000,-1.000000,20.600972],[0.000000,-1.000000,0.000000,-0.002086],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/wrist_pitch/left_wrist_pitch_hornplate.stl", convexity=10);
    }
    left_wrist_pitch_link();
  }
}
module left_elbow_link(){
  multmatrix([[1.000000,0.000000,0.000000,15.783000],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,-80.518000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_elbow, v=[0,1,0]) {
    if (show_links) link_col() import("arms/left/links/left_elbow_link.stl", convexity=10);
    if (show_actuators) {
      color([0.2,0.6,0.9]) multmatrix([[0.000000,0.000000,-1.000000,99.570101],[0.000000,1.000000,0.000000,5.162960],[1.000000,0.000000,0.000000,-9.895393],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/wrist_roll/left_wrist_roll_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[0.000000,0.000000,-1.000000,85.972095],[0.000000,1.000000,0.000000,5.163204],[1.000000,0.000000,0.000000,-9.895439],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/wrist_roll/left_wrist_roll_cartridge.stl", convexity=10);
    }
    left_wrist_roll_link();
  }
}
module left_shoulder_yaw_link(){
  multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,1.000000,0.000000,6.240000],[0.000000,0.000000,1.000000,-103.200000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_shoulder_yaw, v=[0,0,1]) {
    if (show_links) link_col() import("arms/left/links/left_shoulder_yaw_link.stl", convexity=10);
    if (show_actuators) {
      color([0.3,0.8,0.3]) multmatrix([[1.000000,0.000000,0.000000,-0.000960],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,-6.700665],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/shoulder_yaw/left_shoulder_yaw_hornplate.stl", convexity=10);
      color([0.2,0.6,0.9]) multmatrix([[-0.559193,-0.829038,0.000000,10.051072],[0.000000,0.000000,-1.000000,20.041660],[0.829038,-0.559193,0.000000,-72.019813],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/elbow/left_elbow_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[-0.559193,-0.829038,0.000000,10.050037],[0.000000,0.000000,-1.000000,18.040907],[0.829038,-0.559193,0.000000,-72.021069],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/elbow/left_elbow_cartridge.stl", convexity=10);
    }
    left_elbow_link();
  }
}
module left_shoulder_roll_link(){
  multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,0.961262,0.275635,38.000000],[0.000000,-0.275635,0.961262,-13.831000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_shoulder_roll, v=[1,0,0]) {
    if (show_links) link_col() import("arms/left/links/left_shoulder_roll_link.stl", convexity=10);
    if (show_actuators) {
      color([0.3,0.8,0.3]) multmatrix([[0.000000,0.000000,-1.000000,30.602095],[-0.615659,0.788012,0.000000,0.000000],[0.788012,0.615659,0.000000,0.000000],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/shoulder_roll/left_shoulder_roll_hornplate.stl", convexity=10);
      color([0.2,0.6,0.9]) multmatrix([[1.000000,0.000000,0.000000,5.400001],[0.000000,-1.000000,0.000000,6.239927],[0.000000,0.000000,-1.000000,-62.349899],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/shoulder_yaw/left_shoulder_yaw_backcap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[1.000000,0.000000,0.000000,0.000321],[0.000000,1.000000,0.000000,6.239340],[0.000000,0.000000,1.000000,-102.950101],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/shoulder_yaw/left_shoulder_yaw_cartridge.stl", convexity=10);
    }
    left_shoulder_yaw_link();
  }
}
module left_shoulder_pitch_link(){
  multmatrix([[1.000000,0.000000,0.000000,3.956300],[0.000000,0.961246,-0.275692,100.220000],[0.000000,0.275692,0.961246,247.780000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_shoulder_pitch, v=[0,1,0]) {
    if (show_links) link_col() import("arms/left/links/left_shoulder_pitch_link.stl", convexity=10);
    if (show_actuators) {
      color([0.2,0.6,0.9]) multmatrix([[0.000000,0.000000,1.000000,-30.250099],[-0.374607,-0.927184,0.000000,34.375683],[0.927184,-0.374607,0.000000,-5.858687],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/shoulder_roll/left_shoulder_roll_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[0.000000,0.000000,1.000000,-21.500114],[-0.374607,-0.927184,0.000000,34.376503],[0.927184,-0.374607,0.000000,-5.859738],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/shoulder_roll/left_shoulder_roll_cartridge.stl", convexity=10);
      color([0.8,0.2,0.6]) multmatrix([[0.000000,0.000000,1.000000,-39.900101],[-0.374607,-0.927184,0.000000,38.000874],[0.927184,-0.374607,0.000000,-13.833279],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/shoulder_roll/left_shoulder_roll_idler_pin.stl", convexity=10);
      color([0.3,0.8,0.3]) multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,0.000000,-1.000000,8.500289],[0.000000,1.000000,0.000000,0.003860],[0.000000,0.000000,0.000000,1.000000]]) import("arms/left/actuators/shoulder_pitch/left_shoulder_pitch_hornhub.stl", convexity=10);
    }
    left_shoulder_roll_link();
  }
}
module right_wrist_yaw_link(){
  multmatrix([[1.000000,0.000000,0.000000,46.000000],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,0.000000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_wrist_yaw, v=[0,0,1]) {
    if (show_links) link_col() import("arms/right/links/right_wrist_yaw_link.stl", convexity=10);
    if (show_actuators) {
      color([0.3,0.8,0.3]) multmatrix([[-1.000000,0.000000,0.000000,-0.001055],[0.000000,1.000000,0.000000,-0.002086],[0.000000,0.000000,-1.000000,20.600972],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/wrist_yaw/right_wrist_yaw_hornplate.stl", convexity=10);
    }
  }
}
module right_wrist_pitch_link(){
  multmatrix([[1.000000,0.000000,0.000000,38.000000],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,0.000000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_wrist_pitch, v=[0,1,0]) {
    if (show_links) link_col() import("arms/right/links/right_wrist_pitch_link.stl", convexity=10);
    if (show_actuators) {
      color([0.2,0.6,0.9]) multmatrix([[-1.000000,0.000000,0.000000,-2.244756],[0.000000,0.000000,-1.000000,22.492571],[0.000000,-1.000000,0.000000,3.019777],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/wrist_pitch/right_wrist_pitch_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[-1.000000,0.000000,0.000000,-2.244570],[0.000000,0.000000,-1.000000,15.502569],[0.000000,-1.000000,0.000000,3.022455],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/wrist_pitch/right_wrist_pitch_cartridge.stl", convexity=10);
      color([0.8,0.2,0.6]) multmatrix([[-1.000000,0.000000,0.000000,-0.003685],[0.000000,0.000000,-1.000000,30.040098],[0.000000,-1.000000,0.000000,0.002256],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/wrist_pitch/right_wrist_pitch_idler_pin.stl", convexity=10);
      color([0.2,0.6,0.9]) multmatrix([[-1.000000,0.000000,0.000000,43.622897],[0.000000,-1.000000,0.000000,3.019826],[0.000000,0.000000,1.000000,-22.493307],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/wrist_yaw/right_wrist_yaw_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[-1.000000,0.000000,0.000000,43.622898],[0.000000,-1.000000,0.000000,3.022470],[0.000000,0.000000,1.000000,-15.503290],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/wrist_yaw/right_wrist_yaw_cartridge.stl", convexity=10);
      color([0.8,0.2,0.6]) multmatrix([[-1.000000,0.000000,0.000000,45.996315],[0.000000,-1.000000,0.000000,0.002256],[0.000000,0.000000,1.000000,-30.040098],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/wrist_yaw/right_wrist_yaw_idler_pin.stl", convexity=10);
    }
    right_wrist_yaw_link();
  }
}
module right_wrist_roll_link(){
  multmatrix([[1.000000,0.000000,0.000000,100.000000],[0.000000,1.000000,0.000000,-1.887910],[0.000000,0.000000,1.000000,-10.000000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_wrist_roll, v=[1,0,0]) {
    if (show_links) link_col() import("arms/right/links/right_wrist_roll_link.stl", convexity=10);
    if (show_actuators) {
      color([0.3,0.8,0.3]) multmatrix([[0.000000,0.000000,-1.000000,0.000352],[0.000000,1.000000,0.000000,-0.000807],[1.000000,0.000000,0.000000,-0.002433],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/wrist_roll/right_wrist_roll_hornplate.stl", convexity=10);
      color([0.3,0.8,0.3]) multmatrix([[-1.000000,0.000000,0.000000,37.998945],[0.000000,0.000000,1.000000,-20.600972],[0.000000,1.000000,0.000000,-0.002086],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/wrist_pitch/right_wrist_pitch_hornplate.stl", convexity=10);
    }
    right_wrist_pitch_link();
  }
}
module right_elbow_link(){
  multmatrix([[1.000000,0.000000,0.000000,15.783000],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,-80.518000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_elbow, v=[0,1,0]) {
    if (show_links) link_col() import("arms/right/links/right_elbow_link.stl", convexity=10);
    if (show_actuators) {
      color([0.2,0.6,0.9]) multmatrix([[0.000000,0.000000,-1.000000,99.570101],[0.000000,1.000000,0.000000,-5.162960],[1.000000,0.000000,0.000000,-9.895632],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/wrist_roll/right_wrist_roll_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[0.000000,0.000000,-1.000000,85.972095],[0.000000,1.000000,0.000000,-5.163212],[1.000000,0.000000,0.000000,-9.895439],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/wrist_roll/right_wrist_roll_cartridge.stl", convexity=10);
    }
    right_wrist_roll_link();
  }
}
module right_shoulder_yaw_link(){
  multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,1.000000,0.000000,-6.240000],[0.000000,0.000000,1.000000,-103.200000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_shoulder_yaw, v=[0,0,1]) {
    if (show_links) link_col() import("arms/right/links/right_shoulder_yaw_link.stl", convexity=10);
    if (show_actuators) {
      color([0.3,0.8,0.3]) multmatrix([[1.000000,0.000000,0.000000,-0.000960],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,-6.700665],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/shoulder_yaw/right_shoulder_yaw_hornplate.stl", convexity=10);
      color([0.2,0.6,0.9]) multmatrix([[-0.559193,0.829038,0.000000,10.051387],[0.000000,0.000000,1.000000,-20.041660],[0.829038,0.559193,0.000000,-72.019600],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/elbow/right_elbow_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[-0.559193,0.829038,0.000000,10.050037],[0.000000,0.000000,1.000000,-18.040907],[0.829038,0.559193,0.000000,-72.021069],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/elbow/right_elbow_cartridge.stl", convexity=10);
    }
    right_elbow_link();
  }
}
module right_shoulder_roll_link(){
  multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,0.961262,-0.275635,-38.000000],[0.000000,0.275635,0.961262,-13.831000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_shoulder_roll, v=[1,0,0]) {
    if (show_links) link_col() import("arms/right/links/right_shoulder_roll_link.stl", convexity=10);
    if (show_actuators) {
      color([0.3,0.8,0.3]) multmatrix([[0.000000,0.000000,-1.000000,30.602095],[0.615659,0.788012,0.000000,0.000000],[0.788012,-0.615659,0.000000,0.000000],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/shoulder_roll/right_shoulder_roll_hornplate.stl", convexity=10);
      color([0.2,0.6,0.9]) multmatrix([[1.000000,0.000000,0.000000,5.400001],[0.000000,-1.000000,0.000000,-6.239927],[0.000000,0.000000,-1.000000,-62.349899],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/shoulder_yaw/right_shoulder_yaw_backcap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[1.000000,0.000000,0.000000,0.000321],[0.000000,1.000000,0.000000,-6.239340],[0.000000,0.000000,1.000000,-102.950101],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/shoulder_yaw/right_shoulder_yaw_cartridge.stl", convexity=10);
    }
    right_shoulder_yaw_link();
  }
}
module right_shoulder_pitch_link(){
  multmatrix([[1.000000,0.000000,0.000000,3.956300],[0.000000,0.961246,0.275692,-100.220000],[0.000000,-0.275692,0.961246,247.780000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_shoulder_pitch, v=[0,1,0]) {
    if (show_links) link_col() import("arms/right/links/right_shoulder_pitch_link.stl", convexity=10);
    if (show_actuators) {
      color([0.2,0.6,0.9]) multmatrix([[0.000000,0.000000,1.000000,-30.250099],[0.374607,-0.927184,0.000000,-34.375859],[0.927184,0.374607,0.000000,-5.858676],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/shoulder_roll/right_shoulder_roll_cap.stl", convexity=10);
      color([0.95,0.55,0.15]) multmatrix([[0.000000,0.000000,1.000000,-21.500099],[0.374607,-0.927184,0.000000,-34.376508],[0.927184,0.374607,0.000000,-5.859750],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/shoulder_roll/right_shoulder_roll_cartridge.stl", convexity=10);
      color([0.8,0.2,0.6]) multmatrix([[0.000000,0.000000,1.000000,-39.900101],[0.374607,-0.927184,0.000000,-38.000874],[0.927184,0.374607,0.000000,-13.833279],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/shoulder_roll/right_shoulder_roll_idler_pin.stl", convexity=10);
      color([0.3,0.8,0.3]) multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,-8.500289],[0.000000,-1.000000,0.000000,0.003860],[0.000000,0.000000,0.000000,1.000000]]) import("arms/right/actuators/shoulder_pitch/right_shoulder_pitch_hornhub.stl", convexity=10);
    }
    right_shoulder_roll_link();
  }
}
// torso_link frame origin = G1 torso_link (waist yaw). Torso/legs not included.

/* [Legs and waist] */
l_hip_pitch = 0; // [-154:1:166]
l_hip_roll = 0; // [-30:1:170]
l_hip_yaw = 0; // [-158:1:158]
l_knee = 0; // [-5:1:166]
l_ankle_pitch = 0; // [-50:1:30]
l_ankle_roll = 0; // [-15:1:15]
r_hip_pitch = 0; // [-154:1:166]
r_hip_roll = 0; // [-30:1:170]
r_hip_yaw = 0; // [-158:1:158]
r_knee = 0; // [-5:1:166]
r_ankle_pitch = 0; // [-50:1:30]
r_ankle_roll = 0; // [-15:1:15]
waist_yaw = 0; // [-150:1:150]
waist_roll = 0; // [-30:1:30]
waist_pitch = 0; // [-30:1:30]
/* [Hidden] */
module left_ankle_roll(){ multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,-17.558000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_ankle_roll, v=[1,0,0]) {
  color([0.82,0.83,0.86]) import("legs/left_ankle_roll_link_ears.stl", convexity=10);
}}
module left_ankle_pitch(){ multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,1.000000,0.000000,-0.094445],[0.000000,0.000000,1.000000,-300.010000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_ankle_pitch, v=[0,1,0]) {
  color([0.82,0.83,0.86]) scale(1000) import("legs/feet/left_ankle_pitch_link.STL", convexity=10);
  left_ankle_roll();
}}
module left_knee(){ multmatrix([[0.984744,0.000000,0.174010,-78.273000],[0.000000,1.000000,0.000000,2.148900],[-0.174010,0.000000,0.984744,-177.340000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_knee, v=[0,1,0]) {
  color([0.82,0.83,0.86]) if (knee_variant=="ankle") import("legs/left_knee_link_ankle.stl", convexity=10); else import("legs/left_knee_link_integrated.stl", convexity=10);
  left_ankle_pitch();
}}
module left_hip_yaw(){ multmatrix([[1.000000,0.000000,0.000000,25.001000],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,-124.120000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_hip_yaw, v=[0,0,1]) {
  color([0.82,0.83,0.86]) import("legs/left_hip_yaw_link_knee_integrated_yaw.stl", convexity=10);
  left_knee();
}}
module left_hip_roll(){ multmatrix([[0.984744,0.000000,-0.174010,0.000000],[0.000000,1.000000,0.000000,52.000000],[0.174010,0.000000,0.984744,-30.465000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_hip_roll, v=[1,0,0]) {
  color([0.82,0.83,0.86]) import("legs/left_hip_roll_link_integrated_yaw.stl", convexity=10);
  left_hip_yaw();
}}
module left_hip_pitch(){ multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,1.000000,0.000000,64.452000],[0.000000,0.000000,1.000000,-102.700000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=l_hip_pitch, v=[0,1,0]) {
  color([0.82,0.83,0.86]) import("legs/left_hip_pitch_link_integrated.stl", convexity=10);
  left_hip_roll();
}}
module right_ankle_roll(){ multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,-17.558000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_ankle_roll, v=[1,0,0]) {
  color([0.82,0.83,0.86]) import("legs/right_ankle_roll_link_ears.stl", convexity=10);
}}
module right_ankle_pitch(){ multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,1.000000,0.000000,0.094445],[0.000000,0.000000,1.000000,-300.010000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_ankle_pitch, v=[0,1,0]) {
  color([0.82,0.83,0.86]) scale(1000) import("legs/feet/right_ankle_pitch_link.STL", convexity=10);
  right_ankle_roll();
}}
module right_knee(){ multmatrix([[0.984744,0.000000,0.174010,-78.273000],[0.000000,1.000000,0.000000,-2.148900],[-0.174010,0.000000,0.984744,-177.340000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_knee, v=[0,1,0]) {
  color([0.82,0.83,0.86]) if (knee_variant=="ankle") import("legs/right_knee_link_ankle.stl", convexity=10); else import("legs/right_knee_link_integrated.stl", convexity=10);
  right_ankle_pitch();
}}
module right_hip_yaw(){ multmatrix([[1.000000,0.000000,0.000000,25.001000],[0.000000,1.000000,0.000000,0.000000],[0.000000,0.000000,1.000000,-124.120000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_hip_yaw, v=[0,0,1]) {
  color([0.82,0.83,0.86]) import("legs/right_hip_yaw_link_knee_integrated_yaw.stl", convexity=10);
  right_knee();
}}
module right_hip_roll(){ multmatrix([[0.984744,0.000000,-0.174010,0.000000],[0.000000,1.000000,0.000000,-52.000000],[0.174010,0.000000,0.984744,-30.465000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_hip_roll, v=[1,0,0]) {
  color([0.82,0.83,0.86]) import("legs/right_hip_roll_link_integrated_yaw.stl", convexity=10);
  right_hip_yaw();
}}
module right_hip_pitch(){ multmatrix([[1.000000,0.000000,0.000000,0.000000],[0.000000,1.000000,0.000000,-64.452000],[0.000000,0.000000,1.000000,-102.700000],[0.000000,0.000000,0.000000,1.000000]]) rotate(a=r_hip_pitch, v=[0,1,0]) {
  color([0.82,0.83,0.86]) import("legs/right_hip_pitch_link_integrated.stl", convexity=10);
  right_hip_roll();
}}
module torso_link(){
  if (show_torso) color([0.75,0.78,0.82]) for (f=["back_panel","bottom_plate","compute_tray","driver_tray","front_panel","hatch_L","hatch_R","shoulder_block_L","shoulder_block_R","top_panel"]) import(str("torso_frame/",f,".stl"), convexity=10);
  if (show_left) left_shoulder_pitch_link();
  if (show_right) right_shoulder_pitch_link();
  if (show_head) translate([3.9635,0,-44]) color([0.3,0.3,0.32]) { import("head/frame_mm.stl", convexity=10); for (i=[0:4]) import(str("head/panel_",i,"_mm.stl"), convexity=10); import("head/brow_left_mm.stl"); import("head/brow_right_mm.stl"); }
}
module robot(){
  // pelvis frame
  if (show_legs) color([0.82,0.83,0.86]) { if (pelvis_variant=="waist") { import("legs/pelvis_left_half_waist.stl", convexity=10); import("legs/pelvis_right_half_waist.stl", convexity=10);} else { import("legs/pelvis_left_half_integrated.stl", convexity=10); import("legs/pelvis_right_half_integrated.stl", convexity=10);} import("legs/waist_stator_carrier.stl", convexity=10); }
  if (show_legs) { left_hip_pitch(); right_hip_pitch(); }
  rotate(a=waist_yaw, v=[0,0,1]) {
    color([0.82,0.83,0.86]) import("legs/waist_yaw_link_integrated.stl", convexity=10);
    translate([-3.9635,0,44]) rotate(a=waist_roll, v=[1,0,0]) rotate(a=waist_pitch, v=[0,1,0]) torso_link();
  }
}
// pelvis at G1 standing height (feet on z=0 at zero pose)
translate([0,0,793]) robot();
