// Open G1 pelvis + both legs (finished STLs from STL/legs/), posable.
// Angles in degrees, G1 URDF convention for both legs: [hip_pitch, hip_roll, hip_yaw, knee].
// For printing: export the parts from STL/legs/ directly (they are already the final meshes).
include <../lib/legs_lib.scad>
qLeft  = [0, 0, 0, 0];
qRight = [0, 0, 0, 0];
show_leg_actuators = true;
legs(qLeft, qRight);
