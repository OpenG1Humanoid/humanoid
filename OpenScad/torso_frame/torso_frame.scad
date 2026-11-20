include <../lib/torso_frame_lib.scad>
PART = "assembly";

if (PART == "assembly") torso_framed(0);
else if (PART == "exploded") torso_framed(EXPLODE);
else if (PART == "frame") color("LightSteelBlue") frame();
else printed(PART);
