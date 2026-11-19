// Open G1 framed torso. PART = "assembly" | "exploded" | "frame" | one printed part name:
//   front_panel back_panel top_panel shoulder_block_L shoulder_block_R hatch_L hatch_R
//   bottom_plate compute_tray driver_tray
include <../lib/torso_frame_lib.scad>
PART = "assembly";

if (PART == "assembly") torso_framed(0);
else if (PART == "exploded") torso_framed(EXPLODE);
else if (PART == "frame") color("LightSteelBlue") frame();
else printed(PART);
