# G1 left leg with integrated 6512 actuators: OpenSCAD version

`g1_leg_actuators.scad` builds the same parts as `integrate_leg.py`: both pelvis halves, the hip pitch link, hip roll link, thigh and shin, with the 6512 cycloidal actuator built into the hip pitch, hip roll, hip yaw and knee joints.

## What's in here

| File | What it is |
|---|---|
| `g1_leg_actuators.scad` | The model. Parameters at the top, one module per part. |
| `stock/*.stl` | Cleaned G1 links in mm, one solid each (the shin also has its two ankle push rods). |
| `Motor_Shell_-_Motor_Shell.stl` | Your motor shell. The stator floor in every housing is copied from it. |
| `Housing_*.stl`, `Output_*.stl` | Your actuator parts. Only used to show the actuators in the assembly view. |

Keep the folder layout as it is. The .scad finds the other files by relative path.

## Using it

1. Open `g1_leg_actuators.scad` in OpenSCAD.
2. Pick a part with the `part` setting at the top. You can also use the Customizer (Window > Customizer). The choices are `assembly`, `pelvis_left_half`, `pelvis_right_half`, `hip_pitch_link`, `hip_roll_link`, `thigh` and `shin`.
3. Preview with F5, render with F6, then export the STL.

The assembly shows every part at the zero pose. Set `show_actuators = false` to hide the actuator housings and outputs.

From the command line:

```
openscad -o thigh.stl -D 'part="thigh"' g1_leg_actuators.scad
```

Any version works. The 2021.01 release rendered a pelvis half in about 85 s and the hip roll link in about 25 s. A development snapshot with the Manifold backend switched on (Preferences > Advanced > 3D Rendering > Backend) is much faster.

## What you can change

The design parameters block (clearances, wall thickness, insert size, bolt and counterbore sizes, boss size, gap, window size, pelvis joining bolts and dowels) can be edited freely.

The "worked out by integrate_leg.py" block holds values the Python measured or chose at 1:1: where each output face sits, the bolt clockings and the boss heights. Leave those alone unless you rerun the Python. To change the scale, run `integrate_leg.py` at the new scale, copy its `params.json` values into that block and set `SCALE`.

## How it was checked

* **Evaluation:** OpenSCAD evaluated every part and the assembly with no warnings or errors.
* **All six parts:** each was rendered through the Manifold geometry library and compared with the STL parts from `integrate_leg.py`. Every part matched. Each comes out watertight with no bad edges. Each is a single solid apart from the shin, which keeps its two push rods.
* **Native OpenSCAD:** a pelvis half and the hip roll link were also rendered by OpenSCAD 2021.01 itself. Both matched the Python parts to within a few mm³, which is just how the cylinder facets line up.

## Things to know

* **Welded shells:** in the stock G1 meshes, the hip pitch link and thigh are made of shells that just touch each other. That leaves paper-thin sheets inside the part when you cut it. In `stock/` each extra shell has been grown by 0.1% about its own centre so the shells overlap and fuse into one solid. That moves those surfaces by up to 50 µm. These two parts differ from the Python and STEP versions by that much.
* **Faceted floor:** the stator floor comes from the motor shell STL. That makes it faceted. OpenSCAD can't read STEP files.
* **Meshes only:** OpenSCAD exports meshes (STL, 3MF and similar), not STEP. The STEP files remain the CAD version.
