# Open G1 – a 3D-printable humanoid

Open G1 is a printable humanoid built around the **Unitree G1's shape and kinematics**.
The legs use 3D-printed cycloidal actuators in the style of the Berkeley Humanoid. The torso and
arms use **off-the-shelf hobby servos** hidden inside the original G1 link shapes.

Because the joint axes and link frames are identical to the G1 URDF, policies and tools written
for the G1 (pose copying, teleoperation, RL) carry over.

- **Simulation:** [open3dhumanoid/humanoid-gym](https://github.com/open3dhumanoid/humanoid-gym) (MuJoCo + Gymnasium)
- **Design files:** OpenSCAD, generated from a single Python description of every joint feature

<!-- add a photo / render here: previews/torso_arms_original_vs_new.png -->

## What's in this repo

```
g1_servo_arm.scad        # every printable part (set side + part)
assembly.scad            # the whole torso + both arms, posable (qL / qR joint angles)
parts_left/ parts_right/ # one file per printable arm part per side: open, F6, export STL
parts_torso/             # torso + its two shoulder hatches
hardware/                # printed hubs and idler pins
stl/                     # source meshes: the G1 link STLs, the torso split into bodies,
                         #   cleaned rubber-hand meshes, voxel fallbacks
generator/               # design.py (all joint features) + emit.py (writes the .scad files)
previews/                # renders: originals vs servo version
```

## Design approach

1. **Start from the G1 meshes.** Every link is the original G1 STL, imported unchanged. The outside
   shape is not altered.
2. **Cut, don't add.** Each joint gets a servo pocket and a **hatch** (the plug of original material
   removed to fit the servo, which screws back on). It also gets a hex socket for the output hub.
   Material is only added inside the thin wrist covers, clipped to the cover's own outline.
3. **Verify.** Every servo is checked to sit 100% inside its link. Every part is checked to come out
   as a single piece, and every joint is swept through its full G1 range for collisions. These
   checks run on a voxel model of the same geometry.

## Actuators

| Joint | Actuator | Stall torque | Fits inside |
|---|---|---|---|
| Hips, knees, ankles | Printed cycloidal actuators (Berkeley-Humanoid style) | – | leg links |
| Shoulder pitch | DS3235 (40×20×38.5 mm) | 35 kg·cm @ 7.4 V | torso shoulder housing |
| Shoulder roll | DS3235 | 35 kg·cm | shoulder pitch link |
| Shoulder yaw | KST X10 Pro (30×10×33.5 mm) | 11.5 kg·cm | shoulder roll link |
| Elbow | KST X10 Pro | 11.5 kg·cm | shoulder yaw link |
| Wrist roll | TowerPro MG92B (22.8×12×31 mm) | 3.5 kg·cm @ 6 V | elbow link |
| Wrist pitch + yaw | 2× MG92B | 3.5 kg·cm | wrist pitch link |

Holding a printed arm straight out needs roughly:
- **Shoulder:** about 1.3 N·m, rising to about 2.6 N·m with 300 g in the hand.
- **Elbow:** about 0.5 N·m.
- **Wrist:** about 0.15 N·m.

The shoulder pitch housing also fits any 40×20 mm servo up to 48 mm tall, if you want more torque.

## Bill of materials (torso + both arms)

| Item | Qty |
|---|---|
| DS3235 servo (270° version recommended) | 4 |
| KST X10 Pro servo | 4 |
| TowerPro MG92B servo | 6 |
| Round metal servo horns to suit | 14 |
| M3 and M2 self-tapping screws (hatches, hubs, radial retaining screws) | ~150 |
| 5 mm idler pins (printed) | 4 |
| Filament: PLA or PETG (PETG recommended for hubs) | ~2.5 kg |

## Printing and assembly

1. **Export the parts.** Open each file in `parts_left/`, `parts_right/`, `parts_torso/` and
   `hardware/`, then press F6 and export the STL. Use OpenSCAD 2024+ with the Manifold backend
   (Preferences → Features → manifold).
2. **Check your servos.** Measure the shaft offset, horn diameter and horn height on the servos you
   bought and compare them with `SERVO` in `generator/design.py`. If they differ, run
   `python generator/emit.py` to regenerate the files.
3. **Print.** Print hatches and caps in the same material as their link, so the seam is invisible.
   Print hubs in PETG, hex end down.
4. **Assemble each joint:**
   1. Drop the servo through the joint face.
   2. Screw the hub to the servo horn from the horn side.
   3. Fit the horn and hub onto the spline at the servo's centre angle, and fit the centre screw.
   4. Screw the hatch back on.
   5. Slide the child link onto the hex and fit the radial screws.
   6. **Wrist covers only:** fit the pitch link into the cover first. Then put the hub in through
      the cover from outside, fit the idler pin on the opposite side, and glue the cosmetic caps over both.
5. **Pose the assembly.** `assembly.scad` shows the finished robot. Set `qL` / `qR` to pose the arms.

## Simulation

[humanoid-gym](https://github.com/open3dhumanoid/humanoid-gym) contains `open_g1.xml`. It is the
G1 29-DoF MJCF with:
- the printed torso and the 7-DoF servo arms,
- link masses and inertias computed from these CAD files,
- servo torque and travel limits,
- the rubber hand.

It also has a Gymnasium environment (`OpenG1-Stand-v0`).

## Status

- [x] Torso + 7-DoF arms (servo joints): design files and voxel verification
- [x] MuJoCo model + Gymnasium environment
- [ ] OpenSCAD render check of every part (please report any errors)
- [ ] Leg actuator integration (Berkeley-style printed actuators) + measured leg parameters in the sim
- [ ] Waist actuators (the G1 waist linkage parts are included, unmodified)
- [ ] Cable routing between links

## Credits

- **Unitree Robotics:** G1 meshes and URDF (`unitree_ros/robots/g1_description`). Check Unitree's
  licence for redistribution of the meshes.
- **Berkeley Humanoid:** printed actuator design used for the legs.
