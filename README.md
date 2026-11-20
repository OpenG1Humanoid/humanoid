# OpenG1: a 3D-printable Unitree G1-style humanoid

A cheaper, 3D-printable humanoid that keeps the look and joint layout of the Unitree G1. Everything here is printable STL files plus OpenSCAD assemblies for previewing the whole robot.

![Both arms](STL/arms/renders/01_both_arms.png)

- **Legs and waist:** 3D-printed cycloidal actuators of the Berkeley 6512 style.
- **Arms:** cheap hobby servos in swappable cartridges.
- **Torso:** a 2020 aluminium-profile frame with printed panels, sized for a 220 × 220 × 250 mm print bed, with room inside for the compute board, motor and servo drivers, and the head cable.
- **Head:** a dome built around a camera, with printed panels.

Related: [humanoid-gym](https://github.com/open3dhumanoid/humanoid-gym), a Gymnasium simulator for this robot.

## Repository layout

```
STL/
├── robot_assembly.scad          Whole robot in OpenSCAD, with a slider for every joint
├── arms/                        Servo arms (left + right): see STL/arms/README.md
│   ├── arm_assembly.scad          Arms only, with joint sliders
│   ├── left/  right/
│   │   ├── links/                 7 edited G1 arm links per side
│   │   └── actuators/<joint>/     servo cartridges, caps, horn plates, idler pins
│   ├── fit_tests/                 print these first
│   ├── renders/                   pictures
│   └── generator/                 Python that generates the arm parts
├── legs/                        Pelvis halves, hip/knee/ankle links with the actuator built in, waist yaw link
│   ├── actuator/                  6512-style cycloidal actuator (STEP + STLs)
│   ├── feet/                      stock G1 ankle links (in metres)
│   └── stock/                     unmodified reference parts
├── torso_frame/                 Redesigned torso: front/back/top panels, hatches, trays, shoulder blocks
├── torso/                       Older stock torso pieces (reference only)
├── torso_waist_parts_unmodified/  Stock G1 waist linkage (reference)
└── head/                        Head frame, panels and brows
```

## Viewing the whole robot

1. Open `STL/robot_assembly.scad` in [OpenSCAD](https://openscad.org) and press **F5**. It must stay in `STL/`, because it loads the parts by relative path.
2. Use the Customizer (Window → Customizer) to:
   - move every joint: arms, legs and waist, within the G1 limits
   - show or hide each arm, the legs, the torso and the head
   - make the arm links see-through, to see the servos inside
   - pick the knee variant (`ankle` or `integrated`) and the pelvis variant (`waist` or `integrated`)

Joint positions follow Unitree's G1 model (`g1_29dof_with_hand_rev_1_0.urdf`). The pelvis frame is the root, and the robot stands on z = 0 at the zero pose.

## Units

- **Millimetres:** all printable parts, which load into a slicer at the right size.
- **Metres:** a few stock reference meshes (`legs/feet/*.STL`, `torso/torso_body*.stl`, `torso_waist_parts_unmodified/`), the same as the original G1 files. Scale them ×1000 if you print them. The assembly does this for the feet automatically.

## Arms

Each arm has seven joints, laid out as on the G1:

| Joint | Servo | Bearing |
|---|---|---|
| Shoulder pitch | Standard 40×20 (in the torso shoulder block) | |
| Shoulder roll | Standard 40×20, 35 kg·cm | 6803 idler |
| Shoulder yaw | Micro 23×12, high-torque | 6805 |
| Elbow | Standard 40×20 | |
| Wrist roll | Micro 23×12 (MG90S/MG92B) | 6805 |
| Wrist pitch | Micro 23×12 | 6803 idler |
| Wrist yaw | Micro 23×12 | 6803 idler |

- **Cartridges:** each servo sits in a cartridge that slides into the link and screws in with M3 heat-set inserts, so servos can be swapped later.
- **Horns:** the plastic cross horn that comes with the servo screws into a horn-shaped pocket.
- **Outer shape:** the outside of every link is the original G1 shape.

The parts list, print settings, assembly steps for each joint and known limitations are in **[`STL/arms/README.md`](STL/arms/README.md)**. The main limitation is that shoulder yaw is the weakest joint.

![Cutaway](STL/arms/renders/02_left_cutaway.png)

## Legs

The hip, knee and ankle links are G1 links edited to hold a 6512-style 3D-printed cycloidal actuator (see `legs/actuator/`). The right leg is a mirror of the left. Some parts come in two versions:

- `*_knee_link_ankle.stl` / `*_knee_link_integrated.stl`
- `pelvis_*_waist.stl` / `pelvis_*_integrated.stl`

`ankle_crank.stl` is not placed in the assembly yet.

## Torso and head

- **Torso:** `torso_frame/` is the current torso, panels on a 2020 aluminium extrusion frame. The shoulder blocks hold the shoulder-pitch servos.
- **Head:** `head/` holds the frame, panels and brows of the head.

## Printing notes

- 0.4 mm nozzle, at least 4 walls (1.6 mm).
- M3 heat-set inserts use Ø4.0 mm holes.
- Print `STL/arms/fit_tests/` before the big arm parts to check the bearing seats, insert holes and servo pockets on your printer.

## Status

Work in progress. Nothing in the arm set has been printed yet, so check the fits before committing to the large prints. Issues and pull requests are welcome.

## Credits

- Based on the geometry and kinematics of the Unitree G1 (`unitree_ros`).
- Leg actuator design inspired by the Berkeley Humanoid Lite printed actuators.
