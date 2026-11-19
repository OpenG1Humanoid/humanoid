# G1 left leg with integrated 6512 cycloidal actuators (1:1)

This takes the Unitree G1 left leg at 1:1 scale and builds the 6512 cycloidal actuator into three joints: hip pitch, hip roll and knee. At each joint the parent link becomes the motor housing. The gear rings sit flush inside it and the child link bolts straight onto the actuator output.

Hip yaw is still the stock G1 interface for now.

It's a first pass made by mesh booleans on the G1 STLs. Use it to check fit and layout before redoing the parts properly in CAD.

## Files

| File | What it is |
|---|---|
| `integrate_leg.py` | Generator script for every part. Rerun it to change scale or clearances. |
| `pelvis_left_half_integrated.stl` | Left half of the pelvis with the left hip pitch actuator built in. |
| `pelvis_right_half_integrated.stl` | Right half of the pelvis with the right hip pitch actuator built in. |
| `left_hip_pitch_link_integrated.stl` | Hip pitch link. Bolts to the hip pitch output and houses the hip roll actuator. |
| `left_hip_roll_link_integrated.stl` | Hip roll link. Bolts to the hip roll output. Yaw end is still stock. |
| `left_hip_yaw_link_knee_integrated.stl` | Thigh (hip yaw link). Houses the knee actuator. Yaw end is still stock. |
| `left_knee_link_integrated.stl` | Shin (knee link). Bolts to the knee output. Includes the two stock ankle push rods as separate bodies. |
| `left_leg_assembly_preview_1to1.stl` | Everything above plus the actuator housings, placed at the zero pose. One solid for viewing only. |
| `section_hip_pitch.png` | Section through the hip pitch axis showing the split pelvis. |
| `section_roll_and_knee.png` | Sections through the hip roll and knee axes. |

## Requirements

Python 3 plus four packages:

```
pip install trimesh manifold3d numpy rtree
```

## Usage

```
python integrate_leg.py <input folder> <output folder> [scale]
```

The input folder needs these files with exactly these names:

```
pelvis.STL
left_hip_pitch_link.STL
left_hip_roll_link.STL
left_hip_yaw_link.STL
left_knee_link.STL
Motor_Shell_-_Motor_Shell.stl
```

The other actuator STLs aren't read by the script. They get used as they are.

Scale defaults to 1.0, which is 1:1 with the G1 URDF. A full run takes a few minutes. It prints each part's volume and confirms it's watertight.

## The actuator

These numbers were measured from the actuator STLs.

| | |
|---|---|
| Outside diameter | 70 mm |
| Gear stack | 29 mm (front cap 5, ring gear 19, rear ring 5) |
| Motor cage | 32 mm including a 6 mm floor |
| Ratio | 15:1 (16 ring pins, 15 lobes) |
| Housing bolts | 12x M3 on 64 mm PCD |
| Output mount | 6x M3 on 36 mm PCD |
| Pin bolts | 6x on 34 mm PCD, heads recessed in the output face |
| Stator mount | 4x M3 on 25 mm PCD |
| Bearing seats | 58 mm bore over a 45 mm output |

## How each joint is built

Every joint follows the same pattern.

### Parent link (stator side)

The motor shell isn't a separate part any more. The parent link has a motor bay cut into it (radius 29.25 mm, just over the 29 mm of the original cage posts). The floor of the motor shell is copied straight into the link so the stator bolt pattern, centre bore and wire slots match your current build.

Above the motor bay is a counterbore for the three gear rings. The front cap, ring gear and rear ring are your existing parts, unchanged. They drop in flush with the link face and clamp down with 12 M3 bolts into heat-set inserts.

Every pocket has at least 3 mm of wall all round. Where the stock G1 shape was thinner than that, the script adds material. At 1:1 that's 7 cm³ on the pelvis, 5 cm³ on the hip pitch link and 9 cm³ on the thigh. Expect small bulges in a few places.

Behind the stator floor there's an access opening. That's how you reach the stator screws and get the phase wires out.

### Child link (output side)

The stock Unitree interface is removed. Anything within 2 mm of the joint face is cut back so the child only touches the output flange and can't rub on the stationary cap. A boss with 6 M3 clearance holes matches the output's 36 mm bolt circle. There are shallow reliefs over the pin bolt heads. Each bolt hole has an access bore for a hex key.

### Where each actuator sits

| Joint | Housing in | Output bolts to | Output faces | Stator access from |
|---|---|---|---|---|
| Hip pitch | Pelvis halves (one each side) | Hip pitch link | Outboard | The pelvis split face |
| Hip roll | Hip pitch link | Hip roll link | Forward | A short bore in the back of the hip pitch link |
| Knee | Thigh | Shin | Outboard | The inner face of the thigh, where the floor sits flush |
| Hip yaw | Not done yet | | | |

The roll output face sits 33.5 mm forward of the roll joint origin. The knee output face sits 15.4 mm outboard of the knee joint origin. Both come from where the stock G1 housings end. The joint frames are unchanged.

### The split pelvis

At 1:1 the two hip pitch actuators sit back to back. Their stator floors end up only 6.9 mm apart in the middle of the pelvis, which isn't enough room to reach the stator screws. So the pelvis is made in two halves split on the centre plane.

Each half has a 3.45 mm recess behind its stator floor, open on the split face. That's room for M3 cap screw heads, which are 3 mm tall. The two recesses together form a 6.9 mm slot that exits through the back of the pelvis for the phase wires.

The halves line up on two 4 mm dowel pins and clamp together with four M3x12 bolts. Each bolt goes in from the right half through an access bore and into a heat-set insert in the left half. The bolts and dowels all stay clear of both hip pockets, the cavity and the wiring slot.

The script splits the pelvis automatically whenever the stator floors are closer than 40 mm (`SPLIT_GAP`). Above about 1.2x scale it makes the pelvis as one piece with a rear hatch instead.

### Why the gear rings stay separate

The whole stationary housing can't be one piece because of assembly order. The 58 mm output bearings won't pass the ring pin tips (about 53 mm across). The motor won't pass the 52 mm bearing lip on the rear ring either. The stack has to split there anyway.

Keeping the ring gear separate also means the part most likely to wear is a small reprint instead of a whole link.

The output shaft stays its own part too. If it were merged into the child link there'd be no way to get the front cap on over it.

## Assembly order

Work from the pelvis down. Each joint has to be finished before the next actuator goes in.

1. Press M3 heat-set inserts into the 12 holes at the bottom of every counterbore and into the four joining holes on the left pelvis half.
2. Bolt each hip pitch stator to its pelvis half from the split face. Run the phase wires along the recess and out of the slot at the back.
3. Fit the dowels and bolt the two pelvis halves together.
4. Build both hip pitch actuators into their pockets.
5. Bolt the hip pitch link to the hip pitch output with a long 2.5 mm hex key down its access bores. Four of the six bores run through the empty roll pocket. That means this has to happen before the roll actuator goes in.
6. Fit the roll stator through the bore at the back of the hip pitch link, then build the roll actuator into its pocket.
7. Bolt the roll link to the roll output through the bores in the front of the roll link.
8. Attach the thigh to the roll link at the stock yaw joint.
9. Fit the knee stator from the inner side of the thigh, then build the knee actuator into its pocket.
10. Bolt the shin to the knee output through the bores in the shin plate.

For each actuator, build the internals in the same order as your standalone build. The link stands in for the motor shell. The ring clamp bolts need the 29 mm ring stack plus whatever your cap counterbore and insert length need.

## Parameters

All set at the top of `integrate_leg.py`. Values are in mm.

| Name | Default | What it does |
|---|---|---|
| `SCALE` | 1.0 | G1 scale factor, also the third command line argument |
| `RING_CLEAR` | 0.20 | Radial clearance for the gear rings in the counterbore |
| `BAY_R` | 29.25 | Motor bay radius |
| `WALL_MIN` | 3.0 | Minimum wall around every actuator pocket |
| `INSERT_D` | 4.0 | Heat-set insert hole diameter |
| `INSERT_DEPTH` | 8.0 | Heat-set insert hole depth |
| `CAVITY_R` | 30.0 | Radius of the access opening behind each stator floor |
| `HATCH_HALF_H` | 28.0 | Half height of the wiring slot or hatch at the back of the pelvis |
| `SPLIT_GAP` | 40.0 | Split the pelvis when the stator floors are closer than this |
| `JOIN_BOLTS` | 4 positions | Pelvis joining bolts, as (x, z) at 1:1 |
| `JOIN_DOWELS` | 2 positions | Pelvis dowels, as (x, z) at 1:1 |
| `BOLT_GRIP` | 6.0 | Right-half material under each joining bolt head |
| `DOWEL_HOLE` | 4.1 | Dowel hole diameter, for 4 mm pins |
| `DOWEL_DEPTH` | 6.0 | Dowel hole depth in each half |
| `LINK_GAP` | 2.0 | Gap between a child link and the stationary cap |
| `BOSS_R` | 22.4 | Child boss radius (the output flange is 22.5) |
| `BOSS_INTO` | 6.0 | How far the boss runs into the child's existing plate |
| `M3_CLEAR` | 3.4 | M3 clearance hole |
| `CBORE_D` | 6.5 | Counterbore and access bore diameter |
| `BOSS_T` | 4.0 | Boss material under the bolt heads |
| `MIN_PIECE` | 500 mm³ | Loose boolean debris smaller than this is dropped from finished parts |

The bolt patterns are clocked automatically. On the hip pitch link the script also picks the roll actuator's rotation so the pitch bolt bores miss every roll insert and the roll stator floor.

## Mesh cleanup

The stock G1 STLs aren't clean solids. The hip pitch link is two shells that touch plus some zero-volume scraps. The thigh has a thin shell with its faces inside out. The pelvis has a small crack. The script repairs and merges these before cutting anything. It also removes the zero-thickness sheets the booleans leave where two stock shells touched.

Every finished part comes out watertight with no non-manifold edges. Each is one body, apart from the shin with its two push rods.

## Frames and the URDF

At 1:1 the URDF joint origins are used as they are. At any other scale, multiply every joint origin by the scale factor and the frames still line up.

The actuator frame has z pointing out of the joint face with z = 0 on the output face.

Masses and inertias in the URDF will need redoing from CAD once the parts are shelled.

## Checks run

These were done by intersecting the meshes at 1:1:

* **Parent links:** both pelvis halves, the hip pitch link and the thigh have no overlap with the front cap, ring gear, rear ring, output front or output rear.
* **Motor spaces:** all clear.
* **Stator floors:** all fully present.
* **Insert holes:** all intact.
* **Wall thickness:** at least 3 mm around every pocket.
* **Pelvis joining:** the bolts and dowels, with 2 mm of margin, miss both pockets, the cavity and the wiring slot.
* **Pitch bolt bores:** none touch a roll insert or the roll stator floor.
* **Hip pitch link:** clears the pelvis and gear rings across the full URDF pitch range (-145° to 165°).
* **Roll link:** clears the hip pitch link and roll rings across the full URDF roll range (-30° to 170°).
* **Shin:** no contact with the thigh or knee rings at any angle tested between 16° and 144°. It touches at -5° and 0° (dead straight) and at 165°. The stock G1 thigh and shin touch by exactly the same amount at those angles. That comes from the original shapes, not the new parts.
* **Bosses:** each child boss only has face contact with its output flange.

## Known limitations

* **Solid meshes:** the G1 STLs are solid visual meshes so every part comes out solid. Shell them in CAD before printing.
* **Long hex-key bores:** the bores on the hip pitch link are 61 to 92 mm long because of the stock G1 shape. Expect them to shrink once the link is shelled. The roll link bores are 34 to 38 mm and the shin bores are 15 mm.
* **Removal order:** removing the hip pitch link means taking the roll actuator out first.
* **Thin insert walls:** the insert holes leave about 0.75 mm of wall to the motor bay. Go gently when heat-setting.
* **Split through the waist:** the pelvis split runs through the stock waist motor area at the top. That will need a rethink when the waist joint gets its own actuator.
* **Heat:** each motor was in an open cage and is now enclosed.
* **Torque:** at 15:1 the motor needs about 5.9 Nm to match the 88 Nm hip limits in the G1 URDF. The knee needs about 9.3 Nm to match its 139 Nm limit. Both are before losses. Check these against the motor datasheet.
* **Not done yet:** the hip yaw joint and the right leg aren't done. There are no covers for the access openings. There's no locating spigot between a child link and its output.
* **Wiring:** routes for the roll and knee motor wires beyond the access openings aren't designed yet.

## Next steps

* Do the hip yaw joint the same way. That finishes the hip.
* Mirror everything for the right leg. That needs the right-side G1 STLs.
* Move to CAD for the final parts. STEP files of the actuator would make that much cleaner than working from meshes.
