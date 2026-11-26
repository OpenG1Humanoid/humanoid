# arm_fixes

These scripts recut the arm horn pockets and the servo cable channels, and reinforce the shoulder roll link. They work straight on the arm STLs, because the original arm generator isn't in this repo.

Each part goes through the same steps:
1. Voxelise it into a signed-distance grid, 0.08–0.15 mm per voxel (`geom.cpp`).
2. Fill the old pocket and cut the new shapes.
3. Run marching cubes, then QEM decimation back to a normal triangle count.
4. Check that the result is watertight and a single shell.

Away from the edits, the surface moves by less than about 0.15 mm.

- `jobs.py`: what gets cut where. `HORN` and `WIRE` at the top hold the sizes.
- `meshedit.py`: the SDF grid, primitives (box, cylinder, polygon, horn shape) and mesh checks.
- `reinforce.py`: thickens the inside of the shoulder roll link's cup, keeping clear of the pitch side swept over the roll range. Run it after `jobs.py`.
- `reinforce2.py wrist_roll wrist_yaw`: the same inside thickening for the wrist yokes.
- `newyaw.py horn|cart|yaw|roll <side>`: the shoulder yaw moved into the upper arm. Upper arm: old socket filled, cartridge pocket, floor inserts, wire groove, plug channel. Cartridge: full-height lugs with M3 through-holes. Horn plate: 2 inserts in the key. Roll link (from the original): empty ring and old pocket filled solid, hex socket, 2 M3 holes from the yoke floor. Uses `lugfix.py` (2D outlines), `thicken.py` (smooth inward wall thickening) and the pitch-side sweep from `sweep2.py` (`sw2_<side>_*.npz`).
- `check3.py <side>` / `asm3.py`: the new parts assembled; overlaps at several yaw angles, insertion paths, elbow parts.
- `joints.py`, `asm.py`: read `arm_assembly.scad` to get each servo cartridge's spline axis and its cap/link transforms.

The run steps are in `STL/arms/README.md` under "Changing the horn pockets or cable channels". Always run the scripts on the original (pre-fix) STLs. They fill the old pocket based on its measured size.
