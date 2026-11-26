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
- `joints.py`, `asm.py`: read `arm_assembly.scad` to get each servo cartridge's spline axis and its cap/link transforms.

The run steps are in `STL/arms/README.md` under "Changing the horn pockets or cable channels". Always run the scripts on the original (pre-fix) STLs. They fill the old pocket based on its measured size.
