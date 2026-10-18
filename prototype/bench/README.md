# RFC bench: CCD "origin on face"

Measurements for `../NOTES.md`. `./run.sh <label>` collects everything for the worktree's current code into `results/<label>/`, so a prototype is compared with `results/main/`.

| file | what |
| --- | --- |
| `tests_red.txt` | `test_{box_box,mesh_mesh}_aligned_faces`, `test_mesh_mesh_aligned_plates` and `test_mesh_mesh_settling_plate` (#1798 steps 1591/1592) in `collision_gjk_test.py`. On main the seven dropped cases fail; the two separated cases and step 1591 pass |
| `tests_collision.txt` | `collision_gjk_test.py` and `collision_driver_test.py`, for regressions |
| `drop_rate.txt` | scene × xy-offset band × depth decade: dropped contacts, max depth error, and overestimated contacts (Warp deeper than C by > 1 µm) |
| `stack_*.txt` | #1798's 3-plate stacks, nworld 4096, 1000 steps: steps/s, event trace, CG iterations |
| `aloha.txt` | `aloha_pot`, `aloha_clutter` with the flags from `benchmarks/` |
| `ccd_resources.txt` | registers, stack frame and spills of the compiled CCD kernels (box-box and mesh-mesh) |

Other scripts:

- `scenes.py`: the two-geom scenes, plus per-world contact distance in Warp (batched) and C
- `find_red_states.py`: searches for states that drop on main; the red tests use states from `find_red_states.txt`
- `label_exits.py`: prints which CCD exit each of those states takes; run it with `PYTHONPATH` pointing at a package copy patched with `wp.printf` at each CCD exit (the patcher lived in the local issue notes, `issues/D-work/instrument.py`)
- `make_stacks.py`: writes the stack MJCF files
- `aloha.py`: reuses `benchmarks/run.py` asset assembly and flags, but runs this venv's testspeed

The stack worlds start identical and diverge only through contact ordering, so warps are close to fully coherent. For a new branch, that is the worst case.
