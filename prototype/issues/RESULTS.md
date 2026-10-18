# Related reports, rerun on main (c384ac5) and the develop branch (4bf6c9f)

RTX 5080, Warp 1.15.0, MuJoCo 3.16.1. Our own scripts here; the reporters' scripts for #1739 and #1740 were run
unmodified from the issue text (not copied here), without `--fix`.

| report | what | main | develop | why |
| --- | --- | --- | --- | --- |
| #1798 | aligned plate stacks never settle | 18.75 mm drift, 25.9 CG iterations | 0.006 mm, 2.1 | R1 + R2 |
| #1782 comment (Codnay) | lying cylinder on box, 16.5 um: normal and depth | CPU: -190 mm with normal (1,0,0) (multiccd), normal (0.83,-0.56,0) (no multiccd) | -0.0165 mm, normal (0,0,-1), as MuJoCo C | R1 alone (2310f42 already fixes it): the sliver face's direction no longer comes from the origin's projection |
| #1782 body | cylinder-box contact 410 m away (3.14.0, CUDA) | not reproduced: contact 0.22 m from the box center | same | probably fixed earlier (#1613) |
| #1739 | sphere/capsule CCD normal near zero penetration | up to 158.8 deg off | identical to main | GJK inflation path, not EPA or multicontact |
| #1740 | GJK float32 barycentric coordinates | 1945 / 2048 contacts, end-cap depth error up to 2.86 mm | identical to main | GJK `_S2D`, untouched |
| #1510 (closed) | welded box pallet drift | 0.001 mm (MuJoCo C 0.001 mm) | 0.000 mm | already fixed on main |
| mujoco #2471 (C) | intermittent box-box contact with margin | no on/off switches in 3000 steps | same | not reproduced in Warp |
