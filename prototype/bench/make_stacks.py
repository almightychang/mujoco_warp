"""Write #1798's resting stacks (3 plates, 4 mm, fixed slab) as MJCF files for mjwarp-testspeed.

stack_mesh_aligned.xml  mesh plates exactly aligned (drops pairs on main)
stack_box_aligned.xml   box plates exactly aligned (CCD box-box path)
stack_mesh_offset.xml   mesh plates offset by a few mm and degrees (settles on main)
"""

import numpy as np
from scenes import box_mesh

NPLATES = 3
PLATE = (0.105, 0.148, 0.002)
SLAB = (0.3, 0.3, 0.02)
OFFSET_XY = 0.003
OFFSET_YAW_DEG = 2.0
PLATE_MASS = 'mass="0.05"'


def geom(kind, mesh, half, extra=""):
  if kind == "mesh":
    return f'<geom type="mesh" mesh="{mesh}" {extra}/>'
  return f'<geom type="box" size="{half[0]} {half[1]} {half[2]}" {extra}/>'


def plate_body(kind, i, offset):
  z = 2 * SLAB[2] + (2 * i + 1) * PLATE[2]
  sign = 1 if i % 2 else -1
  x, yaw = sign * offset * OFFSET_XY, np.deg2rad(sign * offset * OFFSET_YAW_DEG)
  quat = f"{np.cos(yaw / 2):.9f} 0 0 {np.sin(yaw / 2):.9f}"
  return f'<body pos="{x:.6f} 0 {z:.6f}" quat="{quat}"><freejoint/>{geom(kind, "plate", PLATE, PLATE_MASS)}</body>'


def stack_xml(kind, offset, nplates=NPLATES):
  plates = "".join(plate_body(kind, i, offset) for i in range(nplates))
  return f"""<mujoco>
  <option timestep="0.002" solver="CG" cone="elliptic" impratio="10" integrator="implicitfast"
          iterations="100" ls_iterations="15" tolerance="1e-6"/>
  <asset>{box_mesh("plate", PLATE)}{box_mesh("slab", SLAB)}</asset>
  <worldbody>
    <body pos="0 0 {SLAB[2]}">{geom(kind, "slab", SLAB)}</body>
    {plates}
  </worldbody>
</mujoco>
"""


def main():
  for name, kind, offset in (("mesh_aligned", "mesh", 0), ("box_aligned", "box", 0), ("mesh_offset", "mesh", 1)):
    with open(f"stack_{name}.xml", "w") as f:
      f.write(stack_xml(kind, offset))


if __name__ == "__main__":
  main()
