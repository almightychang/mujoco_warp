"""Two-geom face-aligned scenes and per-world contact distance in MuJoCo Warp and MuJoCo C."""

import mujoco
import numpy as np
import warp as wp

import mujoco_warp as mjw

wp.config.quiet = True

BOTTOM_Z = 0.19972974
SCENES = {
  # #1532's test scene: 0.1 m cube on a 0.2 m cube
  "box": ("box", (0.2, 0.2, 0.2), (0.1, 0.1, 0.1)),
  # #1798's plate on its slab
  "mesh": ("mesh", (0.3, 0.3, 0.02), (0.105, 0.148, 0.002)),
  # two of #1798's plates
  "sheets": ("mesh", (0.105, 0.148, 0.002), (0.105, 0.148, 0.002)),
}


def box_mesh(name, half):
  corners = [(sx * half[0], sy * half[1], sz * half[2]) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
  return f'<mesh name="{name}" vertex="{" ".join(f"{v:.6f}" for c in corners for v in c)}"/>'


def geom(kind, name, half, pos):
  shape = f'type="mesh" mesh="{name}"' if kind == "mesh" else f'type="box" size="{half[0]} {half[1]} {half[2]}"'
  return f'<geom {shape} pos="{pos}"/>'


def static_xml(kind, bottom, top, top_pos):
  return f"""
<mujoco>
  <asset>{box_mesh("bottom", bottom)}{box_mesh("top", top)}</asset>
  <worldbody>
    {geom(kind, "bottom", bottom, f"0 0 {BOTTOM_Z}")}
    {geom(kind, "top", top, top_pos)}
  </worldbody>
</mujoco>"""


def free_xml(kind, bottom, top):
  return f"""
<mujoco>
  <asset>{box_mesh("bottom", bottom)}{box_mesh("top", top)}</asset>
  <worldbody>
    {geom(kind, "bottom", bottom, f"0 0 {BOTTOM_Z}")}
    <body><freejoint/>{geom(kind, "top", top, "0 0 0")}</body>
  </worldbody>
</mujoco>"""


def candidates(nworld, bottom, top, depth_range, offset_range, rng):
  """Top-geom positions with log-uniform depth and xy offset (offset_range None: centered)."""
  depth = 10 ** rng.uniform(*depth_range, nworld)
  radius = 10 ** rng.uniform(*offset_range, nworld) if offset_range else np.zeros(nworld)
  angle = rng.uniform(0, 2 * np.pi, nworld)
  xyz = np.stack([radius * np.cos(angle), radius * np.sin(angle), BOTTOM_Z + bottom[2] + top[2] - depth], axis=1)
  return xyz.astype(np.float32), depth


def warp_dist(mjm, xyz):
  """Deepest contact distance per world, inf where MuJoCo Warp reports no contact."""
  nworld = xyz.shape[0]
  qpos = np.zeros((nworld, 7), np.float32)
  qpos[:, :3] = xyz
  qpos[:, 3] = 1.0
  m = mjw.put_model(mjm)
  d = mjw.put_data(mjm, mujoco.MjData(mjm), nworld=nworld, naconmax=16 * nworld, njmax=64)
  d.qpos.assign(qpos)
  mjw.kinematics(m, d)
  mjw.collision(m, d)
  wp.synchronize()
  ncon = int(d.nacon.numpy()[0])
  dist = np.full(nworld, np.inf)
  np.minimum.at(dist, d.contact.worldid.numpy()[:ncon], d.contact.dist.numpy()[:ncon])
  return dist


def c_dist(mjm, xyz):
  """Deepest contact distance per world, inf where MuJoCo C reports no contact."""
  mjd = mujoco.MjData(mjm)
  dist = np.full(xyz.shape[0], np.inf)
  for i, p in enumerate(xyz):
    mjd.qpos[:3] = p
    mjd.qpos[3:] = (1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(mjm, mjd)
    dist[i] = min((mjd.contact[j].dist for j in range(mjd.ncon)), default=np.inf)
  return dist
