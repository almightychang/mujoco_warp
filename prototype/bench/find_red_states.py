"""Find static two-geom states where MuJoCo Warp CCD drops a contact that MuJoCo C reports.

Each case draws top-geom positions near the symmetric pose, screens them in one batched collision
call, then confirms the dropped ones as a static XML scene (the form used in collision_gjk_test.py).
MuJoCo C skips pairs of geoms on the same body, so its reference runs on the free-joint scene.
Prints XML positions to paste into tests.

Usage: find_red_states.py [nworld] [nkeep]
"""

import sys

import mujoco
import numpy as np
from scenes import SCENES
from scenes import c_dist
from scenes import candidates
from scenes import free_xml
from scenes import static_xml
from scenes import warp_dist

from mujoco_warp import test_data
from mujoco_warp._src.collision_gjk_test import _geom_dist

# scene, log10 depth range, log10 xy offset range (None: centered)
CASES = {
  "shallow-box": ("box", (-6, -5), (-10, -7)),
  "shallow-mesh": ("mesh", (-6, -5), (-10, -7)),
  "deep-box": ("box", (-4, -3), (-10, -8)),
  "deep-mesh": ("mesh", (-4, -3), (-10, -6)),
  "centered-box": ("box", (-5, -3), None),
  "shallow-sheets": ("sheets", (-6, -5), (-10, -7)),
  "deep-sheets": ("sheets", (-4, -3), (-10, -6)),
}


def main():
  nworld = int(sys.argv[1]) if len(sys.argv) > 1 else 4096
  nkeep = int(sys.argv[2]) if len(sys.argv) > 2 else 3
  for case, (scene, depth_range, offset_range) in CASES.items():
    kind, bottom, top = SCENES[scene]
    xyz, depth = candidates(nworld, bottom, top, depth_range, offset_range, np.random.default_rng(0))
    mjm_free = mujoco.MjModel.from_xml_string(free_xml(kind, bottom, top))
    idx = np.flatnonzero(np.isinf(warp_dist(mjm_free, xyz)))
    print(f"CASE {case}: {len(idx)}/{nworld} dropped in batch", flush=True)
    kept = 0
    for i in idx:
      ref = c_dist(mjm_free, xyz[i : i + 1])[0]
      top_pos = " ".join(f"{v:.9g}" for v in xyz[i])
      _, _, m, d = test_data.fixture(xml=static_xml(kind, bottom, top, top_pos))
      dist, ncon, _, _ = _geom_dist(m, d, 0, 1)
      if np.isinf(ref) or dist < 0.0:
        continue
      print(f'  pos="{top_pos}" depth={depth[i]:.3e} warp dist={dist:.3e} ncon={ncon} c dist={ref:.9g}', flush=True)
      kept += 1
      if kept == nkeep:
        break


if __name__ == "__main__":
  main()
