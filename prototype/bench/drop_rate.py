"""Drop rate and depth error of face-aligned contacts, MuJoCo Warp against MuJoCo C.

Per cell (scene x xy-offset band x depth decade): the share of worlds where C reports a contact and
Warp does not, the largest |depth error| where both report one, and the share where Warp is deeper
than C by more than 1 um (an overestimated contact).

Usage: drop_rate.py [nworld]
"""

import sys

import mujoco
import numpy as np
from scenes import SCENES
from scenes import c_dist
from scenes import candidates
from scenes import free_xml
from scenes import warp_dist

# log10 xy offset from the symmetric pose
BANDS = {"near": (-10, -6), "misaligned": (-4, -2)}
DEPTH_DECADES = range(-6, -2)
OVERESTIMATE = 1e-6


def main():
  nworld = int(sys.argv[1]) if len(sys.argv) > 1 else 4096
  print(f"{'scene':6s} {'band':10s} {'depth':>9s} {'dropped':>8s} {'max err':>9s} {'deeper':>7s}")
  for scene, (kind, bottom, top) in SCENES.items():
    mjm = mujoco.MjModel.from_xml_string(free_xml(kind, bottom, top))
    for band, offset_range in BANDS.items():
      for decade in DEPTH_DECADES:
        xyz, _ = candidates(nworld, bottom, top, (decade, decade + 1), offset_range, np.random.default_rng(0))
        warp, c = warp_dist(mjm, xyz), c_dist(mjm, xyz)
        both = np.isfinite(warp) & np.isfinite(c)
        dropped = np.isfinite(c) & np.isinf(warp)
        err = np.abs(warp[both] - c[both]).max(initial=0.0)
        deeper = warp[both] < c[both] - OVERESTIMATE
        print(
          f"{scene:6s} {band:10s} 1e{decade:+d} m {dropped.mean() * 100:7.2f}% {err:9.2e} {deeper.sum() / nworld * 100:6.2f}%",
          flush=True,
        )


if __name__ == "__main__":
  main()
