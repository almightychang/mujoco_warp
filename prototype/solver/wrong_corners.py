"""For each captured state, list the plate pairs whose contact set is not the four plate corners.

A face-face pair of #1798's plates should get exactly the four corners (±105, ±148 mm) from
multicontact. Run it with PYTHONPATH pointing at the code to check (main, a prototype).

Usage: wrong_corners.py <states.npz>
"""

import sys

import numpy as np
from common import c_data
from common import stack_model
from common import warp_data

import mujoco_warp as mjw

CORNERS = {(sx * 105.0, sy * 148.0) for sx in (-1, 1) for sy in (-1, 1)}


def contact_points(d):
  n = int(d.nacon.numpy()[0])
  pairs = {}
  for geom, pos in zip(map(tuple, d.contact.geom.numpy()[:n]), d.contact.pos.numpy()[:n]):
    pairs.setdefault(geom, []).append((round(float(pos[0]) * 1e3, 1), round(float(pos[1]) * 1e3, 1)))
  return n, pairs


def wrong_pairs(pairs):
  return sorted(g for g, points in pairs.items() if len(points) != 4 or set(points) != CORNERS)


def main():
  data = np.load(sys.argv[1])
  mjm = stack_model()
  bad = 0
  for i in range(len(data["niter"])):
    state = {k: data[k][i] for k in ("qpos", "qvel", "qacc_warmstart")}
    m, d = warp_data(mjm, c_data(mjm, state))
    for k, v in state.items():
      getattr(d, k).assign(v[None].astype(np.float32))
    mjw.forward(m, d)
    n, pairs = contact_points(d)
    wrong = wrong_pairs(pairs)
    bad += bool(wrong)
    print(f"state {i:2d} rollout niter {data['niter'][i]:3d} ncon {n:2d} wrong pairs: {wrong or '-'}")
  print(f"states with a wrong corner set: {bad} / {len(data['niter'])}")


if __name__ == "__main__":
  main()
