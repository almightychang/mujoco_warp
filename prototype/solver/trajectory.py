"""Experiment B: per-iteration progress of MuJoCo Warp and MuJoCo C from the same step input.

Each engine is stopped after exactly k iterations (tolerance 0) and C's objective is evaluated at
its qacc. Prints the cost gap to C's converged optimum and the rescaled gradient norm per k.

Usage: trajectory.py <states.npz> <index> [kmax]
"""

import sys

import numpy as np
from common import Oracle
from common import c_forward
from common import stack_model
from common import warp_forward


def main():
  data = np.load(sys.argv[1])
  i = int(sys.argv[2])
  kmax = int(sys.argv[3]) if len(sys.argv) > 3 else 40
  state = {k: data[k][i] for k in ("qpos", "qvel", "qacc_warmstart")}
  oracle = Oracle(stack_model(), state)
  _, qacc_opt, _ = c_forward(stack_model(iterations=1000, tolerance=1e-14), state)
  cost_opt, _ = oracle(qacc_opt)
  print(f"state {i}: rollout niter {data['niter'][i]}")
  print(f"{'k':>3s} {'warp cost gap':>13s} {'warp grad':>9s} {'c cost gap':>11s} {'c grad':>9s}")
  for k in range(1, kmax + 1):
    _, qacc_w = warp_forward(stack_model(iterations=k, tolerance=0.0), state)
    _, qacc_c, _ = c_forward(stack_model(iterations=k, tolerance=0.0), state)
    (cw, gw), (cc, gc) = oracle(qacc_w), oracle(qacc_c)
    print(f"{k:3d} {cw - cost_opt:13.3e} {gw:9.2e} {cc - cost_opt:11.3e} {gc:9.2e}", flush=True)


if __name__ == "__main__":
  main()
