"""Experiment A: the same step inputs solved by MuJoCo Warp and MuJoCo C.

For each captured state: iterations in the rollout, Warp forward() again (twice), C mj_forward,
and both engines with a zero warmstart. Also C's objective at both solutions.

Usage: same_input.py <states.npz>
"""

import sys

import numpy as np
from common import Oracle
from common import c_forward
from common import stack_model
from common import warp_forward


def main():
  data = np.load(sys.argv[1])
  mjm = stack_model()
  print(
    f"{'rollout':>7s} {'warp':>9s} {'c':>4s} {'warp ws=0':>9s} {'c ws=0':>6s}  {'cost warp - c':>13s}  {'grad warp':>9s} {'grad c':>8s}"
  )
  for i in range(len(data["niter"])):
    state = {k: data[k][i] for k in ("qpos", "qvel", "qacc_warmstart")}
    cold = dict(state, qacc_warmstart=np.zeros_like(state["qacc_warmstart"]))
    w1, qacc_w = warp_forward(mjm, state)
    w2, _ = warp_forward(mjm, state)
    c, qacc_c, _ = c_forward(mjm, state)
    w0, _ = warp_forward(mjm, cold)
    c0, _, _ = c_forward(mjm, cold)
    oracle = Oracle(mjm, state)
    (cost_w, grad_w), (cost_c, grad_c) = oracle(qacc_w), oracle(qacc_c)
    print(f"{data['niter'][i]:7d} {w1:4d},{w2:4d} {c:4d} {w0:9d} {c0:6d}  {cost_w - cost_c:13.3e}  {grad_w:9.2e} {grad_c:8.2e}")


if __name__ == "__main__":
  main()
