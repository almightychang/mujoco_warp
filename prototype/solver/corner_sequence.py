"""Roll out the 3-plate aligned stack and mark, per step, whether every pair got its four corners.

Prints the marks for steps 200-399 ('.' correct, 'W' wrong), iterations for steps 200-259, and mean
iterations on wrong steps, on the step after a wrong one, and on the remaining steps.

Usage: corner_sequence.py [nstep]
"""

import sys

import numpy as np
from common import stack_model
from common import warp_data
from wrong_corners import contact_points
from wrong_corners import wrong_pairs

import mujoco_warp as mjw


def main():
  nstep = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
  m, d = warp_data(stack_model())
  wrong, niter = [], []
  for _ in range(nstep):
    mjw.step(m, d)
    _, pairs = contact_points(d)
    wrong.append(len(pairs) != 3 or bool(wrong_pairs(pairs)))
    niter.append(int(d.solver_niter.numpy()[0]))
  wrong, niter = np.array(wrong[200:]), np.array(niter[200:])
  print("".join("W" if w else "." for w in wrong[:200]))
  print(" ".join(str(n) for n in niter[:60]))
  after = np.concatenate([[False], wrong[:-1]])
  rest = ~wrong & ~after
  mean = lambda mask: niter[mask].mean() if mask.any() else float("nan")
  print(
    f"wrong-corner steps {wrong.mean() * 100:.1f}%  niter on wrong {mean(wrong):.1f}  "
    f"on the step after {mean(after & ~wrong):.1f}  elsewhere {mean(rest):.1f}"
  )


if __name__ == "__main__":
  main()
