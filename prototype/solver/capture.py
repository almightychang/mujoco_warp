"""Roll out the 3-plate aligned stack in MuJoCo Warp and save high and low iteration step inputs.

Usage: capture.py <out.npz> [nstep] [high_threshold]
"""

import sys

import numpy as np
from common import stack_model
from common import state_of
from common import warp_data

import mujoco_warp as mjw


def main():
  out = sys.argv[1]
  nstep = int(sys.argv[2]) if len(sys.argv) > 2 else 2000
  high = int(sys.argv[3]) if len(sys.argv) > 3 else 20
  mjm = stack_model()
  m, d = warp_data(mjm)
  states, niters = [], []
  for i in range(nstep):
    state = state_of(d)
    mjw.step(m, d)
    niter = int(d.solver_niter.numpy()[0])
    if i >= 200:
      states.append(state)
      niters.append(niter)
  niters = np.array(niters)
  print(
    f"steps {len(niters)}  mean niter {niters.mean():.1f}  >= {high}: {(niters >= high).sum()}  <= 2: {(niters <= 2).sum()}"
  )
  pick = np.concatenate([np.flatnonzero(niters >= high)[:20], np.flatnonzero(niters <= 2)[:10]])
  np.savez(out, niter=niters[pick], **{k: np.stack([states[i][k] for i in pick]) for k in states[0]})


if __name__ == "__main__":
  main()
