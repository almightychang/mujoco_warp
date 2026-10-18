"""Which solver-side factor keeps aligned stacks from settling once no contact is dropped.

For #1798's aligned mesh stacks, each variant changes one option and reports, over the second half
of the rollout: mean / max CG iterations, steps at the iteration cap, max plate z drift, the share
of steps whose contact set differs from the previous step, and mean iterations on steps with and
without such a change. Contact sets compare geom pairs and positions rounded to 0.1 mm.

Usage: solver_exp.py [nstep]
"""

import sys

import mujoco
import numpy as np
from make_stacks import stack_xml

import mujoco_warp as mjw

VARIANTS = {
  "base": {},
  "impratio=1": {"impratio": 1.0},
  "tol=1e-5": {"tolerance": 1e-5},
  "newton": {"solver": mujoco.mjtSolver.mjSOL_NEWTON},
  "pyramidal": {"cone": mujoco.mjtCone.mjCONE_PYRAMIDAL},
}
PLATES = (3, 4, 5)


def contact_set(geom, pos):
  return frozenset((int(g[0]), int(g[1]), round(float(p[0]), 4), round(float(p[1]), 4)) for g, p in zip(geom, pos))


def rollout_warp(mjm, nstep):
  m = mjw.put_model(mjm)
  d = mjw.put_data(mjm, mujoco.MjData(mjm), nconmax=256, njmax=1024)
  for _ in range(nstep):
    mjw.step(m, d)
    n = int(d.nacon.numpy()[0])
    yield (
      int(d.solver_niter.numpy()[0]),
      contact_set(d.contact.geom.numpy()[:n], d.contact.pos.numpy()[:n]),
      d.qpos.numpy()[0, 2::7],
    )


def rollout_c(mjm, nstep):
  mjd = mujoco.MjData(mjm)
  for _ in range(nstep):
    mujoco.mj_step(mjm, mjd)
    c = mjd.contact[: mjd.ncon]
    yield int(mjd.solver_niter[0]), contact_set(c.geom, c.pos), mjd.qpos[2::7].copy()


def summarize(rollout, nstep, cap):
  niter, changed, z = [], [], []
  prev = None
  for i, (n, contacts, zi) in enumerate(rollout):
    if i >= nstep // 2:
      niter.append(n)
      changed.append(contacts != prev)
      z.append(zi)
    prev = contacts
  niter, changed, z = np.array(niter), np.array(changed), np.array(z)
  drift = np.abs(z - z[0]).max()
  with_change = niter[changed].mean() if changed.any() else float("nan")
  without_change = niter[~changed].mean() if (~changed).any() else float("nan")
  return (
    f"niter {niter.mean():5.1f} max {niter.max():3d} capped {int((niter >= cap).sum()):3d}  drift {drift * 1e3:7.3f} mm  "
    f"set changed {changed.mean() * 100:5.1f}%  niter changed/unchanged {with_change:5.1f} / {without_change:5.1f}"
  )


def main():
  nstep = int(sys.argv[1]) if len(sys.argv) > 1 else 2000
  for nplates in PLATES:
    for name, options in VARIANTS.items():
      mjm = mujoco.MjModel.from_xml_string(stack_xml("mesh", 0, nplates))
      for key, value in options.items():
        setattr(mjm.opt, key, value)
      cap = mjm.opt.iterations
      for engine, rollout in (("warp", rollout_warp), ("c", rollout_c)):
        print(f"{nplates} plates {name:11s} {engine:4s} {summarize(rollout(mjm, nstep), nstep, cap)}", flush=True)


if __name__ == "__main__":
  main()
