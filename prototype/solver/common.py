"""Aligned-stack solver experiments: the scene, state capture, and a MuJoCo C cost oracle."""

import copy
import os
import sys

import mujoco
import numpy as np
import warp as wp

import mujoco_warp as mjw

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "bench"))
from make_stacks import stack_xml  # noqa: E402

wp.config.quiet = True


def stack_model(nplates=3, **options):
  mjm = mujoco.MjModel.from_xml_string(stack_xml("mesh", 0, nplates))
  for key, value in options.items():
    setattr(mjm.opt, key, value)
  return mjm


def warp_data(mjm, mjd=None):
  m = mjw.put_model(mjm)
  d = mjw.put_data(mjm, mujoco.MjData(mjm) if mjd is None else mjd, nconmax=256, njmax=1024)
  return m, d


def state_of(d):
  """The inputs of one step: what forward() reads, including the solver warmstart."""
  return {k: getattr(d, k).numpy()[0].copy() for k in ("qpos", "qvel", "qacc_warmstart")}


def c_data(mjm, state):
  mjd = mujoco.MjData(mjm)
  for k, v in state.items():
    getattr(mjd, k)[:] = v
  return mjd


def warp_forward(mjm, state):
  """MuJoCo Warp forward() on a state; returns (niter, qacc)."""
  m, d = warp_data(mjm, c_data(mjm, state))
  for k, v in state.items():
    getattr(d, k).assign(v[None].astype(np.float32))
  mjw.forward(m, d)
  return int(d.solver_niter.numpy()[0]), d.qacc.numpy()[0].astype(np.float64)


def c_forward(mjm, state):
  mjd = c_data(mjm, state)
  mujoco.mj_forward(mjm, mjd)
  return int(mjd.solver_niter[0]), mjd.qacc.copy(), mjd


class Oracle:
  """MuJoCo C's solver objective for one state, evaluated at any qacc (C's constraint set)."""

  def __init__(self, mjm, state):  # noqa: D107
    self.mjm = mjm
    _, _, self.mjd = c_forward(mjm, state)
    self.M = np.zeros((mjm.nv, mjm.nv))
    mujoco.mj_fullM(mjm, self.mjd, self.M)
    self.J = self.mjd.efc_J.reshape(self.mjd.nefc, mjm.nv) if self.mjd.nefc else np.zeros((0, mjm.nv))
    self.scale = 1.0 / (mjm.stat.meaninertia * max(1, mjm.nv))

  def __call__(self, qacc):
    """Returns (cost, rescaled gradient norm) at qacc."""
    mjd = copy.copy(self.mjd)
    jar = self.J @ qacc - mjd.efc_aref[: mjd.nefc]
    cost = np.zeros(1)
    mujoco.mj_constraintUpdate(self.mjm, mjd, jar, cost, 0)
    da = qacc - mjd.qacc_smooth
    gauss = 0.5 * da @ self.M @ da
    grad = self.M @ qacc - mjd.qfrc_smooth - mjd.qfrc_constraint
    return gauss + cost[0], self.scale * np.linalg.norm(grad)
