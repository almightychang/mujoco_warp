"""#1510: welded box pallet drift; mujoco #2471: intermittent box-box contact with margin."""

import sys

import mujoco
import numpy as np
import warp as wp

import mujoco_warp as mjw

wp.config.quiet = True
XML_1510 = open(sys.argv[1]).read()
XML_2471 = """<mujoco><compiler angle="radian"/><worldbody>
  <body name="1" quat="1 0 1e-6 0"><geom type="box" size="0.007 0.007 0.01" pos="0 0 0.05" condim="1"/></body>
  <body name="2" pos="0 0.02 0"><joint name="2" axis="1 0 0"/><geom type="box" size="0.007 0.007 0.01" pos="0 0 0.06" condim="1" margin="0.0005"/></body>
</worldbody><actuator><motor name="2" joint="2" ctrlrange="0 1e-7"/></actuator></mujoco>"""


def rollout(mjm, nstep, engine, ctrl=None):
  if engine == "c":
    mjd = mujoco.MjData(mjm)
    if ctrl is not None:
      mjd.ctrl[:] = ctrl
    for _ in range(nstep):
      mujoco.mj_step(mjm, mjd)
      yield mjd.xpos.copy(), mjd.xmat.reshape(-1, 3, 3).copy(), mjd.ncon
  else:
    mjd = mujoco.MjData(mjm)
    if ctrl is not None:
      mjd.ctrl[:] = ctrl
    m = mjw.put_model(mjm)
    d = mjw.put_data(mjm, mjd, nconmax=512, njmax=2048)
    for _ in range(nstep):
      mjw.step(m, d)
      yield d.xpos.numpy()[0], d.xmat.numpy()[0], int(d.nacon.numpy()[0])


def pallet_drift(engine):
  mjm = mujoco.MjModel.from_xml_string(XML_1510)
  mjm.opt.integrator = mujoco.mjtIntegrator.mjINT_EULER
  base = mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_BODY, "pallet_base")
  items = [mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_BODY, f"item_{i}") for i in range(12)]
  rel0, drift = None, 0.0
  for i, (xpos, xmat, _) in enumerate(rollout(mjm, 2000, engine)):
    if i < 500:
      continue
    rel = (xpos[items] - xpos[base]) @ xmat[base]
    rel0 = rel if rel0 is None else rel0
    drift = max(drift, float(np.linalg.norm(rel - rel0, axis=1).max()))
  return drift


def toggles(engine):
  mjm = mujoco.MjModel.from_xml_string(XML_2471)
  ncon = np.array([n for _, _, n in rollout(mjm, 3000, engine, ctrl=1e-7)])
  touching = ncon > 0
  first = int(np.argmax(touching)) if touching.any() else len(ncon)
  after = touching[first:]
  return first, int(after.size - after.sum()), int((np.diff(after.astype(int)) != 0).sum())


for engine in ("c", "warp"):
  print(f"#1510 {engine}: max item drift relative to the pallet over 3 s after 1 s, Euler: {pallet_drift(engine) * 1e3:.3f} mm")
for engine in ("c", "warp"):
  first, lost, flips = toggles(engine)
  print(f"#2471 {engine}: first contact at step {first}, steps without contact after it {lost}, on/off switches {flips}")
