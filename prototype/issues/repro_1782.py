"""#1782: cylinder-box EPA on a degenerate face (both reproductions from the issue, rewritten)."""

import mujoco
import numpy as np
import warp as wp

import mujoco_warp as mjw

wp.config.quiet = True

FAR = """<mujoco><worldbody>
  <geom name="box" type="box" size="0.15 3.7 0.078" pos="-24.15 16.0 0.0"/>
  <body name="link4" pos="0 0 0"><freejoint/><geom name="cyl" type="cylinder" size="0.07524211 0.0225" pos="0 0 0.032"/></body>
</worldbody></mujoco>"""
FAR_QPOS = [
  -24.22393226623535,
  15.85853099822998,
  0.15526635944843292,
  -0.6466794610023499,
  -0.6712267398834229,
  -0.262784868478775,
  0.24940837919712067,
]

LYING = """<mujoco><option ccd_tolerance="1e-8" ccd_iterations="35"><flag{flag}/></option>
  <worldbody><geom name="box" type="box" size="0.35875 0.585 0.006" pos="0.66125 -0.025 0.744"/>
    <body pos="0.49133633746319805 -0.28182415790655796 0.7854842383627328"
          quat="0.4249830167428455 0.4056756931584754 0.5791691787286295 -0.5651369125011616"><freejoint/>
      <geom name="cyl" type="cylinder" size="0.0355 0.03668333333333333"/></body></worldbody></mujoco>"""


def warp_contacts(mjm, mjd, device, kinematics):
  with wp.ScopedDevice(device):
    m, d = mjw.put_model(mjm), mjw.put_data(mjm, mjd, nworld=1, nconmax=128, njmax=256)
    if kinematics:
      mjw.kinematics(m, d)
      mjw.collision(m, d)
    else:
      mjw.forward(m, d)
    n = int(d.nacon.numpy()[0])
    return d.contact.pos.numpy()[:n], d.contact.frame.numpy()[:n, 0], d.contact.dist.numpy()[:n]


mjm = mujoco.MjModel.from_xml_string(FAR)
mjd = mujoco.MjData(mjm)
mjd.qpos[:] = FAR_QPOS
mujoco.mj_forward(mjm, mjd)
for device in ("cuda:0", "cpu"):
  pos, _, dist = warp_contacts(mjm, mjd, device, kinematics=False)
  far = np.linalg.norm(pos - np.array([-24.15, 16.0, 0.0]), axis=1).max(initial=0)
  print(f"FAR {device}: ncon {len(pos)} max distance from box center {far:.3f} m  dist mm {np.round(dist * 1e3, 4).tolist()}")

for multiccd in (True, False):
  mjm = mujoco.MjModel.from_xml_string(LYING.format(flag="" if multiccd else ' multiccd="disable"'))
  mjd = mujoco.MjData(mjm)
  mujoco.mj_forward(mjm, mjd)
  c = [(round(1e3 * x.dist, 4), np.round(x.frame[:3], 3).tolist()) for x in mjd.contact[: mjd.ncon]]
  for device in ("cuda:0", "cpu"):
    _, nrm, dist = warp_contacts(mjm, mjd, device, kinematics=True)
    w = [(round(1e3 * float(dd), 4), np.round(nn, 3).tolist()) for dd, nn in zip(dist, nrm)]
    print(f"LYING multiccd={multiccd} {device}: C {c}  Warp {w}")
