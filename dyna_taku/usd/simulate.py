"""Open Taku in Isaac Sim 6.2 using Newton's MuJoCo Warp solver.

Run with Isaac's bundled Python. The original MJCF supplies the home controls;
USD supplies geometry, inertials, joints, collision/contact and actuator data.
"""

import argparse
import os
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--gpu", type=int, default=0)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--seconds", type=float, default=10)
parser.add_argument("--dt", type=float, default=0.0005)
args = parser.parse_args()
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
from isaacsim import SimulationApp

app = SimulationApp(
    {"headless": args.headless, "active_gpu": args.gpu, "physics_gpu": args.gpu, "multi_gpu": False}
)
import numpy as np
import mujoco as mj
import newton
import omni.usd
from pxr import UsdLux
from isaacsim.core.simulation_manager import SimulationManager, NewtonMjcScene

manager = app._app.get_extension_manager()
for extension in ["isaacsim.physics.newton", "isaacsim.physics.newton.tensors"]:
    manager.set_extension_enabled_immediate(extension, True)
from isaacsim.physics.newton import (
    acquire_stage,
    configure_newton,
    NewtonConfig,
    MuJoCoSolverConfig,
)

root = Path(__file__).resolve().parent
SimulationManager.switch_physics_engine("newton")
omni.usd.get_context().open_stage(str(root / "taku.usda"))
for _ in range(4):
    app.update()
stage = omni.usd.get_context().get_stage()
stage.SetEditTarget(stage.GetSessionLayer())
UsdLux.DomeLight.Define(stage, "/ViewerLight").CreateIntensityAttr(1200)
scene = NewtonMjcScene("/PhysicsScene")
scene.set_dt(args.dt)
scene.set_integrator("implicitfast")
scene.set_cone("elliptic")
cfg = NewtonConfig()
cfg.solver_cfg = MuJoCoSolverConfig(use_mujoco_contacts=True, njmax=4096, nconmax=1024)
# Isaac 6.2 forwards additional solver config attributes to SolverMuJoCo.
cfg.solver_cfg.enable_multiccd = True
cfg.num_substeps = 1
cfg.physics_frequency = round(1 / args.dt)
configure_newton(cfg)
ns = acquire_stage()
ns.initialize_newton(f"cuda:{args.gpu}")
assert ns.initialized
model = mj.MjModel.from_xml_path(str(root.parent / "scene.xml"))
data = mj.MjData(model)
mj.mj_resetDataKeyframe(model, data, 0)
q = ns.state_0.joint_q.numpy()
qd = ns.state_0.joint_qd.numpy()
qd[:] = 0
starts = ns.model.joint_q_start.numpy()
names = {model.joint(j).name: j for j in range(model.njnt)}
for i, label in enumerate(ns.model.joint_label):
    name = label.split("/")[-1]
    if i == 0:
        q[starts[i] : starts[i] + 7] = np.r_[data.qpos[:3], data.qpos[4:7], data.qpos[3]]
    elif name in names:
        q[starts[i]] = data.qpos[model.jnt_qposadr[names[name]]]
for state in [ns.state_0, ns.state_1]:
    state.joint_q.assign(q)
    state.joint_qd.assign(qd)
    newton.eval_fk(ns.model, state.joint_q, state.joint_qd, state)
native = ns.solver.mj_model
indices = ns.solver.mjc_actuator_to_newton_idx.numpy()
qi = ns.solver.mjc_actuator_to_newton_target_q_idx.numpy()
tq = ns.control.joint_target_q.numpy()
tv = ns.control.joint_target_qd.numpy()
for i in range(native.nu):
    joint = native.joint(native.actuator_trnid[i, 0]).name
    j = next(
        j for j in range(model.nu) if joint.endswith(model.joint(model.actuator_trnid[j, 0]).name)
    )
    if indices[i] >= 0:
        tq[qi[i]] = data.ctrl[j]
    elif indices[i] <= -2:
        tv[-indices[i] - 2] = data.ctrl[j]
ns.control.joint_target_q.assign(tq)
ns.control.joint_target_qd.assign(tv)
if not args.headless:
    from isaacsim.core.utils.viewports import set_camera_view

    set_camera_view(eye=np.array([2.5, -2, 1.5]), target=np.array([0, 0, 0.7]))
# Direct stepping keeps a single clock; do not also press timeline Play.
ns.playing = True
steps = 0
while app.is_running() and (not args.headless or steps * args.dt < args.seconds):
    for _ in range(max(1, round(1 / 60 / args.dt))):
        ns.step_sim(args.dt)
        steps += 1
    ns.update_fabric()
    app.update()
    if not np.isfinite(ns.state_0.joint_q.numpy()).all():
        raise RuntimeError("Nonfinite robot state")
print(f"Simulated {steps * args.dt:.3f} seconds with Isaac Newton / MuJoCo Warp", flush=True)
app.close()
