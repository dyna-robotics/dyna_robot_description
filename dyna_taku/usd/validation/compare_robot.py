from pathlib import Path
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--work-dir", type=Path, required=True)
parser.add_argument("--gpu", type=int, default=0)
parser.add_argument("--dt", type=float, default=0.0005)
args = parser.parse_args()
REPO = Path(__file__).resolve().parents[3]
USD = Path(__file__).resolve().parents[1]
import os, json, importlib.metadata as metadata

os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"
from isaacsim import SimulationApp

app = SimulationApp(
    {
        "headless": True,
        "active_gpu": args.gpu,
        "multi_gpu": False,
        "physics_gpu": args.gpu,
        "disable_viewport_updates": True,
    }
)
import omni.usd, omni.timeline
from isaacsim.core.simulation_manager import SimulationManager, NewtonMjcScene

manager = app._app.get_extension_manager()
for name in ["isaacsim.physics.newton", "isaacsim.physics.newton.tensors"]:
    manager.set_extension_enabled_immediate(name, True)
from isaacsim.physics.newton import (
    acquire_stage,
    configure_newton,
    NewtonConfig,
    MuJoCoSolverConfig,
)

r = args.work_dir.resolve()
r.mkdir(parents=True, exist_ok=True)
(r / "isaac62_results").mkdir(exist_ok=True)
print(
    "VERSIONS",
    {p: metadata.version(p) for p in ["isaacsim", "newton", "mujoco", "mujoco-warp", "warp-lang"]},
    flush=True,
)
print("ENGINES", SimulationManager.get_available_physics_engines(verbose=True), flush=True)
assert SimulationManager.switch_physics_engine("newton")
omni.usd.get_context().open_stage(str(USD / "taku.usda"))
for _ in range(4):
    app.update()
scene = NewtonMjcScene("/PhysicsScene")
scene.set_dt(args.dt)
scene.set_cone("elliptic")
scene.set_integrator("implicitfast")
cfg = NewtonConfig()
cfg.solver_cfg = MuJoCoSolverConfig(
    use_mujoco_contacts=True,
    njmax=4096,
    nconmax=1024,
    save_to_mjcf=str(r / "isaac62_runtime_robot.xml"),
)
cfg.solver_cfg.enable_multiccd = True
cfg.num_substeps = 1
cfg.physics_frequency = round(1 / args.dt)
configure_newton(cfg)
ns = acquire_stage()
assert ns is not None
ns.initialize_newton(f"cuda:{args.gpu}")
assert ns.initialized

import numpy as np, mujoco as mj, mujoco_warp as mw, warp as wp, newton

wp.set_device(f"cuda:{args.gpu}")
m = mj.MjModel.from_xml_path(str(REPO / "dyna_taku/scene.xml"))
m.opt.timestep = args.dt
d = mj.MjData(m)
mj.mj_resetDataKeyframe(m, d, 0)
mj.mj_forward(m, d)
q = ns.state_0.joint_q.numpy()
qd = ns.state_0.joint_qd.numpy()
qd[:] = 0
starts = ns.model.joint_q_start.numpy()
for i, label in enumerate(ns.model.joint_label):
    name = label.split("/")[-1]
    if i == 0:
        q[starts[i] : starts[i] + 7] = np.r_[d.qpos[:3], d.qpos[4:7], d.qpos[3]]
    elif name in [m.joint(j).name for j in range(m.njnt)]:
        q[starts[i]] = d.qpos[m.joint(name).qposadr[0]]
for state in [ns.state_0, ns.state_1]:
    state.joint_q.assign(q)
    state.joint_qd.assign(qd)
    newton.eval_fk(ns.model, state.joint_q, state.joint_qd, state)
wm = mw.put_model(m)
wd = mw.put_data(m, d, nconmax=1024, njmax=4096)
mw.step(wm, wd)
wp.synchronize()
wd = mw.put_data(m, d, nconmax=1024, njmax=4096)
with wp.ScopedCapture() as cap:
    mw.step(wm, wd)
native = ns.solver.mj_model
indices = ns.solver.mjc_actuator_to_newton_idx.numpy()
qindices = ns.solver.mjc_actuator_to_newton_target_q_idx.numpy()
actmap = []
for i in range(native.nu):
    name = native.joint(native.actuator_trnid[i, 0]).name
    j = next(j for j in range(m.nu) if name.endswith(m.joint(m.actuator_trnid[j, 0]).name))
    actmap.append(j)
body_names = ["agv_base_link", "torso_body_link", "left_wrist_roll_link", "right_wrist_roll_link"]
bodyids = [
    next(i for i, l in enumerate(ns.model.body_label) if l.endswith("/" + n)) for n in body_names
]
sourceids = [m.body(n).id for n in body_names]
initial_error = (
    np.linalg.norm(ns.state_0.body_q.numpy()[bodyids, :3] - d.xpos[sourceids], axis=1) * 1000
)
print("INITIAL_POSITION_ERROR_MM", initial_error.tolist(), flush=True)
records = []
gpu_pose = mj.MjData(m)
for step in range(round(2 / args.dt)):
    t = step * args.dt
    ctrl = m.key_ctrl[0].copy()
    for i in range(m.nu):
        if "agv" not in m.actuator(i).name:
            ctrl[i] += (
                (0.001 if "gripper" in m.actuator(i).name else 0.02)
                * np.sin(t * 2 * np.pi + 0.1 * i)
                * min(1, t / 0.2)
            )
    tq = ns.control.joint_target_q.numpy()
    tv = ns.control.joint_target_qd.numpy()
    for i, j in enumerate(actmap):
        if indices[i] >= 0:
            tq[qindices[i]] = ctrl[j]
        elif indices[i] <= -2:
            tv[-indices[i] - 2] = ctrl[j]
    ns.control.joint_target_q.assign(tq)
    ns.control.joint_target_qd.assign(tv)
    d.ctrl[:] = ctrl
    mj.mj_step(m, d)
    mj.mj_forward(m, d)
    wd.ctrl.assign(ctrl[None, :].astype(np.float32))
    wp.capture_launch(cap.graph)
    ns.step_sim(args.dt)
    if step % 10 == 0:
        positions = ns.state_0.body_q.numpy()[bodyids, :3]
        gpu_pose.qpos[:] = wd.qpos.numpy()[0]
        mj.mj_forward(m, gpu_pose)
        warp_pos = gpu_pose.xpos[sourceids]
        records.append(
            [t + args.dt, *positions.ravel(), *d.xpos[sourceids].ravel(), *warp_pos.ravel()]
        )
arr = np.array(records)
a = arr[:, 1:13].reshape(-1, 4, 3)
b = arr[:, 13:25].reshape(-1, 4, 3)
c = arr[:, 25:37].reshape(-1, 4, 3)
result = {
    "duration_seconds": 2,
    "initial_body_position_error_mm": initial_error.tolist(),
    "versions": {
        p: metadata.version(p) for p in ["isaacsim", "newton", "mujoco", "mujoco-warp", "warp-lang"]
    },
    "dt": args.dt,
    "finite": bool(np.isfinite(arr).all()),
    "body_names": body_names,
    "max_body_position_difference_vs_cpu_mm": np.max(
        np.linalg.norm(a - b, axis=2), axis=0
    ).tolist(),
    "max_body_position_difference_vs_warp_mm": np.max(
        np.linalg.norm(a - c, axis=2), axis=0
    ).tolist(),
    "warnings_cpu": int(d.warning.number.sum()),
    "scope": "small simultaneous upper-body actuation from home; not a joint-limit sweep or full-robot grasp",
}
for key in ["max_body_position_difference_vs_cpu_mm", "max_body_position_difference_vs_warp_mm"]:
    result[key] = [v * 1000 for v in result[key]]
(r / "isaac62_results/robot_motion.json").write_text(json.dumps(result, indent=2))
np.savetxt(r / "isaac62_results/robot_motion.csv", arr, delimiter=",")
print("RESULT", result, flush=True)
app.close()
assert result["finite"] and result["warnings_cpu"] == 0
assert max(result["max_body_position_difference_vs_cpu_mm"]) < 0.1
assert max(result["max_body_position_difference_vs_warp_mm"]) < 0.1
