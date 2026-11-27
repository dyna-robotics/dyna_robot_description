"""Step Isaac Sim's actual NewtonStage backend; compare against bundled MuJoCo."""

from pathlib import Path
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--work-dir", type=Path, required=True)
parser.add_argument("--gpu", type=int, default=0)
parser.add_argument(
    "--cases",
    nargs="+",
    default=["box_center", "cylinder_center", "thin_plate", "box_offset", "box_tilt", "tip_edge"],
)
parser.add_argument("--timesteps", nargs="+", type=float, default=[0.001, 0.0005])
args = parser.parse_args()
REPO = Path(__file__).resolve().parents[3]
USD = Path(__file__).resolve().parents[1]
import os, json, time, importlib.metadata as metadata

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
import omni.usd, numpy as np, mujoco as mj, mujoco_warp as mw, newton, warp as wp
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
out = r / "isaac62_results"
out.mkdir(exist_ok=True)
wp.config.kernel_cache_dir = str(r / "isaac62_warp_cache")
wp.set_device(f"cuda:{args.gpu}")
versions = {
    p: metadata.version(p) for p in ["isaacsim", "newton", "mujoco", "mujoco-warp", "warp-lang"]
}
print("VERSIONS", versions, flush=True)
assert SimulationManager.switch_physics_engine("newton")


def smooth(t):
    t = np.clip(t, 0, 1)
    return t * t * (3 - 2 * t)


results = []
for case, dt in [(c, h) for h in args.timesteps for c in args.cases]:
    source = r / "contact_results" / f"{case}_{dt}_mjcf/scene.xml"
    asset = r / "contact_results" / f"{case}_{dt}_usd/usd/compat.usda"
    omni.usd.get_context().open_stage(str(asset))
    for _ in range(4):
        app.update()
    scene = NewtonMjcScene("/PhysicsScene")
    scene.set_dt(dt)
    scene.set_cone("elliptic")
    scene.set_integrator("implicitfast")
    cfg = NewtonConfig()
    cfg.solver_cfg = MuJoCoSolverConfig(
        use_mujoco_contacts=True,
        njmax=512,
        nconmax=128,
        save_to_mjcf=str(out / f"{case}_import.xml"),
    )
    cfg.solver_cfg.enable_multiccd = True
    cfg.num_substeps = 1
    cfg.physics_frequency = round(1 / dt)
    configure_newton(cfg)
    ns = acquire_stage()
    ns.initialize_newton(f"cuda:{args.gpu}")
    assert ns.initialized
    ref = mj.MjModel.from_xml_path(str(source))
    data = mj.MjData(ref)
    mj.mj_resetDataKeyframe(ref, data, 0)
    mj.mj_forward(ref, data)
    q_lookup = {}
    labels = ns.model.joint_label
    starts = ns.model.joint_q_start.numpy()
    q = ns.state_0.joint_q.numpy()
    qd = ns.state_0.joint_qd.numpy()
    qd[:] = 0
    for name, val in [("gripper_front_joint", -0.028), ("gripper_rear_joint", 0.028)]:
        index = next(
            i for i, label in enumerate(labels) if label.endswith("/" + name) or label == name
        )
        q[starts[index]] = val
        q_lookup[name] = int(starts[index])
    for state in [ns.state_0, ns.state_1]:
        state.joint_q.assign(q)
        state.joint_qd.assign(qd)
        newton.eval_fk(ns.model, state.joint_q, state.joint_qd, state)
    native = ns.solver.mj_model
    (out / f"{case}_parameters.json").write_text(
        json.dumps(
            {
                side: {
                    "option": {
                        a: float(getattr(model.opt, a))
                        for a in [
                            "timestep",
                            "impratio",
                            "tolerance",
                            "iterations",
                            "ls_iterations",
                            "integrator",
                            "cone",
                            "solver",
                            "disableflags",
                            "enableflags",
                        ]
                    },
                    "actuator_gainprm": model.actuator_gainprm.tolist(),
                    "actuator_biasprm": model.actuator_biasprm.tolist(),
                    "joint_damping": model.dof_damping.tolist(),
                    "friction": model.geom_friction.tolist(),
                    "solref": model.geom_solref.tolist(),
                }
                for side, model in [("source", ref), ("isaac", native)]
            },
            indent=2,
        )
    )
    wm = mw.put_model(ref)
    wd = mw.put_data(ref, data, nconmax=128, njmax=512)
    mw.step(wm, wd)
    wp.synchronize()
    wd = mw.put_data(ref, data, nconmax=128, njmax=512)
    with wp.ScopedCapture() as cap:
        mw.step(wm, wd)
    warp_positions = []
    pose_records = []
    object_q = int(ref.joint("object_free").qposadr[0])
    ctrl_source = ns.solver.mjc_actuator_ctrl_source.numpy()
    ctrl_index = ns.solver.mjc_actuator_to_newton_idx.numpy()
    assert (ctrl_source == 0).all(), ctrl_source
    ctrl_index = ns.solver.mjc_actuator_to_newton_target_q_idx.numpy()
    native_control = ns.control.joint_target_q
    mapping = {}
    for i in range(native.nu):
        name = native.joint(native.actuator_trnid[i, 0]).name
        if name.endswith("gripper_rear_joint"):
            mapping["grip"] = int(ctrl_index[i])
        elif name.endswith("mount_lift"):
            mapping["lift"] = int(ctrl_index[i])
    assert len(mapping) == 2, mapping
    obj = next(
        i
        for i, label in enumerate(ns.model.body_label)
        if label.endswith("/test_object") or label == "test_object"
    )
    refobj = ref.body("test_object").id
    ref_fingers = {ref.body(n).id for n in ["gripper_front_link", "gripper_rear_link"]}
    native_fingers = {
        i
        for i in range(native.nbody)
        if native.body(i).name.endswith(("gripper_front_link", "gripper_rear_link"))
    }
    refgrip = ref.actuator("gripper_joint").id
    reflift = ref.actuator("mount_servo").id
    initial = ns.state_0.body_q.numpy()[obj, :3].copy()
    assert np.max(abs(initial - data.xpos[refobj])) < 1e-5, (initial, data.xpos[refobj])
    rows = []
    start = time.monotonic()
    gpu_data = mj.MjData(native)
    sampled_penetration = 0.0
    sampled_force = 0.0
    ref_penetration = 0.0
    ref_force = 0.0
    grip_force = 0.0
    ref_grip_force = 0.0
    native_obj = next(i for i in range(native.nbody) if native.body(i).name.endswith("test_object"))
    for step in range(round(5 / dt)):
        t = step * dt
        grip = (
            0.028 + (-0.0055 - 0.028) * smooth(t - 0.3)
            if t < 4
            else -0.0055 + (0.028 + 0.0055) * smooth((t - 4) / 0.5)
        )
        lift = 0.08 * smooth((t - 1.7) / 0.8)
        ctrl = np.zeros(native_control.shape, dtype=np.float32)
        ctrl[mapping["grip"]] = grip
        ctrl[mapping["lift"]] = lift
        native_control.assign(ctrl)
        data.ctrl[refgrip] = grip
        data.ctrl[reflift] = lift
        mj.mj_step(ref, data)
        mj.mj_forward(ref, data)
        wd.ctrl.assign(data.ctrl[None, :].astype(np.float32))
        wp.capture_launch(cap.graph)
        ns.step_sim(dt)
        if step % 10 == 0:
            warp_positions.append(wd.qpos.numpy()[0, object_q : object_q + 3].copy())
            nsq = ns.state_0.joint_q.numpy()
            pose = data.qpos.copy()
            for joint_name in ["gripper_front_joint", "gripper_rear_joint", "mount_lift"]:
                ji = next(
                    i
                    for i, l in enumerate(labels)
                    if l.endswith("/" + joint_name) or l == joint_name
                )
                pose[ref.joint(joint_name).qposadr[0]] = nsq[starts[ji]]
            transform = ns.state_0.body_q.numpy()[obj]
            pose[object_q : object_q + 7] = np.r_[transform[:3], transform[6], transform[3:6]]
            pose_records.append([pose, data.qpos.copy(), wd.qpos.numpy()[0].copy()])
            pos = ns.state_0.body_q.numpy()[obj, :3]
            joints = ns.state_0.joint_q.numpy()
            opening = (
                joints[q_lookup["gripper_rear_joint"]] - joints[q_lookup["gripper_front_joint"]]
            )
            ref_opening = float(
                data.qpos[ref.joint("gripper_rear_joint").qposadr[0]]
                - data.qpos[ref.joint("gripper_front_joint").qposadr[0]]
            )
            rows.append([t + dt, *pos, *data.xpos[refobj], opening, ref_opening])
            mw.get_data_into(gpu_data, native, ns.solver.mjw_data)
            grip_force = max(
                grip_force,
                abs(
                    float(
                        gpu_data.actuator_force[
                            next(
                                i
                                for i in range(native.nu)
                                if native.joint(native.actuator_trnid[i, 0]).name.endswith(
                                    "gripper_rear_joint"
                                )
                            )
                        ]
                    )
                ),
            )
            ref_grip_force = max(ref_grip_force, abs(float(data.actuator_force[refgrip])))
            for ci, c in enumerate(data.contact):
                if refobj not in (ref.geom_bodyid[c.geom[0]], ref.geom_bodyid[c.geom[1]]):
                    continue
                if not ({ref.geom_bodyid[c.geom[0]], ref.geom_bodyid[c.geom[1]]} & ref_fingers):
                    continue
                ref_penetration = max(ref_penetration, -float(c.dist))
                force = np.zeros(6)
                mj.mj_contactForce(ref, data, ci, force)
                ref_force = max(ref_force, abs(float(force[0])))
            for ci, c in enumerate(gpu_data.contact):
                if native_obj not in (native.geom_bodyid[c.geom[0]], native.geom_bodyid[c.geom[1]]):
                    continue
                if not (
                    {native.geom_bodyid[c.geom[0]], native.geom_bodyid[c.geom[1]]} & native_fingers
                ):
                    continue
                sampled_penetration = max(sampled_penetration, -float(c.dist))
                force = np.zeros(6)
                mj.mj_contactForce(native, gpu_data, ci, force)
                sampled_force = max(sampled_force, abs(float(force[0])))
        if step % 1000 == 0:
            print("STEP", case, step, flush=True)
    arr = np.array(rows)
    hold = (arr[:, 0] > 2.6) & (arr[:, 0] < 3.9)
    result = {
        "case": case,
        "versions": versions,
        "gpu": str(ns.device),
        "actual_isaac_newton_backend": True,
        "native_mujoco_contacts": True,
        "dt": dt,
        "elapsed_seconds_including_compile": time.monotonic() - start,
        "finite": bool(np.isfinite(arr).all()),
        "maximum_object_position_difference_mm": float(
            np.max(np.linalg.norm(arr[:, 1:4] - arr[:, 4:7], axis=1)) * 1000
        ),
        "isaac_hold_lift_mm": float((np.median(arr[hold, 3]) - initial[2]) * 1000),
        "isaac_hold_drift_mm": float((arr[hold, 3][-1] - arr[hold, 3][0]) * 1000),
        "isaac_release_drop_mm": float((np.median(arr[hold, 3]) - arr[-1, 3]) * 1000),
        "mujoco_warnings": int(data.warning.number.sum()),
    }
    result["max_vs_source_warp_mm"] = float(
        np.max(np.linalg.norm(arr[:, 1:4] - np.array(warp_positions), axis=1)) * 1000
    )
    result["max_before_release_vs_warp_mm"] = float(
        np.max(
            np.linalg.norm(
                arr[arr[:, 0] < 4, 1:4] - np.array(warp_positions)[arr[:, 0] < 4], axis=1
            )
        )
        * 1000
    )
    result["max_before_release_vs_cpu_mm"] = float(
        np.max(np.linalg.norm(arr[arr[:, 0] < 4, 1:4] - arr[arr[:, 0] < 4, 4:7], axis=1)) * 1000
    )
    result["sampled_max_penetration_mm"] = 1000 * sampled_penetration
    result["sampled_max_contact_normal_force_n"] = sampled_force
    result["reference_sampled_max_penetration_mm"] = 1000 * ref_penetration
    result["reference_sampled_max_contact_normal_force_n"] = ref_force
    result["max_opening_difference_mm"] = float(np.max(np.abs(arr[:, 7] - arr[:, 8])) * 1000)
    result["sampled_gripper_actuator_force_peak_n"] = grip_force
    result["reference_sampled_gripper_actuator_force_peak_n"] = ref_grip_force
    result["retained_and_released"] = (
        result["isaac_hold_lift_mm"] > 60
        and result["isaac_release_drop_mm"] > 50
        and abs(result["isaac_hold_drift_mm"]) < 2
    )
    np.savetxt(
        out / f"{case}_{dt}.csv",
        arr,
        delimiter=",",
        header="time,isaac_x,isaac_y,isaac_z,mujoco_x,mujoco_y,mujoco_z,isaac_opening,mujoco_opening",
    )
    np.savetxt(
        out / f"{case}_{dt}_warp.csv",
        np.array(warp_positions),
        delimiter=",",
        header="warp_x,warp_y,warp_z",
    )
    np.savez_compressed(
        out / f"{case}_{dt}_poses.npz",
        times=arr[:, 0],
        qpos=np.array(pose_records),
        engines=["isaac", "mujoco_cpu", "mujoco_warp"],
    )
    results.append(result)
    (out / "contact_results.json").write_text(json.dumps(results, indent=2))
    print("RESULT", result, flush=True)
app.close()
assert all(
    x["finite"]
    and x["retained_and_released"]
    and x["sampled_max_penetration_mm"] < 0.5
    and x["max_before_release_vs_cpu_mm"] < 1
    and x["max_before_release_vs_warp_mm"] < 1
    for x in results
), "Grasp screening failed; inspect contact_results.json"
