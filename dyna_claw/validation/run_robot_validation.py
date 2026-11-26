"""Full Taku integration diagnostic with actuator-driven arm and free motor."""

import argparse, csv, json, os
from pathlib import Path
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco as mj
import numpy as np
from run_grasp_validation import REPO, smooth

ARM = [
    "shoulder_pitch",
    "shoulder_roll",
    "shoulder_yaw",
    "elbow",
    "wrist_yaw",
    "wrist_pitch",
    "wrist_roll",
]


def ik(model, data, side, target):
    body = model.body(side + "_wrist_roll_link").id
    joints = [model.joint(side + "_" + n + "_joint").id for n in ARM]
    qa = model.jnt_qposadr[joints]
    va = model.jnt_dofadr[joints]
    for _ in range(300):
        mj.mj_forward(model, data)
        rot = data.xmat[body].reshape(3, 3)
        orient = 0.5 * sum(
            (np.cross(rot[:, i], np.eye(3)[:, i]) for i in range(3)), start=np.zeros(3)
        )
        error = np.r_[target - data.xpos[body], orient]
        if np.linalg.norm(error) < 1e-5:
            break
        jp = np.zeros((3, model.nv))
        jr = jp.copy()
        mj.mj_jacBody(model, data, jp, jr, body)
        jac = np.vstack([jp[:, va], jr[:, va]])
        delta = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(6) * 0.002, error)
        data.qpos[qa] = np.clip(
            data.qpos[qa] + np.clip(delta, -0.08, 0.08),
            model.jnt_range[joints, 0],
            model.jnt_range[joints, 1],
        )
    mj.mj_forward(model, data)
    return dict(
        zip([model.joint(i).name for i in joints], data.qpos[qa].tolist())
    ), float(np.linalg.norm(target - data.xpos[body]))


def run(side, dt, out, assets, render, compensate=False):
    out.mkdir(parents=True, exist_ok=True)
    original = mj.MjModel.from_xml_path(str(REPO / "dyna_taku/taku.xml"))
    origdata = mj.MjData(original)
    mj.mj_resetDataKeyframe(original, origdata, 0)
    mj.mj_forward(original, origdata)
    mj.mj_saveLastXML(str(out / "expanded_source.xml"), original)
    root = ET.parse(out / "expanded_source.xml").getroot()
    root.find("compiler").set("meshdir", str(REPO / "dyna_taku/assets"))
    root.find("compiler").set("texturedir", str(REPO / "dyna_taku"))
    root.remove(root.find("keyframe"))
    root.find("size").attrib.pop("nkey", None)
    root.find("option").set("timestep", str(dt))
    base = root.find(".//body[@name='agv_base_link']")
    ET.SubElement(root.find("equality"), "weld", body1="agv_base_link", solref=".002 1")
    base.set(
        "pos", " ".join(map(str, origdata.xpos[original.body("agv_base_link").id]))
    )
    root.find("worldbody").append(ET.Element("light", pos="-1 -1 2", dir="1 1 -2"))
    path = out / "scene.xml"
    ET.ElementTree(root).write(path)
    model = mj.MjModel.from_xml_path(str(path))
    data = mj.MjData(model)
    home = {
        original.joint(i).name: float(origdata.qpos[original.jnt_qposadr[i]])
        for i in range(original.njnt)
        if original.jnt_type[i] != mj.mjtJoint.mjJNT_FREE
    }
    for name, value in home.items():
        data.qpos[model.joint(name).qposadr[0]] = value
    # Both wrists forward of the chassis, fingers downward; no object teleport during execution.
    poses = {}
    errors = {}
    for hand in ["left", "right"]:
        data.qpos[model.joint(hand + "_elbow_joint").qposadr[0]] = -1.2
        poses[hand], errors[hand] = ik(
            model, data, hand, np.array([0.35, 0.24 if hand == "left" else -0.24, 0.80])
        )
        home.update(poses[hand])
    for hand in ["left", "right"]:
        home[hand + "_gripper_front_joint"] = -0.0345
        home[hand + "_gripper_rear_joint"] = 0.0345
    for name, value in home.items():
        data.qpos[model.joint(name).qposadr[0]] = value
    mj.mj_forward(model, data)
    wrist = model.body(side + "_wrist_roll_link").id
    center = data.xpos[wrist] + data.xmat[wrist].reshape(3, 3) @ np.array(
        [-0.0285205, 0, -0.233]
    )
    assert max(errors.values()) < 0.001, errors
    liftpose, lifterror = ik(
        model, data, side, data.xpos[wrist] + np.array([0, 0, 0.08])
    )
    assert lifterror < 0.001, lifterror
    world = root.find("worldbody")
    ET.SubElement(
        world, "geom", name="ground", type="plane", size="2 2 .01", rgba=".3 .33 .35 1"
    )
    floor = center[2] - 0.07347388928517218 / 2
    ET.SubElement(
        world,
        "geom",
        name="support",
        type="box",
        pos=f"{center[0]} {center[1]} {floor-.01}",
        size=".045 .045 .01",
        rgba=".3 .3 .3 1",
    )
    obj = ET.SubElement(
        world, "body", name="test_object", pos=" ".join(map(str, center))
    )
    ET.SubElement(obj, "freejoint", name="object_free")
    parts = json.loads((assets / "source.json").read_text())["collision_parts"]
    for i, part in enumerate(parts):
        ET.SubElement(
            root.find("asset"), "mesh", name=f"motor_{i}", file=str(assets / part)
        )
        ET.SubElement(
            obj,
            "geom",
            name=f"motor_{i}",
            type="mesh",
            mesh=f"motor_{i}",
            mass=str(0.2 / len(parts)),
            rgba=".3 .3 .35 1",
            condim="4",
        )
    ET.indent(root)
    ET.ElementTree(root).write(path)
    model = mj.MjModel.from_xml_path(str(path))
    data = mj.MjData(model)
    for name, value in home.items():
        data.qpos[model.joint(name).qposadr[0]] = value
    control = {}
    for i in range(model.nu):
        joint = model.actuator_trnid[i, 0]
        name = model.joint(joint).name
        control[i] = home.get(name, 0)
    data.ctrl[:] = list(control.values())
    mj.mj_forward(model, data)
    keys = ET.SubElement(root, "keyframe")
    ET.SubElement(
        keys,
        "key",
        name="initial",
        qpos=" ".join(map(str, data.qpos)),
        ctrl=" ".join(map(str, data.ctrl)),
    )
    ET.indent(root)
    ET.ElementTree(root).write(path)
    grip = model.actuator(side + "_gripper_joint").id
    objid = model.body("test_object").id
    qa = model.joint("object_free").qposadr[0]
    va = model.joint("object_free").dofadr[0]
    renderer = None
    if render:
        model.vis.global_.offwidth = 1200
        model.vis.global_.offheight = 1200
        model.vis.headlight.ambient[:] = 0.4
        renderer = mj.Renderer(model, 1200, 1200)
    camera = mj.MjvCamera()
    camera.lookat[:] = [-0.2, 0, 0.7]
    camera.distance = 2.4
    camera.azimuth = 145
    camera.elevation = -15
    opt = mj.MjvOption()
    opt.geomgroup[3:] = 0
    opt.sitegroup[:] = 0
    records = []
    snap = 0
    contact_log = []
    for step in range(round(5 / dt)):
        t = data.time
        for i in range(model.nu):
            name = model.joint(model.actuator_trnid[i, 0]).name
            data.ctrl[i] = control[i] + (
                liftpose.get(name, control[i]) - control[i]
            ) * smooth((t - 1.7) / 0.8)
        data.ctrl[grip] = (
            0.0345 + (-0.0055 - 0.0345) * smooth((t - 0.3) / 1)
            if t < 4
            else -0.0055 + 0.04 * smooth((t - 4) / 0.5)
        )
        if compensate:
            for i in range(model.nu):
                joint = model.actuator_trnid[i, 0]
                name = model.joint(joint).name
                if "agv_" not in name and "gripper" not in name:
                    data.ctrl[i] += (
                        data.qfrc_bias[model.jnt_dofadr[joint]]
                        / model.actuator_gainprm[i, 0]
                    )
        mj.mj_step(model, data)
        mj.mj_forward(model, data)
        depth = 0.0
        normal = 0.0
        housing = 0
        selfdepth = 0.0
        for i, c in enumerate(data.contact):
            bs = [model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]]
            names = [model.geom(c.geom1).name or "", model.geom(c.geom2).name or ""]
            if c.dist < 0:
                contact_log.append([float(data.time), *names, float(c.dist)])
            if objid in bs and not any(n in ["support", "ground"] for n in names):
                depth = max(depth, -c.dist)
                force = np.zeros(6)
                mj.mj_contactForce(model, data, i, force)
                normal += force[0]
                if any("wrist" in n for n in names):
                    housing += 1
            elif objid not in bs and not any(n in ["support", "ground"] for n in names):
                selfdepth = max(selfdepth, -c.dist)
        records.append(
            [
                data.time,
                *data.qpos[qa : qa + 7],
                *data.qvel[va : va + 6],
                depth,
                normal,
                data.actuator_force[grip],
                housing,
                selfdepth,
                sum(w.number for w in data.warning),
            ]
        )
        if renderer and t >= [0, 1.7, 2.8, 4.9][snap]:
            from PIL import Image

            renderer.update_scene(data, camera, scene_option=opt)
            Image.fromarray(renderer.render()).save(out / f"frame_{t:.2f}.png")
            snap += 1
            if snap == 4:
                renderer.close()
                renderer = None
    a = np.array(records)
    hold = a[(a[:, 0] > 2.6) & (a[:, 0] < 3.9)]
    with (out / "trajectory.csv").open("w") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "time",
                "x",
                "y",
                "z",
                "qw",
                "qx",
                "qy",
                "qz",
                "vx",
                "vy",
                "vz",
                "wx",
                "wy",
                "wz",
                "penetration",
                "normal_force",
                "actuator_force",
                "housing_contacts",
                "self_penetration",
                "warnings",
            ]
        )
        w.writerows(records)
    result = dict(
        side=side,
        dt=dt,
        gravity_feedforward=compensate,
        ik_error_m=errors,
        lift_ik_error_m=lifterror,
        hold_lift_m=float(np.median(hold[:, 3]) - center[2]),
        release_drop_m=float(np.median(hold[:, 3]) - a[-1, 3]),
        max_penetration_mm=float(a[:, 14].max() * 1000),
        max_force_n=float(a[:, 15].max()),
        max_housing_contacts=int(a[:, 17].max()),
        max_self_penetration_mm=float(a[:, 18].max() * 1000),
        warnings=int(a[:, 19].max()),
        finite=bool(np.isfinite(a).all()),
    )
    (out / "contact_pairs.json").write_text(json.dumps(contact_log))
    (out / "summary.json").write_text(json.dumps(result, indent=2))
    (out / "initial_joint_positions.json").write_text(json.dumps(home, indent=2))
    print(json.dumps(result), flush=True)
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--render", action="store_true")
    p.add_argument("--gravity-feedforward", action="store_true")
    p.add_argument("--side", choices=["left", "right"])
    a = p.parse_args()
    results = []
    for side in ([a.side] if a.side else ["left", "right"]):
        for dt in [0.001, 0.0005]:
            results.append(
                run(
                    side,
                    dt,
                    a.output
                    / f'robot_{side}_{dt:g}{"_feedforward" if a.gravity_feedforward else ""}',
                    a.output / "motor_assets",
                    a.render and dt == 0.001,
                    a.gravity_feedforward,
                )
            )
    (
        a.output
        / (
            "robot_feedforward_summary.json"
            if a.gravity_feedforward
            else "robot_summary.json"
        )
    ).write_text(json.dumps(results, indent=2))
