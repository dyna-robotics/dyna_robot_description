"""Reproducible actuator-driven contact experiments; never attach the object."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco as mj
import numpy as np

REPO = Path(__file__).resolve().parents[2]
SOURCE = REPO / "dyna_claw/models/dynaclaw.xml"
CASES = [
    dict(name="box_center", shape="box"),
    dict(name="cylinder_center", shape="cylinder"),
    dict(name="thin_plate", shape="plate"),
    dict(name="box_offset", shape="box", offset=0.004),
    dict(name="box_tilt", shape="box", tilt=15),
    dict(name="tip_edge", shape="box", depth=-0.247, halfheight=0.009),
    dict(name="surface_slide", shape="box", slide=True),
    dict(
        name="seam_slide",
        shape="box",
        slide=True,
        slide_axis=2,
        slide_force=-2,
        depth=-0.225,
        halfheight=0.006,
    ),
    dict(name="thin_plate_stiff_contact", shape="plate", stiff=True),
    dict(name="motor_200g", shape="motor", mass=0.2),
    dict(name="motor_500g", shape="motor", mass=0.5),
    dict(name="motor_200g_long_hold", shape="motor", mass=0.2, release_time=10),
]


def smooth(t):
    t = np.clip(t, 0, 1)
    return t * t * (3 - 2 * t)


def build(case, dt, output):
    root = ET.parse(SOURCE).getroot()
    root.find("compiler").set(
        "meshdir",
        str((SOURCE.parent / root.find("compiler").get("meshdir", "")).resolve()),
    )
    root.remove(root.find("keyframe"))
    if case.get("stiff"):
        for name in ("dynaclaw_collision", "dynaclaw_soft_pad", "dynaclaw_rigid_nail"):
            geom = root.find(f".//default[@class='{name}']/geom")
            if geom is not None:
                geom.set("solref", ".005 1")
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", str(dt))
    option.set("integrator", "implicitfast")
    option.set("gravity", "0 0 -9.81")
    ET.SubElement(root, "visual").append(
        ET.Element("global", offwidth="960", offheight="960")
    )
    world = root.find("worldbody")
    mount = world.find("body")
    mount.set("pos", "0 0 .5")
    ET.SubElement(mount, "inertial", pos="0 0 0", mass="1", diaginertia=".01 .01 .01")
    ET.SubElement(
        mount, "joint", name="mount_lift", type="slide", axis="0 0 1", damping="20"
    )
    ET.SubElement(
        root.find("actuator"),
        "position",
        name="mount_servo",
        joint="mount_lift",
        kp="5000",
        kv="150",
        forcerange="-200 200",
    )
    halfheight = case.get("halfheight", 0.015)
    shape = case["shape"]
    size = ".016 .012 " + str(halfheight)
    if shape == "plate":
        size = ".016 .0015 " + str(halfheight)
    if shape == "cylinder":
        size = ".012 " + str(halfheight)
    if shape == "motor":
        halfheight = 0.07347388928517218 / 2
    center = [-0.0285205, case.get("offset", 0), 0.5 + case.get("depth", -0.233)]
    floor = center[2] - halfheight
    ET.SubElement(
        world,
        "geom",
        name="support",
        type="plane",
        size=".3 .3 .01",
        pos=f"0 0 {floor}",
        rgba=".24 .27 .3 1",
    )
    obj = ET.SubElement(
        world,
        "body",
        name="test_object",
        pos=" ".join(map(str, center)),
        quat=f'{math.cos(math.radians(case.get("tilt",0))/2)} 0 0 {math.sin(math.radians(case.get("tilt",0))/2)}',
    )
    ET.SubElement(obj, "freejoint", name="object_free")
    if shape == "motor":
        assets = output.parent / "motor_assets"
        parts = json.loads((assets / "source.json").read_text())["collision_parts"]
        for i, filename in enumerate(parts):
            mesh = assets / filename
            ET.SubElement(root.find("asset"), "mesh", name=f"motor_{i}", file=str(mesh))
            ET.SubElement(
                obj,
                "geom",
                name=f"motor_{i}",
                type="mesh",
                mesh=f"motor_{i}",
                mass=str(case["mass"] / len(parts)),
                rgba=".25 .28 .3 1",
                condim="4",
            )
        assert len(parts) > 0
    else:
        ET.SubElement(
            obj,
            "geom",
            name="object_geom",
            type="cylinder" if shape == "cylinder" else "box",
            size=size,
            mass=".05",
            rgba=".85 .45 .1 1",
            friction="1 .005 .0001",
            condim="4",
        )
    if case.get("stiff"):
        for geom in obj.findall("geom"):
            geom.set("solref", ".005 1")
    ET.SubElement(world, "light", pos="0 -1 1.5", dir="0 1 -1", diffuse=".7 .7 .7")
    path = output / "scene.xml"
    ET.indent(root)
    ET.ElementTree(root).write(path)
    return mj.MjModel.from_xml_path(str(path)), center


def run(case, dt, output, render=False):
    output.mkdir(parents=True, exist_ok=True)
    model, center = build(case, dt, output)
    data = mj.MjData(model)
    front = model.joint("gripper_front_joint").qposadr[0]
    rear = model.joint("gripper_rear_joint").qposadr[0]
    opening = 0.0345 if case["shape"] == "motor" else 0.028
    data.qpos[front] = -opening
    data.qpos[rear] = opening
    grip = model.actuator("gripper_joint").id
    lift = model.actuator("mount_servo").id
    obj = model.body("test_object").id
    objjoint = model.joint("object_free")
    qa = objjoint.qposadr[0]
    va = objjoint.dofadr[0]
    data.ctrl[grip] = opening
    mj.mj_forward(model, data)
    scene = ET.parse(output / "scene.xml")
    keys = ET.SubElement(scene.getroot(), "keyframe")
    ET.SubElement(
        keys,
        "key",
        name="initial",
        qpos=" ".join(map(str, data.qpos)),
        ctrl=" ".join(map(str, data.ctrl)),
    )
    ET.indent(scene)
    scene.write(output / "scene.xml")
    fingerids = {
        model.body("gripper_front_link").id,
        model.body("gripper_rear_link").id,
    }
    model.vis.headlight.ambient[:] = 0.4
    model.vis.headlight.diffuse[:] = 0.6
    renderer = mj.Renderer(model, 960, 960) if render else None
    camera = mj.MjvCamera()
    camera.lookat[:] = [-0.028, 0, 0.32]
    camera.distance = 0.42
    camera.azimuth = 145
    camera.elevation = -15
    opt = mj.MjvOption()
    opt.geomgroup[3:] = 0
    rows = []
    contacts = []
    force_rows = []
    release_time = case.get("release_time", 4)
    snapshot_times = [0, 0.8, 1.7, 2.5, 3.5, 4.3, 4.9]
    snap = 0
    for step in range(round((release_time + 1) / dt)):
        t = data.time
        data.ctrl[grip] = (
            opening + (-0.0055 - opening) * smooth((t - 0.3) / 1.0)
            if t < release_time
            else -0.0055 + (opening + 0.0055) * smooth((t - release_time) / 0.5)
        )
        data.ctrl[lift] = 0.08 * smooth((t - 1.7) / 0.8)
        data.xfrc_applied[obj, :] = 0
        if case.get("slide") and 2.6 < t < 3.3:
            data.xfrc_applied[obj, case.get("slide_axis", 0)] = case.get(
                "slide_force", 0.25
            ) * math.sin(math.pi * (t - 2.6) / 0.7)
        mj.mj_step(model, data)
        # Recompute contacts and forces for the same timestamp as the state.
        mj.mj_forward(model, data)
        fn = 0.0
        depth = 0.0
        count = 0
        pairset = set()
        for ci, contact in enumerate(data.contact):
            b1 = model.geom_bodyid[contact.geom1]
            b2 = model.geom_bodyid[contact.geom2]
            if obj not in (b1, b2) or not ({b1, b2} & fingerids):
                continue
            force = np.zeros(6)
            mj.mj_contactForce(model, data, ci, force)
            fn += force[0]
            depth = max(depth, -contact.dist)
            count += 1
            other = contact.geom2 if b1 == obj else contact.geom1
            pairset.add(model.geom(other).name)
            force_rows.append(
                [
                    data.time,
                    model.geom(other).name,
                    contact.dist,
                    *contact.pos,
                    *contact.frame,
                    *force,
                ]
            )
        rows.append(
            [
                data.time,
                *data.qpos[qa : qa + 7],
                *data.qvel[va : va + 6],
                data.qpos[rear] - data.qpos[front],
                data.ctrl[grip],
                data.actuator_force[grip],
                depth,
                fn,
                count,
                int(sum(w.number for w in data.warning)),
            ]
        )
        if pairset:
            contacts.append((data.time, sorted(pairset)))
        if renderer and snap < len(snapshot_times) and t >= snapshot_times[snap]:
            from PIL import Image

            renderer.update_scene(data, camera, scene_option=opt)
            Image.fromarray(renderer.render()).save(output / f"frame_{t:.2f}.png")
            snap += 1
    if renderer:
        renderer.close()
    columns = [
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
        "opening",
        "command",
        "actuator_force",
        "penetration",
        "normal_force",
        "contacts",
        "warnings",
    ]
    a = np.array(rows)
    with (output / "trajectory.csv").open("w") as f:
        writer = csv.writer(f)
        writer.writerow(columns)
        writer.writerows(rows)
    with (output / "contact_forces.csv").open("w") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "time",
                "finger_geom",
                "signed_distance",
                "x",
                "y",
                "z",
                *[f"frame_{i}" for i in range(9)],
                "normal",
                "tangent1",
                "tangent2",
                "torque_normal",
                "torque_tangent1",
                "torque_tangent2",
            ]
        )
        writer.writerows(force_rows)
    (output / "contact_pairs.json").write_text(json.dumps(contacts))
    hold = a[(a[:, 0] > 2.6) & (a[:, 0] < release_time - 0.1)]
    result = dict(
        case=case,
        dt=dt,
        mujoco=mj.__version__,
        mass_kg=case.get("mass", 0.05),
        max_penetration_mm=float(a[:, 17].max() * 1000),
        max_normal_force_n=float(a[:, 18].max()),
        max_speed_m_s=float(np.linalg.norm(a[:, 8:11], axis=1).max()),
        warnings=int(a[:, -1].max()),
        hold_lift_m=float(np.median(hold[:, 3]) - center[2]),
        hold_z_range_mm=float(np.ptp(hold[:, 3]) * 1000),
        release_drop_m=float(np.median(hold[:, 3]) - a[-1, 3]),
        finite=bool(np.isfinite(a).all()),
        peak_actuator_force_n=float(abs(a[:, 16]).max()),
    )
    result["hold_z_drift_mm"] = float((hold[-1, 3] - hold[0, 3]) * 1000)
    result["pre_release_max_speed_m_s"] = float(
        np.linalg.norm(a[a[:, 0] < release_time, 8:11], axis=1).max()
    )
    result["warning_counts"] = {
        mj.mjtWarning(i).name: int(w.number)
        for i, w in enumerate(data.warning)
        if w.number
    }
    result["lifted"] = bool(result["hold_lift_m"] > 0.06)
    result["retained_until_release"] = bool(hold[-1, 3] - center[2] > 0.02)
    result["stable_hold_2mm"] = bool(
        result["retained_until_release"] and abs(result["hold_z_drift_mm"]) <= 2
    )
    result["released"] = bool(result["release_drop_m"] > 0.05)
    (output / "summary.json").write_text(json.dumps(result, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--case", choices=[case["name"] for case in CASES])
    parser.add_argument("--dt", nargs="+", type=float, default=[0.001, 0.0005])
    args = parser.parse_args()
    summaries = []
    for case in CASES:
        if args.case and case["name"] != args.case:
            continue
        for dt in args.dt:
            result = run(
                case,
                dt,
                args.output / f'{case["name"]}_{dt:g}',
                args.render and dt == 0.001,
            )
            summaries.append(result)
            print(json.dumps(result), flush=True)
    (
        args.output
        / ("summary_" + args.case + ".json" if args.case else "summary.json")
    ).write_text(json.dumps(summaries, indent=2))


if __name__ == "__main__":
    main()
