"""Check standalone assembly, generated instances, and opening clearances."""

import hashlib
import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# MuJoCo exports these from compiled modules without Python attribute stubs.
from mujoco import (
    MjData,  # pyright: ignore[reportAttributeAccessIssue]
    MjModel,  # pyright: ignore[reportAttributeAccessIssue]
    mj_forward,  # pyright: ignore[reportAttributeAccessIssue]
    mj_geomDistance,  # pyright: ignore[reportAttributeAccessIssue]
    mj_id2name,  # pyright: ignore[reportAttributeAccessIssue]
    mj_resetDataKeyframe,  # pyright: ignore[reportAttributeAccessIssue]
    mjtObj,  # pyright: ignore[reportAttributeAccessIssue]
)

from dyna_claw.generate_taku_includes import generate

REPOSITORY = Path(__file__).resolve().parent.parent


class GripperModelTest(unittest.TestCase):
    def test_full_nail_and_housing_exclusion(self) -> None:
        model = ET.parse(REPOSITORY / "dyna_claw/models/dynaclaw.xml").getroot()
        for finger in ("front", "rear"):
            body = model.find(f".//body[@name='gripper_{finger}_link']")
            assert body is not None
            self.assertIsNone(
                body.find(f"geom[@name='gripper_{finger}_collision_mount']")
            )
            visual = body.find(f"geom[@name='gripper_{finger}_nail_visual']")
            assert visual is not None
            self.assertEqual(visual.attrib["material"], "dynaclaw_aluminum")
            asset = model.find(f"asset/mesh[@name='{visual.attrib['mesh']}']")
            assert asset is not None
            mesh = REPOSITORY / "dyna_claw/assets" / asset.attrib["file"]
            vertices = np.array(
                [
                    [float(value) for value in line.split()[1:4]]
                    for line in mesh.read_text().splitlines()
                    if line.startswith("v ")
                ]
            )
            self.assertGreater(float(np.ptp(vertices[:, 2])), 0.030)
            collision = body.find(f"geom[@name='gripper_{finger}_collision_nail']")
            assert collision is not None
            self.assertEqual(collision.attrib["type"], "box")
            tip = body.find(f"geom[@name='gripper_{finger}_collision_nail_tip']")
            assert tip is not None
            self.assertEqual(tip.attrib["type"], "capsule")

    def test_approved_nail_visuals_are_unchanged(self) -> None:
        manifest = json.loads(
            (REPOSITORY / "dyna_claw/review/approved_nail_visuals.json").read_text()
        )
        for filename, expected in manifest["files"].items():
            with self.subTest(filename=filename):
                contents = (REPOSITORY / "dyna_claw" / filename).read_bytes()
                self.assertEqual(hashlib.sha256(contents).hexdigest(), expected)

    def test_zero_is_nail_tangency_for_each_gripper(self) -> None:
        for filename, prefixes in (
            ("dyna_claw/models/dynaclaw.xml", ("",)),
            ("dyna_taku/taku.xml", ("left_", "right_")),
        ):
            model = MjModel.from_xml_path(str(REPOSITORY / filename))
            for prefix in prefixes:
                for opening in (0.0, 0.00001, -0.00001):
                    with self.subTest(model=filename, gripper=prefix, opening=opening):
                        data = MjData(model)
                        if prefix:
                            mj_resetDataKeyframe(model, data, 0)
                        for finger, position in (
                            ("front", -opening),
                            ("rear", opening),
                        ):
                            joint = model.joint(f"{prefix}gripper_{finger}_joint")
                            data.qpos[joint.qposadr[0]] = position
                        mj_forward(model, data)
                        front = model.geom(
                            f"{prefix}gripper_front_collision_nail_tip"
                        ).id
                        rear = model.geom(f"{prefix}gripper_rear_collision_nail_tip").id
                        distance = mj_geomDistance(model, data, front, rear, 1.0, None)
                        self.assertAlmostEqual(distance, 2 * opening, delta=1e-10)
                        finger_bodies = {
                            model.body(f"{prefix}gripper_{finger}_link").id
                            for finger in ("front", "rear")
                        }
                        contacts = [
                            contact
                            for contact in data.contact
                            if model.geom_bodyid[contact.geom1] in finger_bodies
                            and model.geom_bodyid[contact.geom2] in finger_bodies
                        ]
                        if opening >= 0:
                            self.assertEqual(contacts, [])
                        else:
                            self.assertGreater(len(contacts), 0)

    def test_generated_includes_are_current(self) -> None:
        for filename, expected in generate(REPOSITORY).items():
            with self.subTest(filename=filename):
                actual = (REPOSITORY / "dyna_taku/gripper" / filename).read_text()
                self.assertEqual(actual, expected)

    def test_open_fingers_have_clearance(self) -> None:
        model = MjModel.from_xml_path(str(REPOSITORY / "dyna_claw/models/dynaclaw.xml"))
        self.assertEqual(model.nq, 2)
        self.assertEqual(model.nu, 1)
        for opening in (0.0, 0.000001, 0.00001, 0.001, 0.002, 0.01, 0.02, 0.0345):
            with self.subTest(opening=opening):
                data = MjData(model)
                data.qpos[:] = [-opening, opening]
                mj_forward(model, data)
                self.assertEqual(data.ncon, 0)
                np.testing.assert_allclose(data.efc_pos, 0, atol=1e-12)

    def test_taku_instances_match_standalone_geometry(self) -> None:
        standalone = MjModel.from_xml_path(
            str(REPOSITORY / "dyna_claw/models/dynaclaw.xml")
        )
        taku = MjModel.from_xml_path(str(REPOSITORY / "dyna_taku/taku.xml"))
        for opening in (0.0, 0.01, 0.0345):
            reference = MjData(standalone)
            reference.qpos[:] = [-opening, opening]
            mj_forward(standalone, reference)
            data = MjData(taku)
            mj_resetDataKeyframe(taku, data, 0)
            for side in ("left", "right"):
                for finger, position in (("front", -opening), ("rear", opening)):
                    joint = taku.joint(f"{side}_gripper_{finger}_joint")
                    data.qpos[joint.qposadr[0]] = position
            mj_forward(taku, data)
            for side in ("left", "right"):
                wrist = taku.body(f"{side}_wrist_roll_link").id
                rotation = data.xmat[wrist].reshape(3, 3)
                for index in range(standalone.ngeom):
                    name = mj_id2name(standalone, mjtObj.mjOBJ_GEOM, index)
                    self.assertIsNotNone(name)
                    instance = taku.geom(f"{side}_{name}").id
                    with self.subTest(side=side, opening=opening, geom=name):
                        local_position = rotation.T @ (
                            data.geom_xpos[instance] - data.xpos[wrist]
                        )
                        local_rotation = rotation.T @ data.geom_xmat[instance].reshape(
                            3, 3
                        )
                        np.testing.assert_allclose(
                            local_position, reference.geom_xpos[index], atol=1e-10
                        )
                        np.testing.assert_allclose(
                            local_rotation,
                            reference.geom_xmat[index].reshape(3, 3),
                            atol=1e-10,
                        )
                        self.assertEqual(
                            taku.geom_type[instance], standalone.geom_type[index]
                        )
                        self.assertEqual(
                            taku.geom_condim[instance], standalone.geom_condim[index]
                        )
                        np.testing.assert_array_equal(
                            taku.geom_size[instance], standalone.geom_size[index]
                        )


if __name__ == "__main__":
    unittest.main()
