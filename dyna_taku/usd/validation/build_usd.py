"""Convert MJCF with a separate usd-exchange environment; author compatibility.

Run this script with MuJoCo 4.0.1 / usd-core 25.11. The converter executable
must belong to a different environment (see README: incompatible USD wheels).
"""

import argparse
from pathlib import Path
import subprocess
import shutil
import sys
import mujoco
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

REPO = Path(__file__).resolve().parents[3]
CASES = ["box_center", "cylinder_center", "thin_plate", "box_offset", "box_tilt", "tip_edge"]


def convert(source, output, converter):
    output.mkdir(parents=True, exist_ok=True)
    subprocess.run([converter, str(source), str(output)], check=True)
    roots = [p for p in output.glob("*.usda") if p.name != "compat.usda"]
    if len(roots) != 1:
        raise RuntimeError(f"Expected one converter root in {output}: {roots}")
    base = Usd.Stage.Open(str(roots[0]))
    path = output / "compat.usda"
    layer = Sdf.Layer.FindOrOpen(str(path))
    if layer:
        layer.Clear()
        stage = Usd.Stage.Open(layer)
    else:
        stage = Usd.Stage.CreateNew(str(path))
    stage.GetRootLayer().subLayerPaths = [roots[0].name]
    UsdGeom.SetStageUpAxis(stage, "Z")
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    stage.SetDefaultPrim(stage.GetPrimAtPath(base.GetDefaultPrim().GetPath()))
    for prim in base.Traverse():
        override = stage.OverridePrim(prim.GetPath())
        if prim.GetAttribute("newton:inertia").HasAuthoredValueOpinion():
            override.CreateAttribute("newton:inertia", Sdf.ValueTypeNames.DoubleArray).Block()
        binding = prim.GetRelationship("material:binding:physics")
        if binding and binding.GetTargets():
            override.CreateRelationship("material:binding").SetTargets(binding.GetTargets())
        if prim.GetTypeName() in ["PhysicsRevoluteJoint", "PhysicsPrismaticJoint"]:
            if not prim.GetAttribute("newton:armature").HasAuthoredValue():
                override.CreateAttribute("newton:armature", Sdf.ValueTypeNames.Float).Set(0.0)
            targets = prim.GetRelationship("physics:body0").GetTargets()
            if targets and not base.GetPrimAtPath(targets[0]).HasAPI(UsdPhysics.RigidBodyAPI):
                # A joint attached to the world must not target the non-rigid asset root.
                override.CreateRelationship("physics:body0").SetTargets([])
    model = mujoco.MjModel.from_xml_path(str(source))
    scene = stage.OverridePrim("/PhysicsScene")
    scene.CreateAttribute("mjc:option:timestep", Sdf.ValueTypeNames.Double).Set(model.opt.timestep)
    scene.CreateAttribute("newton:timeStepsPerSecond", Sdf.ValueTypeNames.Float).Set(
        1 / model.opt.timestep
    )
    for i in range(model.nkey):
        key = stage.DefinePrim(
            str(base.GetDefaultPrim().GetPath()) + "/Keyframes/" + model.key(i).name, "MjcKeyframe"
        )
        for field, values in [
            ("qpos", model.key_qpos),
            ("qvel", model.key_qvel),
            ("ctrl", model.key_ctrl),
        ]:
            key.CreateAttribute("mjc:" + field, Sdf.ValueTypeNames.DoubleArray).Set(
                values[i].tolist()
            )
    stage.GetRootLayer().Save()
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--converter", required=True, help="mujoco_usd_converter executable from separate env"
    )
    parser.add_argument(
        "--output", type=Path, required=True, help="Scratch directory, outside tracked assets"
    )
    parser.add_argument("--fixtures", action="store_true")
    parser.add_argument(
        "--install-robot",
        action="store_true",
        help="Install generated robot layers into this checkout; reapply compiled hulls afterward",
    )
    args = parser.parse_args()
    if not args.fixtures:
        compat = convert(REPO / "dyna_taku/scene.xml", args.output, args.converter)
        print(compat)
        if args.install_robot:
            dest = REPO / "dyna_taku/usd"
            layers = dest / "layers"
            layers.mkdir(parents=True, exist_ok=True)
            shutil.copytree(args.output / "Payload", layers / "Payload", dirs_exist_ok=True)
            layer = Sdf.Layer.FindOrOpen(str(compat))
            base = args.output / layer.subLayerPaths[0]
            shutil.copy2(base, layers / "base.usda")
            installed = Sdf.Layer.CreateAnonymous()
            installed.ImportFromString(layer.ExportToString())
            installed.subLayerPaths = ["layers/base.usda"]
            installed.Export(str(dest / "taku.usda"))
            print(
                "Installed robot layers. Run export_collision_hulls.py and apply_collision_hulls.py next."
            )
        return
    sys.path.insert(0, str(REPO))
    from dyna_claw.validation.run_grasp_validation import CASES as definitions, run

    for dt in [0.001, 0.0005]:
        for case in definitions:
            if case["name"] not in CASES:
                continue
            stem = f"{case['name']}_{dt}"
            mjcf = args.output / "contact_results" / (stem + "_mjcf")
            run(case, dt, mjcf)
            print(
                convert(
                    mjcf / "scene.xml",
                    args.output / "contact_results" / (stem + "_usd") / "usd",
                    args.converter,
                )
            )


if __name__ == "__main__":
    main()
