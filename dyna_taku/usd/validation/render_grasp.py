"""Render recorded simulation states, without rerunning or attaching the object.

Use Isaac's Python for --engine isaac; a MuJoCo Python environment otherwise.
The companion compare_grasps.py creates the .npz trajectories.
"""

import argparse
import os
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--work-dir", type=Path, required=True)
parser.add_argument("--engine", choices=["isaac", "mujoco_cpu", "mujoco_warp"], required=True)
parser.add_argument("--case", default="cylinder_center")
parser.add_argument("--dt", type=float, default=0.0005)
parser.add_argument("--gpu", type=int, default=0)
args = parser.parse_args()
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MUJOCO_EGL_DEVICE_ID", str(args.gpu))
if args.engine == "isaac":
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "active_gpu": args.gpu,
            "multi_gpu": False,
            "disable_viewport_updates": True,
        }
    )
import numpy as np
import mujoco as mj
from PIL import Image

r = args.work_dir.resolve()
stem = f"{args.case}_{args.dt}"
record = np.load(r / "isaac62_results" / (stem + "_poses.npz"))
engine = list(record["engines"]).index(args.engine)
import xml.etree.ElementTree as ET

scene_xml = ET.parse(r / "contact_results" / (stem + "_mjcf") / "scene.xml").getroot()
ET.SubElement(
    scene_xml.find("asset"),
    "texture",
    name="review_background",
    type="skybox",
    builtin="flat",
    rgb1=".85 .85 .85",
    width="32",
    height="192",
)
model = mj.MjModel.from_xml_string(ET.tostring(scene_xml, encoding="unicode"))
data = mj.MjData(model)
out = r / "comparison_frames" / args.engine
out.mkdir(parents=True, exist_ok=True)
eye = np.array([-0.4, -0.5, 0.45])
aim = np.array([-0.028, 0, 0.35])
if args.engine == "isaac":
    import omni.usd
    import omni.replicator.core as rep
    from pxr import UsdGeom, UsdLux, UsdShade, Sdf, Gf

    omni.usd.get_context().open_stage(
        str(r / "contact_results" / (stem + "_usd") / "usd/compat.usda")
    )
    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    paths = {p.GetName(): p.GetPath() for p in stage.Traverse()}
    # Physics-only bindings have no surface shader. Restore the simple props'
    # source display colors in this render-only session, without stepping physics.
    for name in ["support", "object_geom"]:
        prim = stage.GetPrimAtPath(paths[name])
        rgb = np.array(prim.GetAttribute("primvars:displayColor").Get()[0])
        rgb = np.where(rgb <= 0.04045, rgb / 12.92, ((rgb + 0.055) / 1.055) ** 2.4)
        material = UsdShade.Material.Define(stage, "/Review/Materials/" + name)
        shader = UsdShade.Shader.Define(stage, str(material.GetPath()) + "/Surface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb.tolist()))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
    for i in range(1, model.nbody):
        xf = UsdGeom.Xformable(stage.GetPrimAtPath(paths[model.body(i).name]))
        xf.ClearXformOpOrder()
        xf.SetResetXformStack(False)
        xf.AddTranslateOp(opSuffix="replay")
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble, opSuffix="replay")
    light = UsdLux.DomeLight.Define(stage, "/Review/Fill")
    light.CreateIntensityAttr(1000)
    light = UsdLux.RectLight.Define(stage, "/Review/Key")
    light.CreateIntensityAttr(1500)
    light.CreateWidthAttr(2)
    light.CreateHeightAttr(2)
    light.AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(Gf.Vec3d(-1, -1, 2), Gf.Vec3d(*aim), Gf.Vec3d(0, 0, 1)).GetInverse()
    )
    camera = UsdGeom.Camera.Define(stage, "/Review/Camera")
    camera.AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*aim), Gf.Vec3d(0, 0, 1)).GetInverse()
    )
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.001, 100))
    camera.CreateFocalLengthAttr(50)
    camera.CreateHorizontalApertureAttr(36)
    camera.CreateVerticalApertureAttr(27)
    for _ in range(20):
        app.update()
    rep.orchestrator.set_capture_on_play(False)
    product = rep.create.render_product("/Review/Camera", (960, 720))
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach(product)
else:
    import math

    model.vis.global_.offwidth = 960
    model.vis.global_.offheight = 720
    model.vis.global_.fovy = math.degrees(2 * math.atan(27 / 100))
    model.vis.headlight.ambient[:] = 0.35
    model.vis.headlight.diffuse[:] = 0.3
    camera = mj.MjvCamera()
    v = eye - aim
    camera.lookat[:] = aim
    camera.distance = np.linalg.norm(v)
    camera.azimuth = 180 + math.degrees(math.atan2(v[1], v[0]))
    camera.elevation = -math.degrees(math.atan2(v[2], np.linalg.norm(v[:2])))
    opt = mj.MjvOption()
    opt.geomgroup[3:] = 0
    renderer = mj.Renderer(model, 720, 960)
for frame in range(150):
    idx = int(np.argmin(abs(record["times"] - frame / 30)))
    data.qpos[:] = record["qpos"][idx, engine]
    mj.mj_forward(model, data)
    if args.engine == "isaac":
        for i in range(1, model.nbody):
            parent = model.body_parentid[i]
            rotation = data.xmat[parent].reshape(3, 3)
            pos = rotation.T @ (data.xpos[i] - data.xpos[parent])
            q = np.empty(4)
            inv = data.xquat[parent].copy()
            inv[1:] *= -1
            mj.mju_mulQuat(q, inv, data.xquat[i])
            ops = UsdGeom.Xformable(
                stage.GetPrimAtPath(paths[model.body(i).name])
            ).GetOrderedXformOps()
            ops[0].Set(Gf.Vec3d(*pos))
            ops[1].Set(Gf.Quatd(float(q[0]), Gf.Vec3d(*q[1:])))
        rep.orchestrator.step(rt_subframes=4, delta_time=0, pause_timeline=False)
        rgb = np.asarray(annot.get_data())[:, :, :3]
    else:
        renderer.update_scene(data, camera, opt)
        rgb = renderer.render()
    Image.fromarray(rgb).save(out / f"{frame:04d}.png")
    if frame % 30 == 0:
        print("FRAME", args.engine, frame, flush=True)
if args.engine == "isaac":
    rep.orchestrator.stop()
    app.close()
else:
    renderer.close()
