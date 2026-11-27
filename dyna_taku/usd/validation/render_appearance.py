"""Render identical home pose/viewpoints; renderer-specific lights, unchanged materials."""

import argparse
import os
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--engine", choices=["isaac", "mujoco"], required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--gpu", type=int, default=0)
a = parser.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MUJOCO_EGL_DEVICE_ID", str(a.gpu))
root = Path(__file__).resolve().parents[1]
views = [
    ("front", (3, 0, 1.3), (0, 0, 0.7)),
    ("three_quarter", (2.5, -2, 1.5), (0, 0, 0.7)),
    ("gripper", (1.5, -1.4, 0.9), (0.1, -0.25, 0.75)),
]
if a.engine == "isaac":
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "renderer": "RaytracedLighting",
            "multi_gpu": False,
            "active_gpu": a.gpu,
            "disable_viewport_updates": True,
        }
    )
import numpy as np
from PIL import Image

if a.engine == "isaac":
    import omni.usd, omni.replicator.core as rep
    from pxr import UsdGeom, Gf

    omni.usd.get_context().open_stage(str(root / "scene.usda"))
    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())
    camera = UsdGeom.Camera(stage.GetPrimAtPath("/Studio/Camera"))
    op = camera.GetOrderedXformOps()[0]
    for _ in range(30):
        app.update()
    rep.orchestrator.set_capture_on_play(False)
    product = rep.create.render_product("/Studio/Camera", (1600, 1600))
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach(product)
    for name, eye, aim in views:
        op.Set(
            Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*aim), Gf.Vec3d(0, 0, 1)).GetInverse()
        )
        for _ in range(8):
            app.update()
        rep.orchestrator.step(rt_subframes=32, delta_time=0, pause_timeline=False)
        Image.fromarray(np.asarray(annot.get_data())[:, :, :3]).save(a.output / f"{name}.png")
        print("RENDERED", name, flush=True)
    rep.orchestrator.stop()
    app.close()
else:
    import math, mujoco as mj

    model = mj.MjModel.from_xml_path(str(root.parent / "scene.xml"))
    data = mj.MjData(model)
    mj.mj_resetDataKeyframe(model, data, 0)
    mj.mj_forward(model, data)
    model.vis.global_.offwidth = 1600
    model.vis.global_.offheight = 1600
    model.vis.global_.fovy = math.degrees(2 * math.atan(36 / 100))
    model.vis.headlight.ambient[:] = 0.4
    model.vis.headlight.diffuse[:] = 0.3
    model.vis.headlight.specular[:] = 0.05
    model.light_diffuse[:] *= 0.65
    model.light_specular[:] *= 0.3
    opt = mj.MjvOption()
    opt.geomgroup[3:] = 0
    with mj.Renderer(model, 1600, 1600) as renderer:
        for name, eye, aim in views:
            v = np.array(eye) - aim
            cam = mj.MjvCamera()
            cam.lookat[:] = aim
            cam.distance = np.linalg.norm(v)
            cam.azimuth = 180 + math.degrees(math.atan2(v[1], v[0]))
            cam.elevation = -math.degrees(math.atan2(v[2], np.linalg.norm(v[:2])))
            renderer.update_scene(data, cam, opt)
            Image.fromarray(renderer.render()).save(a.output / f"{name}.png")
            print("RENDERED", name, flush=True)
