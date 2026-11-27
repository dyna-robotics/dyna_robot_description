"""Author a collision-only overlay from compiled MuJoCo support vertices.

Run in the USD audit environment. Existing transforms and visual meshes remain.
"""

import argparse, json
from pathlib import Path
import numpy as np
from pxr import Usd, UsdGeom, Sdf, Gf, Vt

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--hulls", type=Path, required=True)
p.add_argument("--asset", type=Path, default=Path(__file__).resolve().parents[1] / "taku.usda")
a = p.parse_args()
stage = Usd.Stage.Open(str(a.asset))
cache = UsdGeom.XformCache()
paths = {prim.GetName(): prim.GetPath() for prim in stage.Traverse()}
layer_path = a.asset.parent / "layers/collision_hulls.usda"
layer = Sdf.Layer.FindOrOpen(str(layer_path)) or Sdf.Layer.CreateNew(str(layer_path))
layer.Clear()
# Compute local coordinates before authoring the stronger geometry opinions.
items = []
for item in json.loads(a.hulls.read_text())["meshes"]:
    path = paths[item["geom"]]
    inverse = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path)).GetInverse()
    points = np.array([inverse.Transform(Gf.Vec3d(*v)) for v in item["world_points"]])
    items.append((path, points, np.array(item["triangles"])))
stage.GetRootLayer().subLayerPaths = ["layers/collision_hulls.usda"] + [
    v for v in stage.GetRootLayer().subLayerPaths if v != "layers/collision_hulls.usda"
]
stage.SetEditTarget(layer)
for path, points, faces in items:
    prim = stage.OverridePrim(path)
    prim.SetInstanceable(False)
    mesh = UsdGeom.Mesh(prim)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr([3] * len(faces))
    mesh.CreateFaceVertexIndicesAttr(faces.ravel().tolist())
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr([Gf.Vec3f(*points.min(axis=0)), Gf.Vec3f(*points.max(axis=0))])
    for name in ["normals", "primvars:st", "primvars:st:indices"]:
        attr = prim.GetAttribute(name)
        if attr:
            attr.Block()
layer.Save()
stage.GetRootLayer().Save()
print("Authored collision-only hull overlay:", len(items))
