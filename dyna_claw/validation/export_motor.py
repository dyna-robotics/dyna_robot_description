"""Export the existing USD motor at original scale for local validation only."""

import argparse, json
from pathlib import Path
import numpy as np
import trimesh
from pxr import Usd, UsdGeom, Gf

p = argparse.ArgumentParser()
p.add_argument("source", type=Path)
p.add_argument("output", type=Path)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
s = Usd.Stage.Open(str(a.source))
cache = UsdGeom.XformCache()
meshes = []
for prim in s.Traverse():
    if not prim.IsA(UsdGeom.Mesh):
        continue
    m = UsdGeom.Mesh(prim)
    xf = cache.GetLocalToWorldTransform(prim)
    verts = np.array(
        [xf.Transform(Gf.Vec3d(*v)) for v in m.GetPointsAttr().Get()]
    ) * UsdGeom.GetStageMetersPerUnit(s)
    counts = m.GetFaceVertexCountsAttr().Get()
    indices = m.GetFaceVertexIndicesAttr().Get()
    faces = []
    offset = 0
    for count in counts:
        poly = indices[offset : offset + count]
        faces.extend([[poly[0], poly[j], poly[j + 1]] for j in range(1, count - 1)])
        offset += count
    meshes.append(trimesh.Trimesh(verts, faces, process=False))
bounds = np.array([m.bounds for m in meshes])
lo = bounds[:, 0].min(axis=0)
hi = bounds[:, 1].max(axis=0)
center = (lo + hi) / 2
valid = []
for i, m in enumerate(meshes):
    m.vertices -= center
    m.export(a.output / f"part_{i:02d}.obj")
    if np.linalg.matrix_rank(m.vertices - m.vertices.mean(axis=0), tol=1e-8) == 3:
        valid.append(f"part_{i:02d}.obj")
trimesh.util.concatenate(meshes).export(a.output / "motor_visual.obj")
info = dict(
    source=str(a.source),
    dimensions=(hi - lo).tolist(),
    center=center.tolist(),
    parts=len(meshes),
    collision_parts=valid,
)
(a.output / "source.json").write_text(json.dumps(info, indent=2))
print(info)
