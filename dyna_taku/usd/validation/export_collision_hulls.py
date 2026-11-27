"""Extract the actual MuJoCo-compiled convex hull, before another engine re-hulls it.

Run with the same MuJoCo version used for dynamics (3.12.0 for Isaac 6.2).
The mesh_graph vertex IDs are the support vertices used by MuJoCo collision.
"""

import argparse
import json
from pathlib import Path
import mujoco as mj
import numpy as np
from scipy.spatial import ConvexHull

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
a = parser.parse_args()
repo = Path(__file__).resolve().parents[3]
model = mj.MjModel.from_xml_path(str(repo / "dyna_taku/scene.xml"))
data = mj.MjData(model)
mj.mj_forward(model, data)
meshes = []
for g in range(model.ngeom):
    if model.geom_type[g] != mj.mjtGeom.mjGEOM_MESH or not (
        model.geom_contype[g] or model.geom_conaffinity[g]
    ):
        continue
    mesh = model.geom_dataid[g]
    adr = model.mesh_graphadr[mesh]
    if adr < 0:
        continue
    count = int(model.mesh_graph[adr])
    ids = model.mesh_graph[adr + 2 + count : adr + 2 + 2 * count]
    points = model.mesh_vert[model.mesh_vertadr[mesh] + ids]
    hull = ConvexHull(points)
    faces = hull.simplices.copy()
    for i, face in enumerate(faces):
        if (
            np.dot(
                np.cross(points[face[1]] - points[face[0]], points[face[2]] - points[face[0]]),
                hull.equations[i, :3],
            )
            < 0
        ):
            faces[i] = face[::-1]
    world = points @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
    meshes.append(
        {
            "geom": model.geom(g).name,
            "source_mesh_vertices": int(model.mesh_vertnum[mesh]),
            "hull_vertices": count,
            "world_points": world.tolist(),
            "triangles": faces.tolist(),
        }
    )
a.output.parent.mkdir(parents=True, exist_ok=True)
a.output.write_text(
    json.dumps({"mujoco_version": mj.__version__, "meshes": meshes}, indent=2) + "\n"
)
print("Exported compiled hulls:", len(meshes))
