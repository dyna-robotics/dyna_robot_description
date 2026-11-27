"""Verify the actual Isaac-generated MuJoCo collider support surfaces at rest."""

import argparse, json, re
from pathlib import Path
import mujoco as mj
import numpy as np

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--runtime-model", type=Path, required=True)
p.add_argument("--compiled-hulls", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
model = mj.MjModel.from_xml_path(str(a.runtime_model))
data = mj.MjData(model)
mj.mj_forward(model, data)
reference = json.loads(a.compiled_hulls.read_text())
rng = np.random.default_rng(123)
directions = np.r_[np.eye(3), -np.eye(3), rng.normal(size=(1024, 3))]
directions /= np.linalg.norm(directions, axis=1)[:, None]
rows = []
for item in reference["meshes"]:
    g = next(
        g
        for g in range(model.ngeom)
        if re.sub(r"_\d+$", "", model.geom(g).name.split("/")[-1]) == item["geom"]
    )
    mesh = model.geom_dataid[g]
    start = model.mesh_vertadr[mesh]
    count = model.mesh_vertnum[mesh]
    points = (
        model.mesh_vert[start : start + count] @ data.geom_xmat[g].reshape(3, 3).T
        + data.geom_xpos[g]
    )
    source = np.array(item["world_points"])
    error = float(
        np.max(abs(np.max(points @ directions.T, axis=0) - np.max(source @ directions.T, axis=0)))
    )
    rows.append(
        {
            "geom": item["geom"],
            "source_hull_vertices": item["hull_vertices"],
            "imported_vertices": int(count),
            "max_support_error_mm": error * 1000,
        }
    )
result = {
    "mujoco_version": mj.__version__,
    "directions": len(directions),
    "colliders": rows,
    "max_support_error_mm": max(x["max_support_error_mm"] for x in rows),
}
a.output.parent.mkdir(parents=True, exist_ok=True)
a.output.write_text(json.dumps(result, indent=2) + "\n")
print("Maximum collision support error (mm):", result["max_support_error_mm"])
assert result["max_support_error_mm"] < 0.01
