"""Validate portable USD composition, material colors and texture identity.

Run with MuJoCo 4.0.1 + usd-core 25.11 (separate from Isaac/converter).
"""

from pathlib import Path
import hashlib
import json
import numpy as np
import mujoco
from pxr import Usd, UsdShade, UsdUtils

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT.parent


def main():
    stage = Usd.Stage.Open(str(ROOT / "taku.usda"))
    assert stage and stage.GetDefaultPrim()
    _, _, unresolved = UsdUtils.ComputeAllDependencies(str(ROOT / "taku.usda"))
    assert not unresolved, unresolved
    model = mujoco.MjModel.from_xml_path(str(MODEL / "scene.xml"))
    converted = mujoco.MjSpec.from_file(str(ROOT / "taku.usda")).compile()
    counts = {
        name: [getattr(model, name), getattr(converted, name)]
        for name in ["nq", "nv", "nu", "neq", "nbody", "ngeom", "nexclude"]
    }
    assert all(a == b for a, b in counts.values()), counts
    colors = []
    for prim in stage.Traverse():
        if prim.GetTypeName() != "Material" or "/Materials/" not in str(prim.GetPath()):
            continue
        material = UsdShade.Material(prim)
        rgb = material.GetInput("color").Get()
        if rgb is None:
            continue
        source = model.mat_rgba[model.material(prim.GetName()).id, :3]
        expected = np.where(source <= 0.04045, source / 12.92, ((source + 0.055) / 1.055) ** 2.4)
        error = float(np.max(np.abs(np.array(rgb) - expected)))
        assert error < 2e-6, (prim.GetName(), error)
        colors.append(
            {
                "material": prim.GetName(),
                "source_srgb": source.tolist(),
                "usd_linear_rgb": list(rgb),
                "max_linear_error": error,
            }
        )
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    a = MODEL / "assets/textures/chest_albedo.png"
    b = ROOT / "layers/Payload/Textures/chest_albedo.png"
    assert digest(a) == digest(b), "Chest texture changed during conversion"
    result = {
        "unresolved_assets": [],
        "counts_source_usd": counts,
        "material_colors": colors,
        "texture_sha256": digest(a),
        "note": "Numeric color/texture preservation; PBR and OpenGL shading are not pixel-equivalent.",
    }
    (ROOT / "validation/results/asset.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
