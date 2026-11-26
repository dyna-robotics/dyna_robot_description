# DynaClaw finger assembly

`models/dynaclaw.xml` is the editable, side-neutral model for Taku's two fingers,
including their mounting hardware, inertials, sliding joints, actuator, fingertip
frames, and opposite-motion equality constraint. Its attachment frame coincides
with Taku's wrist-roll frame. The wrist housing and wrist inertia remain in Taku;
this standalone model contains no housing or extra housing mass.

The package follows the standalone-model arrangement of the DynaClaw DVT bundle.
All meshes are OBJ. The visual geometry comes from Taku's former
`right_gripper_{front,rear}_link_surface_{1,2}.obj` files. Each rigid mesh is now
partitioned into the complete nail and the remaining hardware. The nail hole is
simplified to a 12-sided straight opening, and hard-edge face normals prevent
false countersink shading. The full nail uses an aluminum-gray material; other
rigid parts retain their black material. The corresponding left meshes had
identical vertices, so both hands share one set of assets.

## View and edit

With MuJoCo or Drake installed in your chosen development environment:

```bash
python -m mujoco.viewer --mjcf dyna_claw/models/dynaclaw.xml
python -m pydrake.visualization.model_visualizer -w dyna_claw/models/dynaclaw.xml
```

The `home` keyframe uses zero finger travel; `open` uses 20 mm of travel per
finger. In a kinematic viewer, set the front slider to the negative of the rear
slider. The simulated mechanism enforces this through an equality constraint.

After editing the standalone XML, regenerate Taku's include files:

```bash
python dyna_claw/generate_taku_includes.py
python dyna_claw/generate_taku_includes.py --check
python -m unittest dyna_claw.test_model -v
```

The generator uses only the Python standard library; the tests require MuJoCo
and NumPy. Generated files live in `dyna_taku/gripper/`. They preserve Taku's
existing `left_`/`right_` joint, body, actuator, and fingertip-frame names and are
included by `dyna_taku/taku.xml`. Edit the standalone model rather than these
generated files. The full robot's keyframes remain in `taku.xml`.

## Collision geometry

Each finger uses 10 geometries instead of the previous 16 segmented mesh hulls:

| Geometry | Representation | Purpose |
| --- | --- | --- |
| Taper | One convex mesh, 12 vertices | Broad shoulder narrowing from 40 mm to 12 mm |
| Outer back | Box, 0.8 mm thick | Outer face above the full-length nail |
| Inner pad | Box, 1.8 mm thick | Continuous flat inner tip contact face |
| Relief | Four convex prisms, 8 vertices each | Retain the concave relief without bridging it |
| Tip support | One convex prism, 8 vertices | Fill the tapered volume behind the inner pad |
| Nail | One box plus one transverse distal capsule | Constant-width blade with rounded distal tip; 0.992 mm capsule radius |

The rear finger reuses the same collision meshes with a 180-degree rotation
about local z. Both hands share the geometry. Collision geoms retain `condim=4`.

The narrow tip starts at local z = -42.883 mm. The relief ends at -57.111 mm;
the inner pad ends at -75.115 mm. The full nail spans approximately -50.291 to
-80.705 mm. The outer back-face box ends at -51.283 mm where the nail's outer plane
begins, avoiding duplicate planar contacts. Mount collision boxes are omitted
because those parts lie inside Taku's housing. The broad taper is deliberately coarse. The taper fills
the finger's internal rib cavities and approximates its curved lateral shoulder
with straight sides; it is not intended to reproduce those details.

The relief's sampled side-profile deviation from the visual mesh is at most
0.26 mm. Each nail now uses one box and one capsule across its distal tip. Long side
edges are not rounded. The capsule radius is 0.992 mm to match blade thickness;
the exported broad-face corner radius is approximately 2 mm, so this is an
approximation. All finger visuals, collision geoms, and fingertip frames are
shifted outward by 0.595308174533 mm per finger, so q=0 is nail-capsule tangency.
This includes a 1 pm outward numerical guard per finger: MuJoCo signed distance
is approximately 2e-12 m; Drake reports approximately 3.8e-11 m, with no finger
contacts/penetration. Joint definitions, travel, body frames, and inertias are
unchanged. Both Taku grippers are generated from this same source.
The approved nail OBJ bytes (geometry and hard-edge normals) are locked by
review/approved_nail_visuals.json and a regression test. Updating that fixture
requires explicit approval of a visual change. Tests cover zero tangency,
positive-opening clearance, and contact just beyond zero, for both Taku hands
and the standalone model.
The aluminum-gray nail visual has a straight 12-sided 3.6 mm screw hole without
a countersink (500 triangles per nail). Per-face normals preserve the hard
boundary between the hole wall and flat face.

The model and Taku integration were checked with MuJoCo 4.0.1 and Drake 1.58.0.
Mass properties, joint ordering, limits, damping, actuator parameters, coupling,
and Taku's home pose were preserved. Fewer geometries and native box faces reduce
representation complexity; no collision-performance benchmark has been run.

The inner-facing visual tip bevel is filled to the flat inner face. Rounded
corners, the straight screw hole, and hard-edge normals are preserved.
All collision geometry uses a shared display color; aluminum gray applies only
to the nail visual.

## Compliant pad contacts

Green elastomer geoms use a compliant-contact approximation with provisional
sliding friction 1.5; metal nails retain a separate, firmer contact law. This
is not a deforming rubber mesh. Both standalone and Taku models use elliptic
friction cones and `impratio=10` to reduce slow numerical slip. Geometry,
mass/inertia, actuator gains and force limits are unchanged. Contact values
need hardware calibration before interpreting loads quantitatively.

See [validation results and changes](validation/VALIDATION_2026-11-26.md) and
[reproduction instructions](validation/README.md).
