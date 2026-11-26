# Changelog

## [2026-11-26]

- Model compliant, high-friction gripper pads separately from metal nails; reduce contact penetration and holding drift.
- Add reproducible grasp validation and dynamic regression checks.
- Refine soft-touch paint colors and AGV details; set the head to HSB(34, 25, 89).
- Split the AGV collision shell into four hulls to preserve the central U-shaped clearance.
- Update the preview with an upright greeting pose and balanced lighting.

## 2026-11-25

- Remove internal head yaw/pitch collision geoms with no corresponding visuals;
  retain the outer head collision and existing joint dynamics.

- Split each gripper housing collision into two convex hulls for the angled
  front section and rear motor housing, preserving the recess between them.

- Extract a standalone, side-neutral DynaClaw finger model and generate Taku's
  left/right instances without changing joint names or dynamics.
- Replace 16 segmented collision meshes per finger with 10 geometries: native
  box faces, a coarse shoulder taper, inner relief/support wedges, and a nail
  box with a distal capsule.
- Calibrate both grippers so zero travel places the nail capsules at tangency
  without contact; move visuals, collision geoms, and tip frames together.
- Add aluminum nail visuals with straight 12-sided holes, hard-edge normals,
  and filled inner tips; protect the accepted meshes with regression fixtures.
- Convert the remaining STL collision meshes to OBJ, preserving triangle geometry
  and winding, for compatibility with Drake's MJCF importer.

## 2026-11-05

- Refine gripper collision geometry and enable torsional friction.

## 2026-11-03

- Simplify mesh and update surface quality.
- Restore collision.
