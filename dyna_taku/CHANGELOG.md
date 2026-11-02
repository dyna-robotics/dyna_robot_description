# Changelog

## Initial release

- Added the dyna Taku MuJoCo model, visual meshes, materials and chest texture.
- Added a torso capsule and elbow, wrist and wheel sphere collisions, actuators, sensor frames and a home keyframe.

## Mesh storage optimization

- Deduplicated mesh attributes and compacted OBJ precision; model size reduced below 100 MB without removing triangles.

## Visual mesh simplification

- Reduced triangles in nine casing meshes while preserving open boundaries and transferring surface normals.
- Kept detailed head, elbow, wrist, gripper, wheel and textured chest meshes unchanged.
