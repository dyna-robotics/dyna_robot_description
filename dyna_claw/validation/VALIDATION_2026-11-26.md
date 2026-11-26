# DynaClaw contact update and validation — 2026-11-26

This record follows the five requested validation steps. The changes target `dyna_robot_description`; no Menagerie repository files were modified.

## Changes made

- Green elastomer contact geoms use `dynaclaw_soft_pad`: sliding friction **1.5**, torsional/rolling friction **0.005/0.0001**, `solref="0.01 1"`, `solimp="0.95 0.99 0.001"`, `condim=4` and `priority=1`.
- Metal nail box/capsule geoms use `dynaclaw_rigid_nail`: sliding friction **1.0** (unchanged), `solref="0.003 1"`, the same impedance, contact dimension and priority. Geometry and accepted nail visual meshes are unchanged.
- Standalone and Taku use **elliptic** friction cones, **impratio=10**, `implicitfast` and solver tolerance `1e-10`. NoSlip is not enabled. Higher tangential impedance reduces numerical creep without raising the metal friction coefficient or increasing actuator force limits.
- Collision defaults have `density=0`; existing explicit inertials remain authoritative. Compiled masses/inertias, joint ranges, actuator gains/force limits and collision transforms/sizes were checked against the pre-update model and are unchanged.
- Regenerated both Taku hand instances from the standalone source.
- Final head color is **HSB(34°,25%,89%)**, approximately RGB **227,202,170**. The earlier warm-white trial was superseded.
- The final AGV collision shell has four hulls preserving the central U-shaped clearance. The grasp measurements below were recorded before that chassis-only update; the existing geometry and dynamics regression suite was rerun afterward.

**Important modeling distinction:** this is a compliant-contact approximation of rubber pads carried by rigid finger links, not a deforming FEM/flex rubber mesh. There are no additional elastic degrees of freedom and no visible bulk deformation. Friction and compliance are provisional engineering values, not measured material identification. The improvement is due to the combined contact-law/solver changes; it must not be attributed solely to higher green-pad friction, since several pinch cases contact the metal nails.

## 1. Existing checks

The original six checks passed before and after the changes on MuJoCo 4.0.1: zero-opening tangency, positive-opening clearance, contact just beyond zero, approved nail visual hashes, generated-file synchronization, and standalone/left/right geometry consistency.

Two real-dynamics regression tests were added in `grasp_dynamics_test.py`:

- 3 mm plate: lift/hold/release, no warnings, peak penetration below 0.5 mm.
- Cylinder: lift/hold/release, no warnings, holding drift below 2 mm.

Both pass on the update and fail when run against the saved baseline. These are diagnostic thresholds, not hardware acceptance specifications.

## 2. Independent grasp scenes

The reusable runner creates a vertical test-stage mount, free objects with positive mass/inertia, and a support. It commands the original finger actuator through smooth closure and commands an 80 mm stage lift. No object weld, attachment, or per-step joint-position overwrite is used.

Objects: 50 g box, cylinder and 3 mm plate; motor geometry at its original USD dimensions (approximately 59 × 59 × 73 mm), with assumed 200 g and 500 g masses. Motor contacts use 16 nonplanar part-wise convex hulls. Four planar decorative parts are excluded. Inertias are inferred from the assumed mass distribution, not measured.

## 3. Difficult contact locations and motions

The matrix covers centered, 4 mm offset, 15-degree tilted, distal nail edge, lateral sliding and downward-loaded seam-transition cases. Standard episodes last 5 s. The long motor case releases at 10 s, with a 2.6–9.9 s holding assessment window.

Representative results at **1 ms**, MuJoCo **4.0.1**:

| Measurement | Baseline | Updated |
|---|---:|---:|
| 3 mm plate peak penetration | 1.134 mm | 0.151 mm |
| 3 mm plate holding drift | 3.755 mm | 0.022 mm |
| 200 g motor long-hold downward drift | 28.549 mm | 0.479 mm |
| 500 g motor short-hold downward drift | 15.221 mm | 0.143 mm |
| Seam-transition peak penetration | 2.889 mm | 0.373 mm |
| Seam-transition outcome | Loses contact before release | Holds until commanded release |

The updated matrix comprises 28 standalone runs (including the legacy uniform-contact diagnostic and finer-step cases) and four full-robot runs. The baseline retains 28 standalone and eight controller-comparison runs.

All standard updated cases at 1/0.5 ms retained objects through the assessed holding window, met the selected 2 mm drift screen and released; no MuJoCo warnings occurred. The seam case's achieved lift is about 59 mm because of its initial grasp/settling behavior, so it does **not** pass the separate 60 mm median-lift screen. Do not collapse these indicators into one blanket “pass.”

## 4. Recorded data and timestep refinement

Results include the generated MJCF with an `initial` keyframe; timestep-resolution object position/quaternion and velocity; finger opening/command; actuator force; penetration; normal force; active contact pairs; six-dimensional contact force/torque and contact frames; warning counts; and inspection renders.

The baseline and update are kept separately. Baseline refinements used 0.25/0.125 ms for seam and plate diagnostics. Updated plate and seam cases are also refined at these timesteps. Peak impact/contact forces remain sensitive to timestep even when holding drift converges; these forces are not calibrated load predictions. MuJoCo soft-contact penetration is not a measured rubber strain field.

Raw results directory used for this run:

- Baseline: `Downloads/simulation_material/taku_grasp_validation/mujoco_4.0.1/`
- Updated: `Downloads/simulation_material/taku_grasp_validation/soft_pad_final/`
- Saved pre-update XML and preliminary contact-only trials: `.../soft_pad_update/`

`before_after.json` and `before_after.png` compare matching runs. `source_manifest.json` records the updated input hashes. See `README.md` for commands; `compare_contact_results.py BASELINE UPDATED` rebuilds the comparison.

## 5. Whole-robot integration

Both full Taku hands were tested with the actual wrist housings and a free 200 g motor at 1/0.5 ms. The test-only controller uses gravity feedforward through position targets while retaining actuator force limits; the AGV base is welded for the fixture. Both hands lift and release. In these trajectories, no wrist-housing contacts or robot self-penetration were observed, and there were no warnings.

Gravity compensation is **not silently added to the production robot**: whether the actual robot provides onboard compensation needs confirmation. The earlier position-only diagnostic demonstrated sag/missed grasp and is retained as a controller comparison. This is not a collision sweep over all reachable poses or a hardware payload rating.

## References inspected on 2026-11-26

- [Menagerie Sharpa update #303, 2026-08-04](https://github.com/google-deepmind/mujoco_menagerie/commit/c1a4eeb85694ae1dffe33ff1797d4e528928a133): separates elastomer contact defaults and prevents collision geometry from double-counting mass. We adopt the distinction and mass hygiene, **not** Sharpa's robot-specific numerical compliance values.
- [Menagerie Spot arm update #358, 2026-11-02](https://github.com/google-deepmind/mujoco_menagerie/commit/b3435c4841c782ded22bd8e6f2a0b875df38cd75): actuator-level force limits and documented heuristic control parameters. Existing Taku limits are preserved.
- [Menagerie contributor guidance, updated 2026-11-24](https://github.com/google-deepmind/mujoco_menagerie/blob/eea4c7431b63/AGENTS.md): test physics beyond brief CI stepping, preserve identifiers, avoid mass double-counting, and use gravity compensation only if supported by the real hardware. Gallery changes belong to maintainers. No upstream files were edited.
- [MuJoCo 4.0 modeling: preventing slip](https://mujoco.readthedocs.io/en/4.0.0/modeling.html#preventing-slip): elliptic friction cones and increased `impratio` reduce slow regularization-driven slip. NoSlip has separate tradeoffs and is not enabled here.
- [MuJoCo flex gripper example](https://github.com/google-deepmind/mujoco/blob/main/model/flex/gripper.xml): reference for a future genuine deformable-pad model. It would require pad thickness, stiffness/damping, attachment and deformation validation; it is not represented by the present rigid-link contact approximation.
