# DynaClaw dynamic contact validation

The current contact update and its before/after findings are recorded in [VALIDATION_2026-11-26.md](VALIDATION_2026-11-26.md).

These are diagnostic experiments, not a hardware load rating or a blanket model acceptance test. They load the existing collision geometry, coupling, inertias and actuator limits. They do not modify the production XML or its colors.

## Reproduce

Use Python with `mujoco==4.0.1`, NumPy and Pillow; the regression checks also use pytest. The report generator additionally needs Matplotlib. The motor exporter additionally needs `usd-core` and `trimesh`. MuJoCo 3.3.7 was used for preliminary checks; the canonical report uses 4.0.1, matching the model README.

From the repository root:

```bash
python -m unittest dyna_claw.test_model -v
python -m pytest dyna_claw/validation/grasp_dynamics_test.py -q
python dyna_claw/generate_taku_includes.py --check

# Use an output directory outside the repository. Motor geometry is not redistributed.
python dyna_claw/validation/export_motor.py \
  '/path/to/motor.usd' \
  /path/to/results/motor_assets
python dyna_claw/validation/run_grasp_validation.py --output /path/to/results --render
python dyna_claw/validation/run_robot_validation.py --output /path/to/results
python dyna_claw/validation/run_robot_validation.py --output /path/to/results --gravity-feedforward --render
python dyna_claw/validation/make_report.py /path/to/results

# Refine anomalous cases rather than assuming absence of warnings means success.
python dyna_claw/validation/run_grasp_validation.py --output /path/to/results \
  --case seam_slide --dt .00025 .000125
python dyna_claw/validation/run_grasp_validation.py --output /path/to/results \
  --case thin_plate_stiff_contact --dt .00025 .000125
```

`--case` selects one standalone case. `--side left/right` selects a whole-robot hand. `--render` writes inspection stills with EGL; CPU dynamics runs need no GPU. Saved `scene.xml` files have an `initial` keyframe for inspection. Use the Python runner to execute the control sequence; opening the XML alone does not replay it.

## Experiment design

- Standalone mount is constrained to a servo-controlled vertical test stage. It holds position during closure, then commands an 80 mm lift. No lateral/rotational base motion is permitted. Actual stage compliance is not removed.
- All objects have a free joint and inferred positive inertia. They start on a support, close through the original position actuator, lift, hold and release. Joint positions are initialized once; they are never overwritten during dynamics. Objects are never welded or attached to the gripper.
- Closing begins at 0.3 s, finishes at 1.3 s. Lift runs from 1.7 to 2.5 s. Release starts at 4 s, or 10 s for the long-hold motor case. Commands use smooth interpolation.
- Primitives: 32 × 24 × 30 mm box, 24 mm diameter × 30 mm cylinder, and 32 × 3 × 30 mm thin plate; each has an assumed mass of 50 g. Offset is 4 mm and tilt is 15 degrees about vertical. Edge and seam tests use shorter boxes at different finger depths.
- `surface_slide` applies a sinusoidal 0.25 N lateral load from 2.6–3.3 s. `seam_slide` instead applies a downward load peaking at 2 N, forcing contact transitions. Slipping under imposed load is not by itself a geometry defect; timestep-dependent retention or deep penetration warrants investigation.
- The motor keeps original USD scale, approximately 59 × 59 × 73 mm. Each of 16 nonplanar parts contributes one convex collision mesh; four planar decorative parts are excluded. This is a **part-wise convex approximation**, not exact concave triangle contact. Assumed 200/500 g total mass is distributed equally among the parts and inertia inferred from those meshes. Neither mass distribution nor friction has been calibrated against hardware.
- The legacy-named `thin_plate_stiff_contact` diagnostic applies a uniform contact law (stiffer than the old baseline, but not necessarily stiffer than the updated metal nail). It changes `solref` to `0.005 1` on both interacting surfaces **in its generated scene only**. All other experiments retain source/default contact parameters.
- Whole robot keeps its original floating base and welds that base to the world for this fixture; this preserves parent/child collision filtering. Both arms start in reachable forward poses. The selected arm performs an 80 mm wrist lift with actuator commands. Wrist housings and all robot collision geoms remain present.
- Gravity-feedforward comparison adds `qfrc_bias / kp` to non-gripper, non-AGV position targets. It does not directly apply force to the object, modify actuator gains, or bypass actuator force limits. The baseline uses position targets alone.

## Outputs and interpretation

Every case writes its scene, summary and timestep-resolution trajectory CSV. Standalone cases additionally record active contact-pair names and all six contact-force/torque components with contact frames. Whole-robot cases record negative-distance contact pairs, housing contacts and self-contact penetration. Both record object pose/velocity, actuator force and warning counts; standalone cases also record opening and target command.

The short hold window is 2.6–3.9 s; the long window is 2.6–9.9 s. Peak speed over an entire episode includes the intentional release fall. Use the pre-release speed and time trace when looking for ejection.

`lifted`, `retained_until_release`, `released` and `stable_hold_2mm` are separate screening indicators, not certified tolerances. The 2 mm drift threshold is a chosen diagnostic threshold. A positive median lift can hide persistent slip, and a release drop cannot establish that the object was securely held first. The report must consider penetration relative to object thickness, slip, force history, active contact transitions and timestep sensitivity together.

Known limitations: a small deterministic suite, one motor geometry, assumed inertias/friction, no measured force calibration, no full reachability/collision sweep, no hardware controller validation. Failed standalone cases remain open even if a whole-robot diagnostic succeeds. Do not declare the complete model validated based only on zero MuJoCo warnings.
