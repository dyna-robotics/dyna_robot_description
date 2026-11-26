"""Real-dynamics regressions for thin-object penetration and holding drift."""

from pathlib import Path

from dyna_claw.validation.run_grasp_validation import run


def test_thin_plate_lifts_without_deep_penetration(tmp_path: Path) -> None:
    result = run(dict(name="thin_plate", shape="plate"), 0.001, tmp_path)
    assert result["finite"] and result["warnings"] == 0
    assert result["max_penetration_mm"] < 0.5
    assert result["retained_until_release"] and result["released"]


def test_cylinder_hold_drift_is_bounded(tmp_path: Path) -> None:
    result = run(dict(name="cylinder_center", shape="cylinder"), 0.001, tmp_path)
    assert result["finite"] and result["warnings"] == 0
    assert result["lifted"] and result["stable_hold_2mm"] and result["released"]
