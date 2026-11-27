"""Plot the measured trajectories and contact-screening results (no smoothing)."""

import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--results", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
rows = json.loads((a.results / "contact_results.json").read_text())
fig, axes = plt.subplots(1, 2, figsize=(13, 4), layout="constrained")
for dt, style in [(0.001, "-"), (0.0005, "--")]:
    data = np.loadtxt(a.results / f"cylinder_center_{dt}.csv", delimiter=",")
    warp = np.loadtxt(a.results / f"cylinder_center_{dt}_warp.csv", delimiter=",")
    for name, z, color in [
        ("Isaac Newton", data[:, 3], "#0072b2"),
        ("MuJoCo CPU", data[:, 6], "#d55e00"),
        ("MuJoCo Warp", warp[:, 2], "#009e73"),
    ]:
        axes[0].plot(
            data[:, 0],
            1000 * (z - z[0]),
            style,
            color=color,
            label=f"{name}, {dt * 1000:g} ms",
            lw=1.3,
        )
    axes[1].plot(
        data[:, 0],
        np.linalg.norm(data[:, 1:4] - data[:, 4:7], axis=1) * 1000,
        style,
        color="#d55e00",
        label=f"vs CPU, {dt * 1000:g} ms",
    )
    axes[1].plot(
        data[:, 0],
        np.linalg.norm(data[:, 1:4] - warp, axis=1) * 1000,
        style,
        color="#009e73",
        label=f"vs Warp, {dt * 1000:g} ms",
    )
for ax in axes:
    ax.axvline(4, color="gray", ls=":", label="Release begins" if ax == axes[0] else None)
    ax.set_xlabel("Simulation time (s)")
    ax.grid(alpha=0.2)
    ax.legend(fontsize=8)
axes[0].set_ylabel("Object lift (mm)")
axes[0].set_title("Cylinder: grasp → lift → hold → release")
axes[1].set_ylabel("Object position difference (mm)")
axes[1].set_title("Isaac difference, including free motion after release")
fig.savefig(a.output / "trajectories.png", dpi=160)
plt.close(fig)
fig, axes = plt.subplots(1, 2, figsize=(13, 4), layout="constrained")
subset = [r for r in rows if r["dt"] == 0.0005]
x = np.arange(len(subset))
labels = [r["case"].replace("_", " ") for r in subset]
for ax in axes:
    ax.set_xticks(x, labels, rotation=25, ha="right")
    ax.grid(axis="y", alpha=0.2)
axes[0].bar(x - 0.17, [r["max_before_release_vs_cpu_mm"] for r in subset], 0.34, label="vs CPU")
axes[0].bar(x + 0.17, [r["max_before_release_vs_warp_mm"] for r in subset], 0.34, label="vs Warp")
axes[0].set_ylabel("Maximum object position difference (mm)")
axes[0].set_title("Before release, 0.5 ms timestep")
axes[0].legend()
axes[1].bar(x, [r["sampled_max_penetration_mm"] for r in subset], color="#0072b2")
axes[1].axhline(0.5, color="#d55e00", ls="--", label="Screening limit")
axes[1].legend()
axes[1].set_title("Finger–object penetration (ground contacts excluded)")
axes[1].set_ylabel("Maximum sampled depth (mm)")
fig.savefig(a.output / "grasp_screening.png", dpi=160)
plt.close(fig)
