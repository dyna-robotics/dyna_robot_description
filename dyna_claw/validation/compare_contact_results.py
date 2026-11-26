"""Compare measured before/after contact results; preserve both raw result sets."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

p = argparse.ArgumentParser()
p.add_argument("before", type=Path)
p.add_argument("after", type=Path)
a = p.parse_args()
fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
for name, title, ax, column in [
    ("thin_plate", "3 mm plate: contact penetration", axes[0, 0], "penetration"),
    ("motor_200g_long_hold", "200 g motor: long hold", axes[0, 1], "z"),
    ("motor_500g", "500 g motor: short hold", axes[1, 0], "z"),
    ("seam_slide", "Forced contact transition", axes[1, 1], "z"),
]:
    for folder, label in [
        (a.before, "Before"),
        (a.after, "Compliant pad + contact solver"),
    ]:
        data = np.genfromtxt(
            folder / f"{name}_0.001" / "trajectory.csv", delimiter=",", names=True
        )
        y = data[column] * 1000
        if column == "z":
            y -= y[0]
        ax.plot(data["time"], y, label=label, linewidth=1.5)
    ax.set_title(title)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(
        "Penetration (mm)" if column == "penetration" else "Height change (mm)"
    )
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)
fig.savefig(a.after / "before_after.png", dpi=170)
rows = []
for path in sorted(a.after.glob("*/summary.json")):
    old = a.before / path.parent.name / "summary.json"
    if not old.exists():
        continue
    rows.append(
        dict(
            case=path.parent.name,
            before=json.loads(old.read_text()),
            after=json.loads(path.read_text()),
        )
    )
(a.after / "before_after.json").write_text(json.dumps(rows, indent=2))
print("Compared", len(rows), "runs")
