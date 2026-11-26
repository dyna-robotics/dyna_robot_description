"""Summarize measured results without treating lift success as stable grasp."""

import argparse, json, hashlib, subprocess
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

p = argparse.ArgumentParser()
p.add_argument("results", type=Path)
a = p.parse_args()
root = a.results
summaries = [json.loads(f.read_text()) for f in sorted(root.glob("*/summary.json"))]
standalone = [s for s in summaries if "case" in s]
robot = [s for s in summaries if "side" in s]
fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
for name, label, axis, col, scale in [
    ("motor_200g_long_hold", "200 g motor: long hold", axes[0, 0], "z", 1000),
    ("seam_slide", "Forced seam crossing", axes[0, 1], "z", 1000),
    ("thin_plate", "3 mm plate: model contact", axes[1, 0], "penetration", 1000),
    (
        "thin_plate_stiff_contact",
        "3 mm plate: uniform-contact diagnostic",
        axes[1, 1],
        "penetration",
        1000,
    ),
]:
    for dt in [0.001, 0.0005, 0.00025, 0.000125]:
        f = root / f"{name}_{dt:g}" / "trajectory.csv"
        if not f.exists():
            continue
        d = np.genfromtxt(f, delimiter=",", names=True)
        y = d[col] * scale
        if col == "z":
            y = y - y[0]
        axis.plot(d["time"], y, label=f"{dt*1000:g} ms", linewidth=1.3)
    axis.set_title(label)
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Height change (mm)" if col == "z" else "Penetration (mm)")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=8)
fig.savefig(root / "contact_diagnostics.png", dpi=170)
plt.close(fig)
lines = [
    "# DynaClaw 夹持验证记录",
    "",
    "This report summarizes the supplied result files. It does not modify the production model or establish hardware acceptance.",
    "",
    "## 范围与方法",
    "",
    "MuJoCo 4.0.1；标准步长 1/0.5 ms，异常案例追加 0.25/0.125 ms。基础几何检查 6/6 通过。对象始终为自由刚体，执行器闭合和提起，没有把物体绑定到夹爪。电机采用现有 USD 原始尺度及分部件凸包，200/500 g 质量与惯量分配为测试假设。",
    "",
    "## 独立夹爪结果",
    "",
    "| 案例 | 步长 ms | 峰值穿透 mm | 保持段下滑 mm | 中位提起 mm | 警告 |",
    "|---|---:|---:|---:|---:|---:|",
]
for s in standalone:
    lines.append(
        f'| {s["case"]["name"]} | {s["dt"]*1000:g} | {s["max_penetration_mm"]:.3f} | {-s["hold_z_drift_mm"]:.3f} | {s["hold_lift_m"]*1000:.2f} | {s["warnings"]} |'
    )
lines += [
    "",
    "短保持窗口为 2.6–3.9 s；长保持为 2.6–9.9 s。下滑符号为世界高度下降；需要同时看接触和运动轨迹。",
    "",
    "## 整机诊断（不代表独立测试已全部通过）",
    "",
    "| 手 | 重力前馈 | 步长 ms | 中位提起 mm | 腕壳接触数峰值 | 自接触穿透 mm | 警告 |",
    "|---|---|---:|---:|---:|---:|---:|",
]
for s in robot:
    lines.append(
        f'| {s["side"]} | {s["gravity_feedforward"]} | {s["dt"]*1000:g} | {s["hold_lift_m"]*1000:.2f} | {s["max_housing_contacts"]} | {s["max_self_penetration_mm"]:.3f} | {s["warnings"]} |'
    )
lines += [
    "",
    "## 结论与下一步",
    "",
    "Interpret retention, drift, penetration, release, and timestep sensitivity separately. A successful lift or absence of solver warnings does not establish stable grasping.",
    "The motor mass/inertia and friction are assumptions; peak contact forces are not calibrated hardware predictions.",
    "Compare matching baseline and updated runs with compare_contact_results.py. See VALIDATION_2026-11-26.md for the measured before/after findings and known limitations.",
    "",
    "## 文件",
    "",
    "- `*/scene.xml`：案例场景，含 initial keyframe；动力学动作需运行脚本。",
    "- `*/trajectory.csv`：位置、四元数、速度、执行器力、穿透、接触和警告。",
    "- 独立测试的 `contact_forces.csv`：各接触局部坐标系下六维力/力矩及接触坐标系。",
    "- `contact_pairs.json`：接触几何对和时间；整机包含负距离。",
    "- `*/frame_*.png`：检查图片。",
    "- `../source_manifest.json`：模型与测试脚本哈希，记录本地配色版本。",
    "",
    "复现步骤和假设见仓库 `dyna_claw/validation/README.md`。初步 3.3.7 结果保留在父目录，本报告仅使用此目录中的 4.0.1 结果。",
    "",
    "![Contact diagnostics](contact_diagnostics.png)",
]
(root / "REPORT.md").write_text("\n".join(lines) + "\n")
(root / "all_results.json").write_text(json.dumps(summaries, indent=2))
print("Reported", len(standalone), "standalone runs and", len(robot), "full robot runs")
