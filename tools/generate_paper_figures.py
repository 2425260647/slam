#!/usr/bin/env python3
"""Generate reproducible figures for the annotated CEA manuscript draft."""

import csv
from pathlib import Path

import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "docs" / "paper_assets"

COLORS = {
    "navy": "#355C7D",
    "blue": "#6C9BCF",
    "green": "#79A873",
    "red": "#C85C5C",
    "gold": "#D7A84B",
    "gray": "#6F7782",
    "light": "#F5F7F8",
    "dark": "#20252B",
}


def configure_matplotlib():
    mpl.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": [
                "AR PL UMing CN",
                "DejaVu Sans",
            ],
            "axes.unicode_minus": False,
            "svg.fonttype": "none",
            "pdf.fonttype": 42,
            "font.size": 8,
            "axes.linewidth": 0.8,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "legend.frameon": False,
        }
    )


def save_publication_figure(fig, stem):
    OUTPUT.mkdir(parents=True, exist_ok=True)
    png_path = OUTPUT / f"{stem}.png"
    fig.savefig(png_path, dpi=300, bbox_inches="tight")
    fig.savefig(OUTPUT / f"{stem}.svg", bbox_inches="tight")
    fig.savefig(OUTPUT / f"{stem}.pdf", bbox_inches="tight")
    with Image.open(png_path) as image:
        image.save(
            OUTPUT / f"{stem}.tiff",
            format="TIFF",
            compression="tiff_lzw",
            dpi=(600, 600),
        )


def add_box(ax, x, y, width, height, text, color, fontsize=8, linewidth=1.1):
    box = FancyBboxPatch(
        (x, y),
        width,
        height,
        boxstyle="round,pad=0.015,rounding_size=0.02",
        facecolor="white",
        edgecolor=color,
        linewidth=linewidth,
    )
    ax.add_patch(box)
    ax.text(
        x + width / 2,
        y + height / 2,
        text,
        ha="center",
        va="center",
        color=COLORS["dark"],
        fontsize=fontsize,
        linespacing=1.25,
    )
    return box


def arrow(ax, start, end, color=None, style="-|>", linewidth=1.1):
    ax.annotate(
        "",
        xy=end,
        xytext=start,
        arrowprops={
            "arrowstyle": style,
            "color": color or COLORS["gray"],
            "lw": linewidth,
            "shrinkA": 2,
            "shrinkB": 2,
        },
    )


def generate_system_pipeline():
    """Schematic-led figure: which direction is weak, then which source is weak."""
    fig, ax = plt.subplots(figsize=(7.0, 4.25))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    ax.text(0.02, 0.95, "输入与二维投影", color=COLORS["navy"], weight="bold", fontsize=9)
    add_box(ax, 0.02, 0.73, 0.18, 0.13, "多线三维点云\n/velodyne_points", COLORS["blue"])
    add_box(ax, 0.02, 0.52, 0.18, 0.13, "高度/量程筛选\n二维 LaserScan", COLORS["blue"])
    add_box(ax, 0.02, 0.26, 0.18, 0.13, "轮式里程计\n/odom", COLORS["gold"])
    arrow(ax, (0.11, 0.73), (0.11, 0.65))

    ax.text(0.27, 0.95, "创新点一：方向退化", color=COLORS["navy"], weight="bold", fontsize=9)
    add_box(ax, 0.27, 0.73, 0.20, 0.13, "2×2 点云协方差\n特征值/特征向量", COLORS["green"])
    add_box(ax, 0.27, 0.52, 0.20, 0.13, "条件数 κ → Sigmoid\n低通平滑 Dconf", COLORS["green"])
    add_box(ax, 0.27, 0.31, 0.20, 0.13, "S=R diag(...) R^T\n各向异性平移先验", COLORS["green"], fontsize=7)
    arrow(ax, (0.20, 0.585), (0.27, 0.79))
    arrow(ax, (0.37, 0.73), (0.37, 0.65))
    arrow(ax, (0.37, 0.52), (0.37, 0.44))

    ax.text(0.54, 0.95, "创新点二：可靠性仲裁", color=COLORS["navy"], weight="bold", fontsize=9)
    add_box(ax, 0.54, 0.73, 0.20, 0.13, "扫描点数与角度覆盖\nLiDAR 质量 QL", COLORS["red"])
    add_box(ax, 0.54, 0.52, 0.20, 0.13, "ΔT=Todom^(-1) Tlidar\n横向/航向一致性残差", COLORS["red"], fontsize=7)
    add_box(ax, 0.54, 0.31, 0.20, 0.13, "可靠性门控+双阈值迟滞\n快降权、慢恢复", COLORS["red"])
    arrow(ax, (0.20, 0.585), (0.54, 0.79))
    arrow(ax, (0.20, 0.325), (0.54, 0.585), color=COLORS["gold"])
    arrow(ax, (0.47, 0.585), (0.54, 0.585), color=COLORS["green"])
    arrow(ax, (0.64, 0.73), (0.64, 0.65))
    arrow(ax, (0.64, 0.52), (0.64, 0.44))

    ax.text(0.81, 0.95, "优化与输出", color=COLORS["navy"], weight="bold", fontsize=9)
    add_box(ax, 0.81, 0.66, 0.17, 0.13, "Ceres 前端\n方向自适应匹配", COLORS["navy"])
    add_box(ax, 0.81, 0.43, 0.17, 0.13, "Pose Graph 后端\nodom 标量动态权重", COLORS["navy"])
    add_box(ax, 0.81, 0.20, 0.17, 0.13, "二维轨迹与\n占据栅格地图", COLORS["navy"])
    arrow(ax, (0.47, 0.375), (0.81, 0.725), color=COLORS["green"])
    arrow(ax, (0.74, 0.375), (0.81, 0.495), color=COLORS["red"])
    arrow(ax, (0.895, 0.66), (0.895, 0.56))
    arrow(ax, (0.895, 0.43), (0.895, 0.33))

    ax.text(
        0.50,
        0.06,
        "第一阶段回答“哪个方向退化”，第二阶段回答“观测冲突时降低哪一类约束”",
        ha="center",
        va="center",
        fontsize=8.5,
        color=COLORS["dark"],
        weight="bold",
    )
    save_publication_figure(fig, "system_pipeline")
    plt.close(fig)


def read_ros_csv(path):
    times = []
    values = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        next(reader, None)
        for row in reader:
            if len(row) < 2:
                continue
            try:
                times.append(int(row[0]) * 1e-9)
                values.append(float(row[1]))
            except ValueError:
                continue
    return times, values


def downsample(times, values, max_points=3000):
    if len(times) <= max_points:
        return times, values
    stride = max(1, len(times) // max_points)
    return times[::stride], values[::stride]


def generate_preliminary_timeline():
    """Diagnostic trace only; no fabricated injected interval is shaded."""
    run = ROOT / "paper_exp" / "innovation2_slip_adaptive_backend" / "runs" / "slip_proposed_adaptive"
    series = {
        "异常分数": read_ros_csv(run / "slip_metric.csv"),
        "LiDAR可靠性": read_ros_csv(run / "slip_lidar_reliability.csv"),
        "退化置信度": read_ros_csv(run / "degeneracy_metric.csv"),
        "odom权重比例": read_ros_csv(run / "odom_weight_scale.csv"),
    }
    starts = [t[0] for t, _ in series.values() if t]
    if not starts:
        return
    t0 = min(starts)

    fig, axes = plt.subplots(3, 1, figsize=(7.0, 4.7), sharex=True, gridspec_kw={"hspace": 0.12})
    plot_spec = [
        (axes[0], "异常分数", COLORS["red"], "一致性异常分数"),
        (axes[1], "LiDAR可靠性", COLORS["green"], "LiDAR可靠性"),
        (axes[1], "退化置信度", COLORS["blue"], "退化置信度"),
        (axes[2], "odom权重比例", COLORS["navy"], "odom权重比例"),
    ]
    for ax, key, color, label in plot_spec:
        times, values = downsample(*series[key])
        times = [t - t0 for t in times]
        ax.plot(times, values, color=color, linewidth=0.9, label=label)
        ax.grid(axis="y", color="#D9DDE2", linewidth=0.5, alpha=0.8)
        ax.legend(loc="upper right", ncol=2, fontsize=7)

    axes[0].axhline(0.03, color=COLORS["gray"], linestyle="--", linewidth=0.8, label="候选高阈值")
    axes[0].set_ylabel("分数")
    axes[1].set_ylabel("置信度")
    axes[1].set_ylim(-0.03, 1.03)
    axes[2].set_ylabel("比例")
    axes[2].set_ylim(0.0, 1.05)
    axes[2].set_xlabel("记录时间/s")
    fig.suptitle("创新点二现有运行记录的时间对齐诊断（阶段性）", y=0.995, fontsize=9, weight="bold")
    fig.text(
        0.5,
        0.005,
        "注：当前日志未保存权威注入起止时间，故本图不标注“真实异常区间”；最终实验需由注入节点写出标签文件。",
        ha="center",
        fontsize=6.5,
        color=COLORS["red"],
    )
    save_publication_figure(fig, "innovation2_preliminary_timeline")
    plt.close(fig)


def main():
    configure_matplotlib()
    generate_system_pipeline()
    generate_preliminary_timeline()
    print(OUTPUT)


if __name__ == "__main__":
    main()
