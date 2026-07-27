#!/usr/bin/env python3

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image


ROOT = "/home/slam/slam_ws/paper_exp/formal_corridor_20260724"
MAPS = {
    "repeat_01": os.path.join(
        ROOT, "partial_precheck", "clean_baseline", "map"),
    "repeat_02": os.path.join(
        ROOT, "repeat_02_03_clean_baseline", "repeat_02", "map"),
    "repeat_03": os.path.join(
        ROOT, "repeat_02_03_clean_baseline", "repeat_03", "map"),
}


def load_map(prefix):
    image = np.asarray(Image.open(prefix + ".pgm"), dtype=np.uint8)
    with open(prefix + ".yaml", "r", encoding="utf-8") as source:
        metadata = yaml.safe_load(source)
    return image, metadata


def map_metrics(image, metadata):
    resolution = float(metadata["resolution"])
    total = image.size
    occupied = int(np.count_nonzero(image == 0))
    free = int(np.count_nonzero(image == 254))
    unknown = int(np.count_nonzero(image == 205))
    non_unknown = occupied + free
    return {
        "width_pixels": int(image.shape[1]),
        "height_pixels": int(image.shape[0]),
        "resolution_m": resolution,
        "width_m": float(image.shape[1] * resolution),
        "height_m": float(image.shape[0] * resolution),
        "occupied_cells": occupied,
        "free_cells": free,
        "unknown_cells": unknown,
        "occupied_ratio_all": float(occupied / max(1, total)),
        "occupied_ratio_observed": float(occupied / max(1, non_unknown)),
    }


def occupied_crop(image, padding=8):
    rows, columns = np.where(image == 0)
    if not len(rows):
        return image
    row_min = max(0, int(rows.min()) - padding)
    row_max = min(image.shape[0], int(rows.max()) + padding + 1)
    col_min = max(0, int(columns.min()) - padding)
    col_max = min(image.shape[1], int(columns.max()) + padding + 1)
    return image[row_min:row_max, col_min:col_max]


def draw_comparison(loaded, output_path):
    figure, axes = plt.subplots(3, 1, figsize=(16, 8),
                                constrained_layout=True)
    for axis, (name, (image, metadata)) in zip(axes, loaded.items()):
        crop = occupied_crop(image)
        resolution = float(metadata["resolution"])
        axis.imshow(crop, cmap="gray", vmin=0, vmax=255,
                    extent=[0, crop.shape[1] * resolution,
                            0, crop.shape[0] * resolution])
        axis.set_title(f"Clean Baseline - {name}")
        axis.set_xlabel("map width (m)")
        axis.set_ylabel("map height (m)")
        axis.set_aspect("equal")
    figure.savefig(output_path, dpi=200)
    plt.close(figure)


def write_report(metrics, output_path):
    with open(output_path, "w", encoding="utf-8") as output:
        output.write("# corridor_repeat 三次独立录制 Clean Baseline 对照\n\n")
        output.write("## 控制变量\n\n")
        output.write("- 三包统一使用 `clean_baseline_no_innovation.lua`。\n")
        output.write("- 创新点一、创新点二和扫描质量门控均关闭。\n")
        output.write("- 统一使用 `base_link -> laser_link z=0.33521 m`、"
                     "yaw=-1.5708 rad。\n")
        output.write("- 点云投影高度带统一为 `[-0.515,1.435] m`。\n\n")
        output.write("## 地图统计\n\n")
        output.write("| bag | 地图宽度/m | 地图高度/m | 占据栅格 | 空闲栅格 | 未知栅格 | 已观测区占据率 |\n")
        output.write("|---|---:|---:|---:|---:|---:|---:|\n")
        for name, current in metrics.items():
            output.write(
                f"| {name} | {current['width_m']:.2f} "
                f"| {current['height_m']:.2f} "
                f"| {current['occupied_cells']} "
                f"| {current['free_cells']} "
                f"| {current['unknown_cells']} "
                f"| {100.0 * current['occupied_ratio_observed']:.2f}% |\n")
        output.write("\n## 结果判定\n\n")
        output.write("- 三次地图均为约 81 m 画布宽度的单条长走廊，未出现 90 度错误折转、平行双走廊或全局拓扑崩溃。\n")
        output.write("- `repeat_02` 和 `repeat_03` 均重复出现与 `repeat_01` 相似的长墙缺失，尤其是走廊上侧墙面在完整往返后出现大段空白。\n")
        output.write("- 三条独立录制均出现同类缺墙，结合 `repeat_01` 的去程/回程和关闭自由空间单变量实验，可排除第一包偶发损坏；问题来自共同的概率栅格自由空间更新与后期子图覆盖机制。\n")
        output.write("- 三个 bag 均可继续作为论文实验输入，但在正式六组消融前应先冻结一组不会大面积清墙的 hit/miss 参数；否则地图连续性指标会被共同的栅格插入问题污染。\n")


def main():
    loaded = {name: load_map(prefix) for name, prefix in MAPS.items()}
    metrics = {name: map_metrics(*value) for name, value in loaded.items()}
    metrics_path = os.path.join(ROOT, "repeat_clean_baseline_metrics.json")
    report_path = os.path.join(ROOT, "repeat_clean_baseline_report.md")
    figure_path = os.path.join(ROOT, "repeat_clean_baseline_comparison.png")
    with open(metrics_path, "w", encoding="utf-8") as output:
        json.dump(metrics, output, ensure_ascii=False, indent=2,
                  sort_keys=True)
    draw_comparison(loaded, figure_path)
    write_report(metrics, report_path)
    print(json.dumps(metrics, ensure_ascii=False, indent=2, sort_keys=True))
    print("report:", report_path)
    print("figure:", figure_path)


if __name__ == "__main__":
    main()
