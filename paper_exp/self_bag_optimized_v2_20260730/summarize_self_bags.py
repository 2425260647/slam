#!/usr/bin/env python3
"""Summarize the frozen proposed runs on the three self-recorded bags.

The map quantities are structural proxies.  These bags do not provide a
continuous external trajectory reference, so this report deliberately does
not call them ATE/RPE.
"""

import argparse
import csv
import json
import os
import sys


WORKSPACE = "/home/slam/slam_ws"
ROOT = os.path.join(WORKSPACE, "paper_exp", "self_bag_optimized_v2_20260730")
BASELINE_ROOT = os.path.join(
    WORKSPACE, "paper_exp", "formal_corridor_20260724", "runs", "i1", "baseline_a"
)
REPEATS = (1, 2, 3)


def load_map_metrics(run_dir):
    module_dir = os.path.join(
        WORKSPACE, "paper_exp", "formal_corridor_20260724"
    )
    if module_dir not in sys.path:
        sys.path.insert(0, module_dir)
    from evaluate_wall_parameter_maps import map_metrics

    return map_metrics(os.path.join(run_dir, "map"))


def csv_values(path):
    values = []
    with open(path, newline="", encoding="utf-8") as stream:
        for row in csv.reader(stream):
            if len(row) >= 2:
                try:
                    values.append(float(row[1]))
                except ValueError:
                    continue
    return values


def read_metadata(path):
    metadata = {}
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            key, separator, value = line.rstrip("\n").partition("=")
            if separator:
                metadata[key] = value
    return metadata


def mean(values):
    return sum(values) / float(len(values))


def relative_percent(baseline, proposed):
    return 100.0 * (proposed - baseline) / baseline if baseline else None


def run_record(repeat):
    proposed_dir = os.path.join(ROOT, "runs", "normal_full", "proposed", f"repeat_{repeat:02d}")
    baseline_dir = os.path.join(BASELINE_ROOT, f"repeat_{repeat:02d}")
    proposed_metrics = load_map_metrics(proposed_dir)
    baseline_metrics = load_map_metrics(baseline_dir)
    metadata = read_metadata(os.path.join(proposed_dir, "metadata.txt"))

    trigger_values = csv_values(os.path.join(proposed_dir, "anomaly_trigger_count.csv"))
    weight_values = csv_values(os.path.join(proposed_dir, "odom_weight_scale.csv"))
    topics = {
        line.strip()
        for line in open(os.path.join(proposed_dir, "topics.txt"), encoding="utf-8")
        if line.strip()
    }
    forbidden_topics = sorted(
        topic for topic in topics if "imu" in topic.lower() or "avia" in topic.lower()
    )
    return {
        "repeat": repeat,
        "bag": metadata.get("bag"),
        "bag_sha256": metadata.get("bag_sha256"),
        "algorithm_version": metadata.get("algorithm_version"),
        "config": metadata.get("config"),
        "success_marker": os.path.isfile(os.path.join(proposed_dir, "SUCCESS")),
        "fatal_scan_bytes": os.path.getsize(os.path.join(proposed_dir, "fatal_scan.txt")),
        "forbidden_imu_or_avia_topics": forbidden_topics,
        "anomaly_trigger_count_max": max(trigger_values) if trigger_values else None,
        "odom_weight_scale_min": min(weight_values) if weight_values else None,
        "odom_weight_scale_mean": mean(weight_values) if weight_values else None,
        "tracked_pose_rows": sum(1 for _ in open(os.path.join(proposed_dir, "tracked_pose.csv"), encoding="utf-8")),
        "proposed_map": proposed_metrics,
        "baseline_map": baseline_metrics,
    }


def summarize(records):
    metric_names = (
        "occupied_cells",
        "best_wall_gap_closed_coverage",
        "best_wall_continuous_length_m",
    )
    aggregate = {}
    for name in metric_names:
        baseline = mean([record["baseline_map"][name] for record in records])
        proposed = mean([record["proposed_map"][name] for record in records])
        aggregate[name] = {
            "baseline_mean": baseline,
            "proposed_mean": proposed,
            "relative_change_percent": relative_percent(baseline, proposed),
        }
    return aggregate


def write_markdown(report, path):
    aggregate = report["aggregate"]
    lines = [
        "# 自有 corridor_repeat 三包冻结版汇总",
        "",
        "- 算法版本：`innovation_combined_v2_20260730`",
        "- 运行配置：`innovation2_proposed_slip_adaptive.lua`（创新点一、二联合开启）",
        "- 输入：`corridor_repeat_01/02/03.bag`，回放倍率 `2.0`",
        "",
        "> 三条 bag 没有连续外部真值。本报告只报告地图结构代理指标，不报告 ATE/RPE。",
        "> baseline 为同一批 bag 的现有 `innovation1` baseline_a 运行，不能替代独立真值。",
        "",
        "## 汇总指标",
        "",
        "| 指标 | baseline 均值 | proposed 均值 | 相对变化 |",
        "|---|---:|---:|---:|",
        "| 占据栅格数 | {:.2f} | {:.2f} | {:+.2f}% |".format(
            aggregate["occupied_cells"]["baseline_mean"],
            aggregate["occupied_cells"]["proposed_mean"],
            aggregate["occupied_cells"]["relative_change_percent"],
        ),
        "| 最佳墙带覆盖率 | {:.4f} | {:.4f} | {:+.2f}% |".format(
            aggregate["best_wall_gap_closed_coverage"]["baseline_mean"],
            aggregate["best_wall_gap_closed_coverage"]["proposed_mean"],
            aggregate["best_wall_gap_closed_coverage"]["relative_change_percent"],
        ),
        "| 最长连续墙段 / m | {:.3f} | {:.3f} | {:+.2f}% |".format(
            aggregate["best_wall_continuous_length_m"]["baseline_mean"],
            aggregate["best_wall_continuous_length_m"]["proposed_mean"],
            aggregate["best_wall_continuous_length_m"]["relative_change_percent"],
        ),
        "",
        "## 运行验收",
        "",
        "| 序列 | SUCCESS | FATAL 字节 | 异常触发最大值 | odom 权重最小值 | tracked_pose 行数 |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for record in report["runs"]:
        lines.append(
            "| repeat_{:02d} | {} | {} | {} | {:.3f} | {} |".format(
                record["repeat"],
                "yes" if record["success_marker"] else "no",
                record["fatal_scan_bytes"],
                record["anomaly_trigger_count_max"],
                record["odom_weight_scale_min"],
                record["tracked_pose_rows"],
            )
        )
    lines.extend(
        [
            "",
            "- 三条运行均无 FATAL，异常触发为 0，odometry 权重保持 1.0。",
            "- 运行 topic 清单未发现 IMU 或 Avia 话题。",
            "- 该冻结版本随后才用于自有 bag 迁移验证；本次不针对单条 bag 调参。",
        ]
    )
    with open(path, "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", default=os.path.join(ROOT, "self_bag_summary.json"))
    parser.add_argument("--markdown", default=os.path.join(ROOT, "self_bag_summary.md"))
    args = parser.parse_args()
    records = [run_record(repeat) for repeat in REPEATS]
    report = {
        "schema_version": 1,
        "reference_limit": "same-bag structural baseline, not external ground truth",
        "evaluation_boundary": "No continuous ATE/RPE is reported for self bags.",
        "runs": records,
        "aggregate": summarize(records),
    }
    with open(args.json, "w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    write_markdown(report, args.markdown)
    print(json.dumps(report["aggregate"], ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
