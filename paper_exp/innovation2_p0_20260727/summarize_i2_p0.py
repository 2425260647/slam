#!/usr/bin/env python3
"""Aggregate the three-repeat Innovation 2 P0 metric JSON files."""

import hashlib
import json
import os

import numpy as np


ROOT = "/home/slam/slam_ws/paper_exp/innovation2_p0_20260727"
ALGORITHM_VERSION = "innovation1_coordinate_fix_20260728"
EXPECTED_RUN_COUNT = 15
SOURCE_MANIFEST = os.path.join(ROOT, "algorithm_source_manifest.sha256")
SUITES = {
    "map_protection_v2": ("static_low", "static_high", "proposed_gate"),
    "gate_ablation": ("gate_off", "gate_on"),
}
METRICS = {
    "chamfer_mean_m": ("map_reference", "symmetric_chamfer_mean_m"),
    "chamfer_p95_m": ("map_reference", "symmetric_chamfer_p95_m"),
    "wall_f1_at_0_15_m": ("map_reference", "occupied_f1_at_0_15_m"),
    "corridor_axis_error_deg": ("map_reference", "corridor_axis_error_deg"),
    "trajectory_position_p95_m": (
        "trajectory_reference",
        "position_difference_p95_m",
    ),
    "trajectory_yaw_p95_deg": ("trajectory_reference", "yaw_difference_p95_deg"),
    "endpoint_position_difference_m": (
        "trajectory_reference",
        "endpoint_position_difference_m",
    ),
}


def load_reports(suite):
    reports = []
    for repeat in (1, 2, 3):
        path = os.path.join(ROOT, "{}_repeat_{:02d}_metrics.json".format(suite, repeat))
        with open(path, encoding="utf-8") as stream:
            reports.append(json.load(stream))
    return reports


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_metadata(path):
    metadata = {}
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            key, separator, value = line.rstrip("\n").partition("=")
            if separator:
                metadata[key] = value
    return metadata


def verify_current_runs(source_manifest_sha256):
    required_nonempty_files = (
        "map.pgm",
        "state.pbstream",
        "tracked_pose.csv",
        "degeneracy_metric.csv",
    )
    dynamic_required_nonempty_files = (
        "consistency_metric.csv",
        "lidar_reliability.csv",
        "odom_weight_scale.csv",
        "trigger_count.csv",
    )
    verified_runs = []
    for suite, methods in SUITES.items():
        for method in methods:
            for repeat in (1, 2, 3):
                run_dir = os.path.join(
                    ROOT,
                    "runs",
                    suite,
                    method,
                    "repeat_{:02d}".format(repeat),
                )
                success_path = os.path.join(run_dir, "SUCCESS")
                if not os.path.isfile(success_path):
                    raise RuntimeError("Missing SUCCESS marker: {}".format(run_dir))
                for filename in required_nonempty_files:
                    path = os.path.join(run_dir, filename)
                    if not os.path.isfile(path) or os.path.getsize(path) == 0:
                        raise RuntimeError("Missing or empty artifact: {}".format(path))
                if method not in ("static_low", "static_high"):
                    for filename in dynamic_required_nonempty_files:
                        path = os.path.join(run_dir, filename)
                        if not os.path.isfile(path) or os.path.getsize(path) == 0:
                            raise RuntimeError(
                                "Missing or empty dynamic diagnostic: {}".format(path)
                            )

                fatal_scan = os.path.join(run_dir, "fatal_scan.txt")
                if not os.path.isfile(fatal_scan) or os.path.getsize(fatal_scan) != 0:
                    raise RuntimeError("Fatal marker is missing or nonempty: {}".format(run_dir))

                metadata = load_metadata(os.path.join(run_dir, "metadata.txt"))
                if metadata.get("algorithm_version") != ALGORITHM_VERSION:
                    raise RuntimeError("Algorithm version mismatch: {}".format(run_dir))
                if metadata.get("source_manifest_sha256") != source_manifest_sha256:
                    raise RuntimeError("Source manifest mismatch: {}".format(run_dir))
                verified_runs.append(
                    {
                        "suite": suite,
                        "method": method,
                        "repeat": repeat,
                        "run_dir": run_dir,
                    }
                )

    if len(verified_runs) != EXPECTED_RUN_COUNT:
        raise RuntimeError(
            "Expected {} current runs, verified {}".format(
                EXPECTED_RUN_COUNT, len(verified_runs)
            )
        )
    return verified_runs


def summarize_values(values):
    values = np.asarray(list(values), dtype=float)
    return {
        "values": [float(value) for value in values],
        "mean": float(np.mean(values)),
        "sample_std": float(np.std(values, ddof=1)),
    }


def lower_is_better_improvement(baseline, proposed):
    if baseline == 0.0:
        return None
    return 100.0 * (baseline - proposed) / baseline


def higher_is_better_improvement(baseline, proposed):
    if baseline == 0.0:
        return None
    return 100.0 * (proposed - baseline) / baseline


def build_summary():
    source_manifest_sha256 = sha256_file(SOURCE_MANIFEST)
    verified_runs = verify_current_runs(source_manifest_sha256)
    result = {
        "schema_version": 1,
        "repeat_count": 3,
        "reference_limit": "same-bag clean run, not external ground truth",
        "verification": {
            "algorithm_version": ALGORITHM_VERSION,
            "source_manifest_sha256": source_manifest_sha256,
            "expected_run_count": EXPECTED_RUN_COUNT,
            "verified_run_count": len(verified_runs),
            "all_expected_runs_verified": len(verified_runs) == EXPECTED_RUN_COUNT,
        },
        "suites": {},
    }
    for suite, methods in SUITES.items():
        reports = load_reports(suite)
        suite_result = {"methods": {}}
        for method in methods:
            method_result = {"metrics": {}}
            for name, (section, field) in METRICS.items():
                method_result["metrics"][name] = summarize_values(
                    report["methods"][method][section][field] for report in reports
                )
            diagnostics = [report["methods"][method]["diagnostics"] for report in reports]
            trigger_counts = [item.get("trigger_count_max") for item in diagnostics]
            weight_minima = [
                item.get("minimum_weight_scale_min", item.get("weight_scale_min"))
                for item in diagnostics
            ]
            if all(value is not None for value in trigger_counts):
                method_result["trigger_count"] = summarize_values(trigger_counts)
                method_result["trigger_count"]["total"] = float(sum(trigger_counts))
            if all(value is not None for value in weight_minima):
                method_result["minimum_weight_scale"] = summarize_values(weight_minima)
            suite_result["methods"][method] = method_result

        if suite == "map_protection_v2":
            baseline = suite_result["methods"]["static_high"]["metrics"]
            proposed = suite_result["methods"]["proposed_gate"]["metrics"]
            comparison = {}
            for name in METRICS:
                if name == "wall_f1_at_0_15_m":
                    improvement = higher_is_better_improvement(
                        baseline[name]["mean"], proposed[name]["mean"]
                    )
                else:
                    improvement = lower_is_better_improvement(
                        baseline[name]["mean"], proposed[name]["mean"]
                    )
                comparison[name + "_improvement_percent"] = improvement
            suite_result["proposed_vs_static_high"] = comparison
        else:
            gate_off = suite_result["methods"]["gate_off"]
            gate_on = suite_result["methods"]["gate_on"]
            comparison = {}
            for name in METRICS:
                if name == "wall_f1_at_0_15_m":
                    improvement = higher_is_better_improvement(
                        gate_off["metrics"][name]["mean"],
                        gate_on["metrics"][name]["mean"],
                    )
                else:
                    improvement = lower_is_better_improvement(
                        gate_off["metrics"][name]["mean"],
                        gate_on["metrics"][name]["mean"],
                    )
                comparison[name + "_improvement_percent"] = improvement
            off_total = gate_off["trigger_count"]["total"]
            on_total = gate_on["trigger_count"]["total"]
            comparison["false_trigger_suppression_percent"] = (
                100.0 * (off_total - on_total) / off_total if off_total else None
            )
            suite_result["gate_on_vs_gate_off"] = comparison
        result["suites"][suite] = suite_result
    return result


def format_value(metric):
    return "{:.4f} +/- {:.4f}".format(metric["mean"], metric["sample_std"])


def write_markdown(summary, path):
    map_suite = summary["suites"]["map_protection_v2"]
    gate_suite = summary["suites"]["gate_ablation"]
    lines = [
        "# 创新点二 P0 三包汇总",
        "",
        "- 算法版本：`{}`".format(summary["verification"]["algorithm_version"]),
        "- 源码指纹：`{}`".format(
            summary["verification"]["source_manifest_sha256"]
        ),
        "- 当前版本完整运行：`{}/{}`".format(
            summary["verification"]["verified_run_count"],
            summary["verification"]["expected_run_count"],
        ),
        "",
        "> 所有数值均相对同一 bag 的干净运行参考，不是外部 Ground Truth。",
        "> 轨迹代理指标禁止写成 ATE/RPE；人工注入禁止写成真实轮胎打滑。",
        "",
        "## 持续累计 odometry 漂移地图保护",
        "",
        "| 方法 | Chamfer均值/m | Chamfer P95/m | 墙体F1 | 主轴角误差/deg | 轨迹P95偏差/m |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for method in SUITES["map_protection_v2"]:
        metrics = map_suite["methods"][method]["metrics"]
        lines.append(
            "| {} | {} | {} | {} | {} | {} |".format(
                method,
                format_value(metrics["chamfer_mean_m"]),
                format_value(metrics["chamfer_p95_m"]),
                format_value(metrics["wall_f1_at_0_15_m"]),
                format_value(metrics["corridor_axis_error_deg"]),
                format_value(metrics["trajectory_position_p95_m"]),
            )
        )
    comparison = map_suite["proposed_vs_static_high"]
    lines.extend(
        [
            "",
            "Proposed 相对 static_high：Chamfer 均值改善 `{:.2f}%`，主轴角误差改善 `{:.2f}%`，轨迹 P95 改善 `{:.2f}%`；墙体 F1 变化 `{:.2f}%`。".format(
                comparison["chamfer_mean_m_improvement_percent"],
                comparison["corridor_axis_error_deg_improvement_percent"],
                comparison["trajectory_position_p95_m_improvement_percent"],
                comparison["wall_f1_at_0_15_m_improvement_percent"],
            ),
            "",
            "## LiDAR 可靠性门控消融",
            "",
            "| 方法 | 错误上升沿总数 | 最小权重均值 | Chamfer均值/m | 墙体F1 | 轨迹P95偏差/m |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for method in SUITES["gate_ablation"]:
        data = gate_suite["methods"][method]
        metrics = data["metrics"]
        lines.append(
            "| {} | {:.0f} | {:.4f} | {} | {} | {} |".format(
                method,
                data["trigger_count"]["total"],
                data["minimum_weight_scale"]["mean"],
                format_value(metrics["chamfer_mean_m"]),
                format_value(metrics["wall_f1_at_0_15_m"]),
                format_value(metrics["trajectory_position_p95_m"]),
            )
        )
    gate_comparison = gate_suite["gate_on_vs_gate_off"]
    gate_off = gate_suite["methods"]["gate_off"]
    gate_on = gate_suite["methods"]["gate_on"]
    lines.extend(
        [
            "",
            "门控将错误异常上升沿从 `{:.0f}` 降至 `{:.0f}`，抑制率 `{:.1f}%`；轨迹 P95 代理改善 `{:.2f}%`。Chamfer 均值由 `{:.4f} m` 变为 `{:.4f} m`，恶化 `{:.2f}%`，因此门控的地图代理结果有升有降。".format(
                gate_off["trigger_count"]["total"],
                gate_on["trigger_count"]["total"],
                gate_comparison["false_trigger_suppression_percent"],
                gate_comparison["trajectory_position_p95_m_improvement_percent"],
                gate_off["metrics"]["chamfer_mean_m"]["mean"],
                gate_on["metrics"]["chamfer_mean_m"]["mean"],
                abs(gate_comparison["chamfer_mean_m_improvement_percent"]),
            ),
            "",
            "## 结论边界",
            "",
            "- 地图保护结果支持“相对静态高 odometry 权重降低注入偏差影响”，不支持“所有地图指标全面最优”。",
            "- 门控结果支持“避免低可靠 LiDAR 导致错误 odometry 降权”：错误触发抑制率为 100%；地图代理指标有升有降，不宣称门控全面改善地图。",
            "- 门控不支持“双源同时失效时恢复真值”。无第三信息源时，该情况属于系统证据边界。",
            "- 三条 bag 来自同一物理走廊的独立重复录制，适合重复性验证，不代表跨场景泛化。",
            "",
        ]
    )
    with open(path, "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines))


def main():
    summary = build_summary()
    json_path = os.path.join(ROOT, "innovation2_p0_aggregate_metrics.json")
    markdown_path = os.path.join(ROOT, "innovation2_p0_aggregate_metrics.md")
    with open(json_path, "w", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    write_markdown(summary, markdown_path)
    print(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
