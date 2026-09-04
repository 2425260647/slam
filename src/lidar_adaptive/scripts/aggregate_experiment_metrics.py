#!/usr/bin/env python3
"""Aggregate per-run lidar_adaptive metrics without inventing accuracy values."""

import argparse
import json
import math
import statistics
from pathlib import Path


def finite(values):
    return [float(value) for value in values
            if value is not None and math.isfinite(float(value))]


def stats(values):
    values = finite(values)
    if not values:
        return {"count": 0, "mean": None, "std": None,
                "min": None, "max": None}
    return {
        "count": len(values),
        "mean": statistics.mean(values),
        "std": statistics.stdev(values) if len(values) > 1 else 0.0,
        "min": min(values),
        "max": max(values),
    }


def load_run(run_dir):
    run_dir = Path(run_dir)
    summary_path = run_dir / "metrics" / "summary.json"
    if not summary_path.is_file():
        return {"run": str(run_dir), "status": "missing_summary"}
    with summary_path.open(encoding="utf-8") as stream:
        summary = json.load(stream)
    name = run_dir.name
    case = next((part.upper() for part in name.split("_")
                 if part.lower() in {"c0", "c1", "c2", "c3"}), "unknown")
    trajectory = None
    trajectory_path = run_dir / "metrics" / "trajectory.json"
    if trajectory_path.is_file():
        with trajectory_path.open(encoding="utf-8") as stream:
            trajectory = json.load(stream)
    return {"run": str(run_dir), "name": name, "case": case,
            "status": "ok", "summary": summary, "trajectory": trajectory}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", nargs="+", help="experiment directories")
    parser.add_argument("--output", required=True)
    parser.add_argument("--markdown-output", default="")
    args = parser.parse_args()

    runs = [load_run(path) for path in args.runs]
    valid = [run for run in runs if run["status"] == "ok"]
    grouped = {}
    for run in valid:
        grouped.setdefault(run["case"], []).append(run)

    metric_names = [
        "projection_quality_mean", "information_score_mean",
        "quality_sync_fraction", "selection_fraction",
        "projection_processing_ms_mean", "selector_processing_ms_mean",
    ]
    groups = {}
    for case, case_runs in sorted(grouped.items()):
        case_result = {
            "runs": [run["name"] for run in case_runs],
            "run_count": len(case_runs),
            "metrics": {},
        }
        for metric in metric_names:
            case_result["metrics"][metric] = stats(
                [run["summary"].get(metric) for run in case_runs])
        case_result["input_messages"] = stats([
            run["summary"].get("message_counts", {}).get("/velodyne_points")
            for run in case_runs])
        case_result["cartographer_scan_messages"] = stats([
            run["summary"].get("message_counts", {}).get("/scan_selected",
                run["summary"].get("message_counts", {}).get("/scan"))
            for run in case_runs])
        case_result["selected_messages"] = stats([
            run["summary"].get("selected_messages") for run in case_runs])
        case_result["ground_truth_available"] = any(
            run["summary"].get("ground_truth_available", False)
            for run in case_runs)
        case_result["accuracy_metrics_allowed"] = all(
            run["summary"].get("accuracy_metrics_allowed", False)
            for run in case_runs)
        trajectory_metrics = [
            "ate_translation_rmse", "ate_translation_p95",
            "ate_rotation_rmse_rad", "rpe_translation_rmse",
            "rpe_rotation_rmse_rad",
        ]
        case_result["trajectory_metrics"] = {
            metric: stats([(run["trajectory"] or {}).get(metric)
                           for run in case_runs])
            for metric in trajectory_metrics
        }
        groups[case] = case_result

    has_truth = any(data["ground_truth_available"]
                    for data in groups.values())
    result = {
        "runs_requested": len(runs),
        "runs_valid": len(valid),
        "runs_missing_summary": len(runs) - len(valid),
        "ground_truth_note": (
            "Gazebo runs include continuous model-state truth; ATE/RPE are "
            "computed from recorded truth."
            if has_truth else
            "Real Scout corridor bags have no continuous external truth; "
            "ATE/RPE are intentionally unavailable."),
        "groups": groups,
        "runs": runs,
    }
    baseline = groups.get("C0")
    relative_to_c0 = {}
    if baseline:
        for case, case_result in groups.items():
            if case == "C0":
                continue
            changes = {}
            for metric, values in case_result["trajectory_metrics"].items():
                base_mean = baseline["trajectory_metrics"][metric]["mean"]
                case_mean = values["mean"]
                changes[metric] = ((case_mean - base_mean) / abs(base_mean)
                                   if base_mean not in (None, 0) and
                                   case_mean is not None else None)
            relative_to_c0[case] = changes
    result["relative_to_C0"] = relative_to_c0
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(rendered, encoding="ascii")
    print(rendered, end="")
    if args.markdown_output:
        has_truth = any(data["ground_truth_available"]
                        for data in groups.values())
        truth_note = ("Gazebo runs include continuous model-state truth; "
                      "ATE/RPE are computed from the recorded truth."
                      if has_truth else
                      "Real Scout corridor bags have no continuous external "
                      "truth; ATE/RPE are intentionally unavailable.")
        markdown = [
            "# Experiment aggregate",
            "",
            "This report aggregates the JSON summaries in the listed run directories.",
            truth_note,
            "",
            "| Case | Runs | Scan messages (mean) | Selection fraction (mean) | ATE RMSE (mean, m) | 1 s RPE RMSE (mean, m) |",
            "| --- | ---: | ---: | ---: | ---: | ---: |",
        ]
        for case, data in sorted(groups.items()):
            scan_mean = data["cartographer_scan_messages"]["mean"]
            select_mean = data["metrics"]["selection_fraction"]["mean"]
            ate_mean = data["trajectory_metrics"]["ate_translation_rmse"]["mean"]
            rpe_mean = data["trajectory_metrics"]["rpe_translation_rmse"]["mean"]
            fmt = lambda value: "n/a" if value is None else f"{value:.6f}"
            markdown.append(
                f"| {case} | {data['run_count']} | {fmt(scan_mean)} | "
                f"{fmt(select_mean)} | {fmt(ate_mean)} | {fmt(rpe_mean)} |")
        markdown.extend([
            "",
            "Relative changes use `(case - C0) / abs(C0)` on the means. A negative",
            "value is lower than C0 for that metric; it is not by itself evidence of",
            "statistical significance or generalization.",
            "",
            "```json",
            json.dumps(relative_to_c0, indent=2, sort_keys=True),
            "```",
            "",
        ])
        markdown_path = Path(args.markdown_output)
        markdown_path.parent.mkdir(parents=True, exist_ok=True)
        markdown_path.write_text("\n".join(markdown), encoding="ascii")


if __name__ == "__main__":
    main()
