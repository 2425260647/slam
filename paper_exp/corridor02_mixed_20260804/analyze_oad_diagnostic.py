#!/usr/bin/env python3
"""Analyze read-only OAD-CSM diagnostics without claiming trajectory truth."""

import argparse
import csv
import json
import math
from pathlib import Path


def read_float(path, key="field.data"):
    rows = []
    with open(path, newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            rows.append((int(float(row["%time"])), float(row[key])))
    return rows


def read_offset(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            rows.append((int(float(row["%time"])), float(row["field.x"]),
                         float(row["field.y"]), float(row["field.z"])))
    return rows


def read_uint(path):
    return read_float(path)


def nearest(series, timestamp):
    return min(series, key=lambda item: abs(item[0] - timestamp))


def quantiles(values):
    if not values:
        return {"count": 0}
    values = sorted(values)

    def q(p):
        index = (len(values) - 1) * p
        low = int(math.floor(index))
        high = int(math.ceil(index))
        if low == high:
            return values[low]
        return values[low] + (values[high] - values[low]) * (index - low)

    return {"count": len(values), "min": values[0], "p50": q(0.5),
            "p95": q(0.95), "max": values[-1]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    root = Path(args.run_dir)

    offset = read_offset(root / "correlative_candidate_offset.csv")
    fixed = read_float(root / "correlative_fixed_map_score.csv")
    expanded = read_float(root / "correlative_expanded_map_score.csv")
    margin = read_float(root / "correlative_peak_margin.csv")
    second_valid = read_float(root / "correlative_second_peak_valid.csv")
    boundary = read_float(root / "correlative_candidate_on_boundary.csv")
    search_ms = read_float(root / "correlative_candidate_search_time_ms.csv")
    candidates = read_uint(root / "correlative_candidate_count.csv")
    submap = read_uint(root / "correlative_candidate_submap_num_range_data.csv")

    records = []
    for timestamp, x, y, yaw in offset:
        f = nearest(fixed, timestamp)[1]
        e = nearest(expanded, timestamp)[1]
        m = nearest(margin, timestamp)[1]
        v = nearest(second_valid, timestamp)[1] > 0.5
        b = nearest(boundary, timestamp)[1] > 0.5
        t = nearest(search_ms, timestamp)[1]
        n = nearest(candidates, timestamp)[1]
        s = nearest(submap, timestamp)[1]
        records.append({
            "time_sec": timestamp / 1e9,
            "offset_x_m": x,
            "offset_y_m": y,
            "offset_yaw_rad": yaw,
            "offset_translation_m": math.hypot(x, y),
            "fixed_map_score": f,
            "expanded_map_score": e,
            "map_score_gain": e - f,
            "independent_peak_margin": m,
            "second_peak_valid": v,
            "boundary": b,
            "search_time_ms": t,
            "candidate_count": n,
            "submap_num_range_data": s,
        })

    candidates_outside = [r for r in records if r["offset_translation_m"] > 0.15]
    eligible = [r for r in candidates_outside if not r["boundary"] and
                r["map_score_gain"] >= 0.02 and r["second_peak_valid"] and
                r["independent_peak_margin"] >= 0.01]

    event_groups = []
    for record in eligible:
        if not event_groups or record["time_sec"] - event_groups[-1][-1]["time_sec"] > 2.0:
            event_groups.append([record])
        else:
            event_groups[-1].append(record)

    persistent_event_groups = [group for group in event_groups if len(group) >= 2]
    result = {
        "run_dir": str(root),
        "sample_count": len(records),
        "outside_fixed_window_count": len(candidates_outside),
        "outside_fixed_window_ratio": len(candidates_outside) / len(records) if records else 0.0,
        "boundary_count": sum(r["boundary"] for r in records),
        "second_peak_valid_count": sum(r["second_peak_valid"] for r in records),
        "map_score_gain": quantiles([r["map_score_gain"] for r in records]),
        "offset_translation_m": quantiles([r["offset_translation_m"] for r in records]),
        "search_time_ms": quantiles([r["search_time_ms"] for r in records]),
        "candidate_count": quantiles([r["candidate_count"] for r in records]),
        "submap_num_range_data": quantiles([r["submap_num_range_data"] for r in records]),
        "eligible_sample_count": len(eligible),
        "eligible_event_count": len(persistent_event_groups),
        "eligible_event_lengths": [len(group) for group in persistent_event_groups],
        "pre_registered_gate": {
            "outside_offset_m": 0.15,
            "map_score_gain": 0.02,
            "independent_peak_margin": 0.01,
            "not_boundary": True,
            "second_peak_valid": True,
            "min_consecutive_samples": 2,
            "min_events": 3,
        },
        "events": persistent_event_groups,
        "limitations": [
            "No official Corridor02 GT body trajectory was available during this run.",
            "Candidate is read-only and never replaces production pose.",
            "Map score is an empirical occupancy-grid score, not a likelihood.",
            "Eligible events are a mechanism gate, not proof of trajectory improvement.",
        ],
    }
    Path(args.output).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: result[k] for k in (
        "sample_count", "outside_fixed_window_count", "outside_fixed_window_ratio",
        "eligible_sample_count", "eligible_event_count", "search_time_ms",
        "map_score_gain", "offset_translation_m")}, indent=2))


if __name__ == "__main__":
    main()
