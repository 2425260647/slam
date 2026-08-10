#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path

import rosbag


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    index = min(len(ordered) - 1, int(math.ceil(fraction * len(ordered))) - 1)
    return ordered[max(0, index)]


def main():
    parser = argparse.ArgumentParser(description="Audit a deskewed LaserScan bag")
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    scan_count = 0
    frames = set()
    stamps = []
    scan_times = []
    time_increments = []
    finite_counts = []
    nonmonotonic_stamps = 0
    range_lengths = set()
    isolated_return_count = 0
    finite_return_count = 0

    with rosbag.Bag(str(args.bag), "r") as bag:
        for _, scan, _ in bag.read_messages(topics=["/scan"]):
            scan_count += 1
            stamp = scan.header.stamp.to_sec()
            if stamps and stamp <= stamps[-1]:
                nonmonotonic_stamps += 1
            stamps.append(stamp)
            frames.add(scan.header.frame_id)
            scan_times.append(float(scan.scan_time))
            time_increments.append(float(scan.time_increment))
            range_lengths.add(len(scan.ranges))
            finite = [math.isfinite(value) for value in scan.ranges]
            finite_count = sum(finite)
            finite_counts.append(finite_count)
            finite_return_count += finite_count
            for index, valid in enumerate(finite):
                if not valid:
                    continue
                supported = False
                current = float(scan.ranges[index])
                for offset in range(-3, 4):
                    if offset == 0:
                        continue
                    neighbor = index + offset
                    if neighbor < 0 or neighbor >= len(scan.ranges):
                        continue
                    other = float(scan.ranges[neighbor])
                    if not math.isfinite(other):
                        continue
                    angular_difference = offset * float(scan.angle_increment)
                    distance_squared = (
                        current * current
                        + other * other
                        - 2.0 * current * other * math.cos(angular_difference)
                    )
                    if distance_squared <= 0.30 * 0.30:
                        supported = True
                        break
                if not supported:
                    isolated_return_count += 1

    result = {
        "bag": str(args.bag.resolve()),
        "scan_count": scan_count,
        "frames": sorted(frames),
        "range_lengths": sorted(range_lengths),
        "nonmonotonic_scan_stamps": nonmonotonic_stamps,
        "duration_sec": stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0,
        "scan_time_min_sec": min(scan_times) if scan_times else None,
        "scan_time_median_sec": percentile(scan_times, 0.5),
        "scan_time_max_sec": max(scan_times) if scan_times else None,
        "time_increment_abs_max_sec": (
            max(abs(value) for value in time_increments)
            if time_increments else None
        ),
        "finite_bins_min": min(finite_counts) if finite_counts else None,
        "finite_bins_median": percentile(finite_counts, 0.5),
        "finite_bins_p95": percentile(finite_counts, 0.95),
        "finite_bins_max": max(finite_counts) if finite_counts else None,
        "isolated_return_count": isolated_return_count,
        "isolated_fraction": (
            isolated_return_count / finite_return_count
            if finite_return_count else None
        ),
    }
    failures = []
    if not scan_count:
        failures.append("no /scan messages")
    if frames != {"base_footprint"}:
        failures.append("unexpected frame_id")
    if nonmonotonic_stamps:
        failures.append("scan timestamps are not strictly increasing")
    if any(abs(value) > 1e-12 for value in time_increments):
        failures.append("deskewed scan has non-zero time_increment")
    if finite_counts and min(finite_counts) < 30:
        failures.append("scan has fewer than 30 finite bins")
    result["failures"] = failures
    result["passed"] = not failures

    output = json.dumps(result, indent=2, sort_keys=True)
    print(output)
    if args.output:
        args.output.write_text(output + "\n", encoding="utf-8")
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
