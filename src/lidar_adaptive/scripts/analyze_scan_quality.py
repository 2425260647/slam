#!/usr/bin/env python3
"""Summarize quality CSV/rosbag exports without claiming localization accuracy."""

import argparse
import csv
import math
import statistics


def values(path):
    result = []
    with open(path, newline="") as stream:
        for row in csv.DictReader(stream):
            for key in ("data", "quality", "value"):
                if key in row:
                    try:
                        value = float(row[key])
                    except ValueError:
                        continue
                    if math.isfinite(value):
                        result.append(value)
                    break
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file")
    args = parser.parse_args()
    data = values(args.csv_file)
    if not data:
        raise SystemExit("no finite values found")
    print("count={}".format(len(data)))
    print("mean={:.6f}".format(statistics.mean(data)))
    print("median={:.6f}".format(statistics.median(data)))
    print("min={:.6f}".format(min(data)))
    print("max={:.6f}".format(max(data)))


if __name__ == "__main__":
    main()
