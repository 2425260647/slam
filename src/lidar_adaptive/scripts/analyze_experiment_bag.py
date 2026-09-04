#!/usr/bin/env python3
"""Summarize lidar_adaptive rosbag evidence without inventing ground truth."""

import argparse
import json
import math
import statistics

import rosbag


def finite_mean(values):
    values = [value for value in values if math.isfinite(value)]
    return statistics.mean(values) if values else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    counts = {}
    quality_values = []
    information_values = []
    projection_values = []
    projection_time_values = []
    selector_time_values = []
    sync_count = 0
    selection_count = 0
    selection_true_count = 0
    stamps = []

    topics = [
        "/velodyne_points", "/scan", "/scan_confidence", "/scan_selected",
        "/lidar_scan_quality", "/scan_selection", "/keyframe_selected",
        "/gazebo/model_states",
        "/scout_mini_velocity_controller/odom",
        "/lidar_projection_processing_ms", "/lidar_selector_processing_ms",
    ]
    with rosbag.Bag(args.bag, "r") as bag:
        for topic, message, _ in bag.read_messages(topics=topics):
            counts[topic] = counts.get(topic, 0) + 1
            if topic == "/lidar_scan_quality":
                quality_values.append(float(message.quality))
            elif topic == "/lidar_projection_processing_ms":
                projection_time_values.append(float(message.data))
            elif topic == "/lidar_selector_processing_ms":
                selector_time_values.append(float(message.data))
            elif topic == "/scan_selection":
                selection_count += 1
                if message.selected:
                    selection_true_count += 1
                if message.quality_synchronized:
                    sync_count += 1
                information_values.append(float(message.information_score))
                projection_values.append(float(message.projection_quality))
                if message.header.stamp.to_sec() > 0:
                    stamps.append(message.header.stamp.to_sec())

    summary = {
        "bag": args.bag,
        "message_counts": counts,
        "selection_messages": selection_count,
        "selected_messages": selection_true_count,
        "selection_fraction": (selection_true_count / selection_count
                                if selection_count else None),
        "quality_sync_messages": sync_count,
        "quality_sync_fraction": (sync_count / selection_count
                                   if selection_count else None),
        "quality_mean": finite_mean(quality_values),
        "information_score_mean": finite_mean(information_values),
        "projection_quality_mean": finite_mean(projection_values),
        "projection_processing_ms_mean": finite_mean(projection_time_values),
        "selector_processing_ms_mean": finite_mean(selector_time_values),
        "selection_stamp_start": min(stamps) if stamps else None,
        "selection_stamp_end": max(stamps) if stamps else None,
        "ground_truth_available": counts.get("/gazebo/model_states", 0) > 0,
        "accuracy_metrics_allowed": counts.get("/gazebo/model_states", 0) > 0,
    }
    rendered = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    if args.output:
        with open(args.output, "w", encoding="ascii") as stream:
            stream.write(rendered)
    else:
        print(rendered, end="")


if __name__ == "__main__":
    main()
