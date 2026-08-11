#!/usr/bin/env python3
"""Evaluate bag odometry against Gazebo model pose without replaying ROS."""

import argparse
import bisect
import json
import math
import statistics

import rosbag


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z),
    )


def pose_tuple(pose):
    return pose.position.x, pose.position.y, yaw(pose.orientation)


def relative(origin, pose):
    dx = pose[0] - origin[0]
    dy = pose[1] - origin[1]
    cosine = math.cos(origin[2])
    sine = math.sin(origin[2])
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        normalize(pose[2] - origin[2]),
    )


def percentile(values, fraction):
    values = sorted(values)
    index = max(0, min(len(values) - 1,
                       int(math.ceil(fraction * len(values))) - 1))
    return values[index]


def metric(values):
    return {
        "rmse": math.sqrt(statistics.mean(value * value for value in values)),
        "mean": statistics.mean(values),
        "p95": percentile(values, 0.95),
        "maximum": max(values),
        "final": values[-1],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--odom-topic",
                        default="/scout_mini_velocity_controller/odom")
    parser.add_argument("--truth-topic", default="/gazebo/model_states")
    parser.add_argument("--model-name", default="scout_mini")
    parser.add_argument("--max-time-delta", type=float, default=0.08)
    arguments = parser.parse_args()

    truth = []
    estimates = []
    with rosbag.Bag(arguments.bag) as bag:
        for topic, message, stamp in bag.read_messages(
                topics=[arguments.truth_topic, arguments.odom_topic]):
            if topic == arguments.truth_topic:
                try:
                    index = message.name.index(arguments.model_name)
                except ValueError:
                    continue
                truth.append((stamp.to_sec(), pose_tuple(message.pose[index])))
            else:
                sample_stamp = message.header.stamp.to_sec()
                if sample_stamp <= 0.0:
                    sample_stamp = stamp.to_sec()
                estimates.append((sample_stamp, pose_tuple(message.pose.pose)))

    truth_times = [sample[0] for sample in truth]
    associated = []
    for stamp, estimate in estimates:
        index = bisect.bisect_left(truth_times, stamp)
        candidates = truth[max(0, index - 1):min(len(truth), index + 1)]
        if not candidates:
            continue
        truth_stamp, truth_pose = min(
            candidates, key=lambda sample: abs(sample[0] - stamp))
        delta = abs(truth_stamp - stamp)
        if delta <= arguments.max_time_delta:
            associated.append((delta, truth_pose, estimate))

    if not associated:
        raise RuntimeError("no associated odometry/truth samples")
    truth_origin = associated[0][1]
    estimate_origin = associated[0][2]
    translation_errors = []
    yaw_errors = []
    for _, truth_pose, estimate in associated:
        truth_relative = relative(truth_origin, truth_pose)
        estimate_relative = relative(estimate_origin, estimate)
        translation_errors.append(math.hypot(
            truth_relative[0] - estimate_relative[0],
            truth_relative[1] - estimate_relative[1],
        ))
        yaw_errors.append(abs(normalize(
            truth_relative[2] - estimate_relative[2]
        )))

    print(json.dumps({
        "algorithm": "wheel_odometry",
        "associated_samples": len(associated),
        "max_association_delta_s": max(sample[0] for sample in associated),
        "translation_error_m": metric(translation_errors),
        "yaw_error_rad": metric(yaw_errors),
        "evaluation_mode": "online_pose_start_aligned_se2",
        "truth_used_for_odometry": False,
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
