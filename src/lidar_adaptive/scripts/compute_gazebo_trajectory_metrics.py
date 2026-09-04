#!/usr/bin/env python3
"""Compute aligned ATE/RPE from Gazebo model truth and Cartographer TF."""

import argparse
import bisect
import json
import math
import statistics

import rosbag


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(first, second):
    x1, y1, yaw1 = first
    x2, y2, yaw2 = second
    c = math.cos(yaw1)
    s = math.sin(yaw1)
    return (x1 + c * x2 - s * y2,
            y1 + s * x2 + c * y2,
            math.atan2(math.sin(yaw1 + yaw2), math.cos(yaw1 + yaw2)))


def inverse(pose):
    x, y, yaw = pose
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (-c * x - s * y, s * x - c * y, -yaw)


def relative(origin, pose):
    return compose(inverse(origin), pose)


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(math.ceil(fraction * len(ordered))) - 1))
    return ordered[index]


def nearest_at_or_before(stamps, values, stamp):
    index = bisect.bisect_right(stamps, stamp) - 1
    return values[index] if index >= 0 else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--model", default="scout_mini")
    parser.add_argument("--rpe-seconds", type=float, default=1.0)
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    truth = []
    tf_updates = []
    map_odom = None
    odom_base = None
    with rosbag.Bag(args.bag, "r") as bag:
        for topic, message, timestamp in bag.read_messages(
                topics=["/gazebo/model_states", "/tf"]):
            stamp = timestamp.to_sec()
            if topic == "/gazebo/model_states":
                try:
                    index = message.name.index(args.model)
                except ValueError:
                    continue
                pose = message.pose[index]
                truth.append((stamp, (pose.position.x, pose.position.y,
                                      yaw_from_quaternion(pose.orientation))))
                continue
            for transform in message.transforms:
                parent = transform.header.frame_id.lstrip("/")
                child = transform.child_frame_id.lstrip("/")
                value = (transform.transform.translation.x,
                         transform.transform.translation.y,
                         yaw_from_quaternion(transform.transform.rotation))
                if parent == "map" and child == "odom":
                    map_odom = value
                elif parent == "odom" and child == "base_link":
                    odom_base = value
            if map_odom is not None and odom_base is not None:
                tf_updates.append((stamp, compose(map_odom, odom_base)))

    truth.sort(key=lambda item: item[0])
    tf_updates.sort(key=lambda item: item[0])
    tf_stamps = [item[0] for item in tf_updates]
    tf_poses = [item[1] for item in tf_updates]
    paired = []
    for stamp, truth_pose in truth:
        estimate = nearest_at_or_before(tf_stamps, tf_poses, stamp)
        if estimate is not None:
            paired.append((stamp, truth_pose, estimate))

    if not paired:
        raise SystemExit("no paired Gazebo truth and map->odom->base_link TF samples")

    truth_origin = paired[0][1]
    estimate_origin = paired[0][2]
    aligned = [(stamp, relative(truth_origin, truth_pose),
                relative(estimate_origin, estimate))
               for stamp, truth_pose, estimate in paired]
    translation_errors = []
    rotation_errors = []
    for _, truth_pose, estimate in aligned:
        error = relative(truth_pose, estimate)
        translation_errors.append(math.hypot(error[0], error[1]))
        rotation_errors.append(abs(math.atan2(math.sin(error[2]), math.cos(error[2]))))

    rpe_translation = []
    rpe_rotation = []
    rpe_delta = max(0.01, args.rpe_seconds)
    aligned_stamps = [item[0] for item in aligned]
    for index, (stamp, truth_pose, estimate) in enumerate(aligned):
        target = stamp + rpe_delta
        next_index = bisect.bisect_left(aligned_stamps, target, lo=index + 1)
        if next_index >= len(aligned):
            continue
        truth_relative = relative(truth_pose, aligned[next_index][1])
        estimate_relative = relative(estimate, aligned[next_index][2])
        error = relative(truth_relative, estimate_relative)
        rpe_translation.append(math.hypot(error[0], error[1]))
        rpe_rotation.append(abs(math.atan2(math.sin(error[2]), math.cos(error[2]))))

    result = {
        "bag": args.bag,
        "model": args.model,
        "truth_source": "/gazebo/model_states",
        "estimate_source": "/tf map->odom->base_link",
        "paired_samples": len(aligned),
        "truth_samples": len(truth),
        "rpe_seconds": rpe_delta,
        "ate_translation_rmse": math.sqrt(statistics.mean(v * v for v in translation_errors)),
        "ate_translation_p95": percentile(translation_errors, 0.95),
        "ate_rotation_rmse_rad": math.sqrt(statistics.mean(v * v for v in rotation_errors)),
        "rpe_translation_rmse": (math.sqrt(statistics.mean(v * v for v in rpe_translation))
                                  if rpe_translation else None),
        "rpe_rotation_rmse_rad": (math.sqrt(statistics.mean(v * v for v in rpe_rotation))
                                   if rpe_rotation else None),
        "ground_truth_available": True,
    }
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        with open(args.output, "w", encoding="ascii") as stream:
            stream.write(rendered)
    else:
        print(rendered, end="")


if __name__ == "__main__":
    main()
