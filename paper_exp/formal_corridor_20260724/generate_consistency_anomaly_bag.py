#!/usr/bin/env python3
"""Generate reproducible odometry-consistency anomaly bags for formal tests.

Cartographer consumes nav_msgs/Odometry.pose.pose in this repository.  The
generator therefore perturbs consecutive local-frame SE(2) pose increments and
reintegrates the absolute pose.  Twist is updated as well, but only to keep the
message semantics internally consistent.
"""

import argparse
import copy
import hashlib
import json
import math
import os

import rosbag


ODOM_TOPIC = "/odom"
POINT_CLOUD_TOPIC = "/velodyne_points"


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def set_planar_quaternion(quaternion, yaw):
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(0.5 * yaw)
    quaternion.w = math.cos(0.5 * yaw)


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build_events(duration, variant):
    """Return seven separated, deterministic events for one recording."""
    templates = [
        ("mixed_light", 1.2, 0.22, 0.06),
        ("lateral_moderate", 1.2, -0.30, 0.0),
        ("yaw_moderate", 0.6, 0.0, -0.12),
        ("mixed_moderate", 1.5, 0.35, 0.10),
        ("mixed_light", 1.2, -0.22, -0.06),
        ("lateral_moderate", 1.2, -0.30, 0.0),
        ("yaw_moderate", 0.6, 0.0, 0.12),
    ]
    # The detector is intentionally valid only where the directional LiDAR
    # reference is reliable.  These route fractions correspond to the two
    # long-corridor traversals, not the elevator-hall/open transition zones.
    fractions = [0.09, 0.16, 0.38, 0.44, 0.62, 0.68, 0.88]
    # A small deterministic phase offset prevents all three bags from using
    # exactly the same elapsed timestamps while retaining the same intensities.
    offset = (variant - 2) * 1.5
    events = []
    for index, (template, fraction) in enumerate(zip(templates, fractions), 1):
        event_type, event_duration, lateral, yaw = template
        center = fraction * duration + offset
        start = max(10.0, center - 0.5 * event_duration)
        end = min(duration - 10.0, start + event_duration)
        events.append(
            {
                "event_id": "repeat_{:02d}_event_{:02d}".format(variant, index),
                "type": event_type,
                "start_sec": start,
                "end_sec": end,
                "duration_sec": end - start,
                "target_lateral_m": lateral,
                "target_yaw_rad": yaw,
                "profile": "sin_squared_rate",
            }
        )
    return events


def event_rates(elapsed, events):
    lateral_rate = 0.0
    yaw_rate = 0.0
    active_ids = []
    for event in events:
        if event["start_sec"] <= elapsed <= event["end_sec"]:
            duration = event["duration_sec"]
            progress = (elapsed - event["start_sec"]) / duration
            profile = math.sin(math.pi * progress) ** 2
            lateral_rate += (
                2.0 * event["target_lateral_m"] / duration * profile
            )
            yaw_rate += 2.0 * event["target_yaw_rad"] / duration * profile
            active_ids.append(event["event_id"])
    return lateral_rate, yaw_rate, active_ids


def pose2d(message):
    pose = message.pose.pose
    return (
        pose.position.x,
        pose.position.y,
        yaw_from_quaternion(pose.orientation),
    )


def local_delta(previous, current):
    dx_world = current[0] - previous[0]
    dy_world = current[1] - previous[1]
    cosine = math.cos(previous[2])
    sine = math.sin(previous[2])
    return (
        cosine * dx_world + sine * dy_world,
        -sine * dx_world + cosine * dy_world,
        normalize_angle(current[2] - previous[2]),
    )


def integrate_local_delta(previous, delta):
    cosine = math.cos(previous[2])
    sine = math.sin(previous[2])
    return (
        previous[0] + cosine * delta[0] - sine * delta[1],
        previous[1] + sine * delta[0] + cosine * delta[1],
        normalize_angle(previous[2] + delta[2]),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--labels", required=True)
    parser.add_argument("--variant", type=int, choices=(1, 2, 3), required=True)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    if os.path.exists(args.output) and not args.overwrite:
        raise SystemExit("Output already exists: {}".format(args.output))
    if os.path.abspath(args.input) == os.path.abspath(args.output):
        raise SystemExit("Input and output bag paths must differ")

    with rosbag.Bag(args.input, "r") as source:
        bag_start = source.get_start_time()
        bag_end = source.get_end_time()
        duration = bag_end - bag_start
        events = build_events(duration, args.variant)

        previous_original = None
        previous_modified = None
        previous_stamp = None
        odom_count = 0
        point_cloud_count = 0
        injected_lateral = 0.0
        injected_yaw = 0.0
        event_sample_counts = {event["event_id"]: 0 for event in events}

        with rosbag.Bag(args.output, "w", compression=rosbag.Compression.LZ4) as output:
            for topic, message, bag_time in source.read_messages(
                topics=[ODOM_TOPIC, POINT_CLOUD_TOPIC]
            ):
                if topic == POINT_CLOUD_TOPIC:
                    output.write(topic, message, bag_time)
                    point_cloud_count += 1
                    continue

                modified = copy.deepcopy(message)
                current_original = pose2d(message)
                stamp = message.header.stamp.to_sec()
                if previous_original is None:
                    previous_original = current_original
                    previous_modified = current_original
                    previous_stamp = stamp
                else:
                    dt = max(0.0, stamp - previous_stamp)
                    elapsed_midpoint = 0.5 * (stamp + previous_stamp) - bag_start
                    lateral_rate, yaw_rate, active_ids = event_rates(
                        elapsed_midpoint, events
                    )
                    original_delta = local_delta(previous_original, current_original)
                    modified_delta = (
                        original_delta[0],
                        original_delta[1] + lateral_rate * dt,
                        normalize_angle(original_delta[2] + yaw_rate * dt),
                    )
                    previous_modified = integrate_local_delta(
                        previous_modified, modified_delta
                    )
                    injected_lateral += lateral_rate * dt
                    injected_yaw += yaw_rate * dt
                    for event_id in active_ids:
                        event_sample_counts[event_id] += 1

                    modified.pose.pose.position.x = previous_modified[0]
                    modified.pose.pose.position.y = previous_modified[1]
                    set_planar_quaternion(
                        modified.pose.pose.orientation, previous_modified[2]
                    )
                    modified.twist.twist.linear.y += lateral_rate
                    modified.twist.twist.angular.z += yaw_rate
                    previous_original = current_original
                    previous_stamp = stamp

                output.write(topic, modified, bag_time)
                odom_count += 1

    for event in events:
        event["absolute_start_sec"] = bag_start + event["start_sec"]
        event["absolute_end_sec"] = bag_start + event["end_sec"]
        event["odom_samples"] = event_sample_counts[event["event_id"]]

    labels = {
        "schema_version": 1,
        "source_bag": os.path.abspath(args.input),
        "source_bag_sha256": sha256_file(args.input),
        "output_bag": os.path.abspath(args.output),
        "output_bag_sha256": sha256_file(args.output),
        "variant": args.variant,
        "bag_start_sec": bag_start,
        "bag_end_sec": bag_end,
        "duration_sec": duration,
        "topics": [ODOM_TOPIC, POINT_CLOUD_TOPIC],
        "odom_messages": odom_count,
        "point_cloud_messages": point_cloud_count,
        "event_count": len(events),
        "events": events,
        "integrated_signed_lateral_m": injected_lateral,
        "integrated_signed_yaw_rad": injected_yaw,
        "method": (
            "Perturb local-frame consecutive SE(2) odometry increments, then "
            "reintegrate pose; synchronize twist for message consistency."
        ),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.labels)), exist_ok=True)
    with open(args.labels, "w", encoding="utf-8") as stream:
        json.dump(labels, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(labels, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
