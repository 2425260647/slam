#!/usr/bin/env python3
"""Generate deterministic Innovation 2 pressure bags.

Two modes are intentionally separated:

* odom_pressure: injects same-sign local-frame SE(2) increments and reintegrates
  nav_msgs/Odometry.pose.pose. This tests whether dynamic backend odometry
  weighting protects the map from accumulated wheel-odometry inconsistency.
* lidar_gate: applies a smooth, temporary planar rigid transform to PointCloud2
  x/y coordinates while leaving odometry untouched. Events are placed in
  low-D_conf intervals observed in all three clean recordings. This tests
  whether the D_conf gate suppresses incorrect odometry down-weighting when the
  LiDAR reference is unreliable.

The generated data are controlled stress tests, not physical wheel-slip or
LiDAR-failure measurements.
"""

import argparse
import copy
import hashlib
import json
import math
import os

import numpy as np
import rosbag


ODOM_TOPIC = "/odom"
POINT_CLOUD_TOPIC = "/velodyne_points"


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


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


def pose2d(message):
    pose = message.pose.pose
    return pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)


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


def pressure_events(lateral, yaw, experiment_version):
    # Version 1 is retained verbatim as a rejected calibration candidate.
    if experiment_version == 1:
        windows = [(55.0, 58.0), (230.0, 233.0)]
    else:
        # Version 2 uses four shorter windows that have high D_conf in all
        # three clean recordings. Concentrating the same per-event drift over
        # two seconds makes the injected inconsistency visible between
        # adjacent trajectory nodes instead of only in high-rate odom frames.
        windows = [
            (55.0, 57.0),
            (70.0, 72.0),
            (220.0, 222.0),
            (230.0, 232.0),
        ]
    return [
        {
            "event_id": "odom_pressure_{:02d}".format(index + 1),
            "start_sec": start_sec,
            "end_sec": end_sec,
            "target_lateral_m": lateral,
            "target_yaw_rad": yaw,
            "profile": "sin_squared_rate",
        }
        for index, (start_sec, end_sec) in enumerate(windows)
    ]


def lidar_gate_events(lateral, yaw):
    # These windows are inside common low-D_conf intervals of all clean bags.
    return [
        {
            "event_id": "lidar_gate_stress_01",
            "start_sec": 130.0,
            "end_sec": 134.0,
            "peak_lateral_m": lateral,
            "peak_yaw_rad": yaw,
            "profile": "sin_squared_offset",
        },
        {
            "event_id": "lidar_gate_stress_02",
            "start_sec": 440.0,
            "end_sec": 444.0,
            "peak_lateral_m": lateral,
            "peak_yaw_rad": yaw,
            "profile": "sin_squared_offset",
        },
    ]


def pressure_rates(elapsed, events):
    lateral_rate = 0.0
    yaw_rate = 0.0
    active_ids = []
    for event in events:
        if event["start_sec"] <= elapsed <= event["end_sec"]:
            duration = event["end_sec"] - event["start_sec"]
            progress = (elapsed - event["start_sec"]) / duration
            profile = math.sin(math.pi * progress) ** 2
            lateral_rate += 2.0 * event["target_lateral_m"] / duration * profile
            yaw_rate += 2.0 * event["target_yaw_rad"] / duration * profile
            active_ids.append(event["event_id"])
    return lateral_rate, yaw_rate, active_ids


def lidar_offset(elapsed, events):
    lateral = 0.0
    yaw = 0.0
    active_ids = []
    for event in events:
        if event["start_sec"] <= elapsed <= event["end_sec"]:
            duration = event["end_sec"] - event["start_sec"]
            progress = (elapsed - event["start_sec"]) / duration
            profile = math.sin(math.pi * progress) ** 2
            lateral += event["peak_lateral_m"] * profile
            yaw += event["peak_yaw_rad"] * profile
            active_ids.append(event["event_id"])
    return lateral, yaw, active_ids


def transform_point_cloud_xy(message, lateral, yaw):
    if message.is_bigendian:
        raise ValueError("Big-endian PointCloud2 is not supported by this experiment")
    fields = {field.name: field for field in message.fields}
    for name in ("x", "y"):
        if name not in fields or fields[name].datatype != 7 or fields[name].count != 1:
            raise ValueError("PointCloud2 x/y must be scalar FLOAT32 fields")

    result = copy.deepcopy(message)
    buffer = bytearray(result.data)
    point_count = result.width * result.height
    dtype = np.dtype(
        {
            "names": ["x", "y"],
            "formats": ["<f4", "<f4"],
            "offsets": [fields["x"].offset, fields["y"].offset],
            "itemsize": result.point_step,
        }
    )
    points = np.frombuffer(buffer, dtype=dtype, count=point_count)
    x = points["x"].copy()
    y = points["y"].copy()
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    points["x"] = cosine * x - sine * y
    points["y"] = sine * x + cosine * y + lateral
    result.data = bytes(buffer)
    return result


def validate_events(events, duration):
    for event in events:
        if event["start_sec"] < 0.0 or event["end_sec"] > duration:
            raise ValueError("Event outside bag duration: {}".format(event))
        if event["end_sec"] <= event["start_sec"]:
            raise ValueError("Invalid event duration: {}".format(event))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--labels", required=True)
    parser.add_argument("--mode", choices=("odom_pressure", "lidar_gate"), required=True)
    parser.add_argument("--variant", type=int, choices=(1, 2, 3), required=True)
    parser.add_argument("--experiment-version", type=int, choices=(1, 2), default=1)
    parser.add_argument("--pressure-lateral-m", type=float, default=0.35)
    parser.add_argument("--pressure-yaw-rad", type=float, default=0.10)
    parser.add_argument("--lidar-peak-lateral-m", type=float, default=0.30)
    parser.add_argument("--lidar-peak-yaw-rad", type=float, default=0.15)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    if os.path.abspath(args.input) == os.path.abspath(args.output):
        raise SystemExit("Input and output bag paths must differ")
    if os.path.exists(args.output) and not args.overwrite:
        raise SystemExit("Output already exists: {}".format(args.output))

    if args.mode == "odom_pressure":
        events = pressure_events(
            args.pressure_lateral_m,
            args.pressure_yaw_rad,
            args.experiment_version,
        )
    else:
        events = lidar_gate_events(
            args.lidar_peak_lateral_m, args.lidar_peak_yaw_rad
        )

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.labels)), exist_ok=True)
    event_sample_counts = {event["event_id"]: 0 for event in events}
    integrated_lateral = 0.0
    integrated_yaw = 0.0
    odom_count = 0
    cloud_count = 0
    modified_cloud_count = 0

    with rosbag.Bag(args.input, "r") as source:
        bag_start = source.get_start_time()
        bag_end = source.get_end_time()
        duration = bag_end - bag_start
        validate_events(events, duration)
        previous_original = None
        previous_modified = None
        previous_stamp = None

        with rosbag.Bag(args.output, "w", compression=rosbag.Compression.LZ4) as output:
            for topic, message, bag_time in source.read_messages(
                topics=[ODOM_TOPIC, POINT_CLOUD_TOPIC]
            ):
                if topic == ODOM_TOPIC:
                    odom_count += 1
                    if args.mode != "odom_pressure":
                        output.write(topic, message, bag_time)
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
                        lateral_rate, yaw_rate, active_ids = pressure_rates(
                            elapsed_midpoint, events
                        )
                        delta = local_delta(previous_original, current_original)
                        modified_delta = (
                            delta[0],
                            delta[1] + lateral_rate * dt,
                            normalize_angle(delta[2] + yaw_rate * dt),
                        )
                        previous_modified = integrate_local_delta(
                            previous_modified, modified_delta
                        )
                        integrated_lateral += lateral_rate * dt
                        integrated_yaw += yaw_rate * dt
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
                    continue

                cloud_count += 1
                if args.mode != "lidar_gate":
                    output.write(topic, message, bag_time)
                    continue
                elapsed = message.header.stamp.to_sec() - bag_start
                lateral, yaw, active_ids = lidar_offset(elapsed, events)
                if active_ids:
                    message = transform_point_cloud_xy(message, lateral, yaw)
                    modified_cloud_count += 1
                    for event_id in active_ids:
                        event_sample_counts[event_id] += 1
                output.write(topic, message, bag_time)

    for event in events:
        event["duration_sec"] = event["end_sec"] - event["start_sec"]
        event["absolute_start_sec"] = bag_start + event["start_sec"]
        event["absolute_end_sec"] = bag_start + event["end_sec"]
        event["samples"] = event_sample_counts[event["event_id"]]

    labels = {
        "schema_version": 1,
        "mode": args.mode,
        "variant": args.variant,
        "experiment_version": args.experiment_version,
        "source_bag": os.path.abspath(args.input),
        "source_bag_sha256": sha256_file(args.input),
        "output_bag": os.path.abspath(args.output),
        "bag_start_sec": bag_start,
        "bag_end_sec": bag_end,
        "duration_sec": duration,
        "events": events,
        "event_count": len(events),
        "odom_messages": odom_count,
        "point_cloud_messages": cloud_count,
        "modified_point_cloud_messages": modified_cloud_count,
        "integrated_signed_lateral_m": integrated_lateral,
        "integrated_signed_yaw_rad": integrated_yaw,
        "parameters": {
            "pressure_lateral_m_per_event": args.pressure_lateral_m,
            "pressure_yaw_rad_per_event": args.pressure_yaw_rad,
            "lidar_peak_lateral_m": args.lidar_peak_lateral_m,
            "lidar_peak_yaw_rad": args.lidar_peak_yaw_rad,
        },
        "method": (
            "Local-frame SE(2) odometry increment reintegration"
            if args.mode == "odom_pressure"
            else "Temporary rigid x/y transform of PointCloud2 in laser_link"
        ),
    }
    labels["output_bag_sha256"] = sha256_file(args.output)
    with open(args.labels, "w", encoding="utf-8") as stream:
        json.dump(labels, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(labels, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
