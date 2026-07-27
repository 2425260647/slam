#!/usr/bin/env python3

import argparse
import json
import math
import os
import statistics
from collections import defaultdict

import rosbag


REQUIRED_TOPICS = ("/velodyne_points", "/odom")
RECOMMENDED_TOPICS = ("/tf", "/tf_static", "/cmd_vel", "/scout_status")


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    ratio = position - lower
    return ordered[lower] * (1.0 - ratio) + ordered[upper] * ratio


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def decode_header_value(value):
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def longest_sparse_duration(times, sparse_flags):
    longest = 0.0
    start = None
    previous = None
    for timestamp, sparse in zip(times, sparse_flags):
        if sparse:
            if start is None or (previous is not None and timestamp - previous > 0.5):
                start = timestamp
            longest = max(longest, timestamp - start)
        else:
            start = None
        previous = timestamp
    return longest


def summarize_timing(times):
    if len(times) < 2:
        return {"count": len(times), "frequency_hz": None, "max_gap_s": None,
                "non_monotonic_count": 0}
    gaps = [current - previous for previous, current in zip(times, times[1:])]
    positive_gaps = [gap for gap in gaps if gap > 0.0]
    duration = times[-1] - times[0]
    return {
        "count": len(times),
        "frequency_hz": (len(times) - 1) / duration if duration > 0.0 else None,
        "max_gap_s": max(positive_gaps) if positive_gaps else None,
        "p95_gap_s": percentile(positive_gaps, 0.95),
        "non_monotonic_count": sum(gap <= 0.0 for gap in gaps),
    }


def audit_bag(path):
    result = {
        "path": os.path.abspath(path),
        "file_size_bytes": os.path.getsize(path) if os.path.exists(path) else None,
        "readable": False,
        "errors": [],
        "warnings": [],
    }
    if not os.path.exists(path):
        result["errors"].append("file does not exist")
        result["accepted_for_slam"] = False
        return result
    try:
        bag = rosbag.Bag(path, "r")
    except Exception as error:  # rosbag uses several exception subclasses.
        result["errors"].append(str(error))
        return result

    with bag:
        result["readable"] = True
        result["start_time"] = bag.get_start_time()
        result["end_time"] = bag.get_end_time()
        result["duration_s"] = bag.get_end_time() - bag.get_start_time()
        topic_info = bag.get_type_and_topic_info().topics
        result["topics"] = {
            topic: {"type": info.msg_type, "messages": info.message_count}
            for topic, info in topic_info.items()
        }

        for topic in REQUIRED_TOPICS:
            if topic not in topic_info:
                result["errors"].append("missing required topic: " + topic)
        for topic in RECOMMENDED_TOPICS:
            if topic not in topic_info:
                result["warnings"].append("missing recommended topic: " + topic)

        topic_times = defaultdict(list)
        cloud_widths = []
        odom_poses = []
        odom_twists = []
        tf_pairs = defaultdict(set)
        static_transforms = []

        selected_topics = [
            topic for topic in ("/velodyne_points", "/odom", "/tf", "/tf_static")
            if topic in topic_info
        ]
        for topic, message, bag_time, connection in bag.read_messages(
                topics=selected_topics, return_connection_header=True):
            timestamp = bag_time.to_sec()
            if hasattr(message, "header") and message.header.stamp.to_sec() > 0.0:
                timestamp = message.header.stamp.to_sec()
            topic_times[topic].append(timestamp)

            if topic == "/velodyne_points":
                cloud_widths.append(int(message.width) * int(message.height))
            elif topic == "/odom":
                position = message.pose.pose.position
                yaw = yaw_from_quaternion(message.pose.pose.orientation)
                odom_poses.append((timestamp, position.x, position.y, yaw))
                odom_twists.append((timestamp, message.twist.twist.linear.x,
                                    message.twist.twist.angular.z))
            elif topic in ("/tf", "/tf_static"):
                caller = decode_header_value(
                    connection.get("callerid", "unknown")) if connection else "unknown"
                for transform in message.transforms:
                    parent = transform.header.frame_id.lstrip("/")
                    child = transform.child_frame_id.lstrip("/")
                    tf_pairs[(parent, child)].add(caller)
                    if topic == "/tf_static":
                        translation = transform.transform.translation
                        rotation = transform.transform.rotation
                        static_transforms.append({
                            "parent": parent,
                            "child": child,
                            "translation": [translation.x, translation.y, translation.z],
                            "yaw_rad": yaw_from_quaternion(rotation),
                            "caller": caller,
                        })

        result["timing"] = {
            topic: summarize_timing(times) for topic, times in topic_times.items()
        }

        if cloud_widths:
            median_width = statistics.median(cloud_widths)
            sparse_limit = 0.2 * median_width
            sparse_flags = [width < sparse_limit for width in cloud_widths]
            result["point_cloud"] = {
                "minimum_points": min(cloud_widths),
                "p01_points": percentile(cloud_widths, 0.01),
                "p05_points": percentile(cloud_widths, 0.05),
                "median_points": median_width,
                "maximum_points": max(cloud_widths),
                "sparse_limit_points": sparse_limit,
                "sparse_frame_count": sum(sparse_flags),
                "longest_sparse_interval_s": longest_sparse_duration(
                    topic_times["/velodyne_points"], sparse_flags),
            }

        if len(odom_poses) >= 2:
            translations = []
            rotations = []
            total_path = 0.0
            for previous, current in zip(odom_poses, odom_poses[1:]):
                translation = math.hypot(current[1] - previous[1], current[2] - previous[2])
                rotation = abs(wrap_angle(current[3] - previous[3]))
                translations.append(translation)
                rotations.append(rotation)
                total_path += translation
            initial = odom_poses[0]
            final = odom_poses[-1]
            result["odometry"] = {
                "total_path_m": total_path,
                "net_displacement_m": math.hypot(final[1] - initial[1], final[2] - initial[2]),
                "net_yaw_rad": wrap_angle(final[3] - initial[3]),
                "maximum_step_translation_m": max(translations),
                "p99_step_translation_m": percentile(translations, 0.99),
                "maximum_step_rotation_rad": max(rotations),
                "p99_step_rotation_rad": percentile(rotations, 0.99),
                "step_translation_over_0_1_m": sum(value > 0.1 for value in translations),
                "step_rotation_over_0_1_rad": sum(value > 0.1 for value in rotations),
            }

        if odom_twists:
            start = odom_twists[0][0]
            end = odom_twists[-1][0]
            for name, samples in (
                    ("initial_10s", [item for item in odom_twists if item[0] <= start + 10.0]),
                    ("final_10s", [item for item in odom_twists if item[0] >= end - 10.0])):
                if samples:
                    result.setdefault("stationary_check", {})[name] = {
                        "mean_abs_linear_mps": statistics.mean(abs(item[1]) for item in samples),
                        "mean_abs_angular_radps": statistics.mean(abs(item[2]) for item in samples),
                        "max_abs_linear_mps": max(abs(item[1]) for item in samples),
                        "max_abs_angular_radps": max(abs(item[2]) for item in samples),
                    }

        result["tf"] = {
            "pairs_and_publishers": {
                parent + " -> " + child: sorted(callers)
                for (parent, child), callers in sorted(tf_pairs.items())
            },
            "static_transforms": static_transforms,
        }

        cloud_timing = result.get("timing", {}).get("/velodyne_points", {})
        odom_timing = result.get("timing", {}).get("/odom", {})
        if cloud_timing.get("frequency_hz") is not None and not 8.0 <= cloud_timing["frequency_hz"] <= 12.0:
            result["warnings"].append("point-cloud frequency outside 8-12 Hz")
        if odom_timing.get("frequency_hz") is not None and not 40.0 <= odom_timing["frequency_hz"] <= 60.0:
            result["warnings"].append("odometry frequency outside 40-60 Hz")
        if cloud_timing.get("max_gap_s") is not None and cloud_timing["max_gap_s"] > 0.3:
            result["warnings"].append("point-cloud gap exceeds 0.3 s")
        if odom_timing.get("max_gap_s") is not None and odom_timing["max_gap_s"] > 0.1:
            result["warnings"].append("odometry gap exceeds 0.1 s")
        if result.get("point_cloud", {}).get("longest_sparse_interval_s", 0.0) > 1.0:
            result["warnings"].append("point cloud stays below 20% of median for more than 1 s")
        if result.get("odometry", {}).get("step_translation_over_0_1_m", 0) > 0:
            result["warnings"].append("odometry contains a translation step over 0.1 m")
        if result.get("odometry", {}).get("step_rotation_over_0_1_rad", 0) > 0:
            result["warnings"].append("odometry contains a rotation step over 0.1 rad")

    result["accepted_for_slam"] = result["readable"] and not result["errors"]
    return result


def format_number(value, digits=3):
    if value is None:
        return "N/A"
    return f"{value:.{digits}f}"


def write_markdown(results, output_path):
    lines = ["# Formal Corridor Bag Audit", ""]
    lines.append("| Bag | Readable | Duration (s) | Cloud Hz | Odom Hz | Path (m) | Net displacement (m) | Result |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---|")
    for result in results:
        name = os.path.basename(result["path"])
        timing = result.get("timing", {})
        odometry = result.get("odometry", {})
        status = "PASS" if result.get("accepted_for_slam") else "FAIL"
        lines.append(
            f"| {name} | {result['readable']} | {format_number(result.get('duration_s'), 1)} "
            f"| {format_number(timing.get('/velodyne_points', {}).get('frequency_hz'))} "
            f"| {format_number(timing.get('/odom', {}).get('frequency_hz'))} "
            f"| {format_number(odometry.get('total_path_m'))} "
            f"| {format_number(odometry.get('net_displacement_m'))} | {status} |"
        )
    lines.append("")
    for result in results:
        lines.extend(["## " + os.path.basename(result["path"]), ""])
        if result["errors"]:
            lines.append("Errors: " + "; ".join(result["errors"]))
        if result["warnings"]:
            lines.append("Warnings: " + "; ".join(result["warnings"]))
        if not result["errors"] and not result["warnings"]:
            lines.append("No automatic audit warnings.")
        lines.append("")
        lines.append("```json")
        lines.append(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
        lines.append("```")
        lines.append("")
    with open(output_path, "w", encoding="utf-8") as output:
        output.write("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description="Audit formal corridor ROS bags.")
    parser.add_argument("bags", nargs="+", help="Bag paths")
    parser.add_argument("--json", dest="json_path", required=True)
    parser.add_argument("--markdown", dest="markdown_path", required=True)
    args = parser.parse_args()

    results = [audit_bag(path) for path in args.bags]
    os.makedirs(os.path.dirname(os.path.abspath(args.json_path)), exist_ok=True)
    with open(args.json_path, "w", encoding="utf-8") as output:
        json.dump(results, output, ensure_ascii=False, indent=2, sort_keys=True)
    write_markdown(results, args.markdown_path)
    print(json.dumps(results, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
