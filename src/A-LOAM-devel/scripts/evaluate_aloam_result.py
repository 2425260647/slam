#!/usr/bin/env python3
import argparse
import json
import math
import os

import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2


def quat_to_rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def load_odom(bag_path, topic):
    rows = []
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            p = msg.pose.pose.position
            r, pitch, yaw = quat_to_rpy(msg.pose.pose.orientation)
            rows.append([msg.header.stamp.to_sec(), p.x, p.y, p.z, r, pitch, yaw])
    return np.asarray(rows, dtype=float)


def summarize_pose(arr):
    if len(arr) < 2:
        return {}
    t = arr[:, 0]
    xyz = arr[:, 1:4]
    rpy = arr[:, 4:7]
    dt = np.diff(t)
    valid = dt > 1e-6
    dxyz = np.diff(xyz, axis=0)
    dxy = np.linalg.norm(dxyz[:, :2], axis=1)
    dz = np.abs(dxyz[:, 2])
    dyaw = np.asarray([abs(angle_diff(arr[i + 1, 6], arr[i, 6])) for i in range(len(arr) - 1)])
    speed_xy = dxy[valid] / dt[valid]
    speed_z = dz[valid] / dt[valid]
    yaw_rate = dyaw[valid] / dt[valid]

    jump_xy = int(np.sum(dxy > 1.0))
    jump_z = int(np.sum(dz > 0.3))
    jump_yaw = int(np.sum(np.rad2deg(dyaw) > 20.0))

    return {
        "count": int(len(arr)),
        "duration_s": float(t[-1] - t[0]),
        "path_length_xy_m": float(np.sum(dxy)),
        "path_length_3d_m": float(np.sum(np.linalg.norm(dxyz, axis=1))),
        "start_xyz_m": xyz[0].round(4).tolist(),
        "end_xyz_m": xyz[-1].round(4).tolist(),
        "z_range_m": float(np.max(xyz[:, 2]) - np.min(xyz[:, 2])),
        "z_final_minus_start_m": float(xyz[-1, 2] - xyz[0, 2]),
        "z_std_m": float(np.std(xyz[:, 2])),
        "max_step_xy_m": float(np.max(dxy)),
        "max_step_z_m": float(np.max(dz)),
        "p95_speed_xy_mps": float(np.percentile(speed_xy, 95)) if len(speed_xy) else 0.0,
        "max_speed_xy_mps": float(np.max(speed_xy)) if len(speed_xy) else 0.0,
        "p95_speed_z_mps": float(np.percentile(speed_z, 95)) if len(speed_z) else 0.0,
        "max_speed_z_mps": float(np.max(speed_z)) if len(speed_z) else 0.0,
        "roll_mean_deg": float(np.rad2deg(np.mean(rpy[:, 0]))),
        "roll_std_deg": float(np.rad2deg(np.std(rpy[:, 0]))),
        "roll_max_abs_deg": float(np.rad2deg(np.max(np.abs(rpy[:, 0])))),
        "pitch_mean_deg": float(np.rad2deg(np.mean(rpy[:, 1]))),
        "pitch_std_deg": float(np.rad2deg(np.std(rpy[:, 1]))),
        "pitch_max_abs_deg": float(np.rad2deg(np.max(np.abs(rpy[:, 1])))),
        "yaw_range_deg": float(np.rad2deg(np.max(np.unwrap(rpy[:, 2])) - np.min(np.unwrap(rpy[:, 2])))),
        "p95_yaw_rate_degps": float(np.rad2deg(np.percentile(yaw_rate, 95))) if len(yaw_rate) else 0.0,
        "max_yaw_rate_degps": float(np.rad2deg(np.max(yaw_rate))) if len(yaw_rate) else 0.0,
        "jump_xy_count_gt_1m": jump_xy,
        "jump_z_count_gt_0p3m": jump_z,
        "jump_yaw_count_gt_20deg": jump_yaw,
    }


def revisit_metrics(arr, min_dt=30.0, close_xy=1.0):
    if len(arr) < 2:
        return {}
    sample_step = max(1, len(arr) // 600)
    s = arr[::sample_step]
    pairs = []
    for i in range(len(s)):
        dt = np.abs(s[i + 1 :, 0] - s[i, 0])
        mask = dt >= min_dt
        if not np.any(mask):
            continue
        idxs = np.where(mask)[0] + i + 1
        dxy = np.linalg.norm(s[idxs, 1:3] - s[i, 1:3], axis=1)
        close = idxs[dxy < close_xy]
        for j in close:
            dz = abs(s[j, 3] - s[i, 3])
            dyaw = abs(angle_diff(s[j, 6], s[i, 6]))
            pairs.append((float(np.linalg.norm(s[j, 1:3] - s[i, 1:3])), float(dz), float(np.rad2deg(dyaw))))
    if not pairs:
        return {
            "pair_count": 0,
            "note": "No trajectory revisits found under the selected threshold.",
        }
    p = np.asarray(pairs)
    return {
        "pair_count": int(len(pairs)),
        "xy_offset_m_median": float(np.median(p[:, 0])),
        "xy_offset_m_p95": float(np.percentile(p[:, 0], 95)),
        "z_offset_m_median": float(np.median(p[:, 1])),
        "z_offset_m_p95": float(np.percentile(p[:, 1], 95)),
        "yaw_offset_deg_median": float(np.median(p[:, 2])),
        "yaw_offset_deg_p95": float(np.percentile(p[:, 2], 95)),
    }


def final_cloud_stats(bag_path, topic):
    last_msg = None
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            last_msg = msg
    if last_msg is None:
        return {}

    pts = []
    for p in pc2.read_points(last_msg, field_names=("x", "y", "z"), skip_nans=True):
        pts.append(p)
    if not pts:
        return {"point_count": 0}
    arr = np.asarray(pts, dtype=float)
    xy = arr[:, :2]
    z = arr[:, 2]
    cell = 0.2
    keys = np.floor(xy / cell).astype(np.int64)
    _, counts = np.unique(keys, axis=0, return_counts=True)
    dense = counts[counts >= 3]
    return {
        "point_count": int(len(arr)),
        "x_range_m": float(np.max(arr[:, 0]) - np.min(arr[:, 0])),
        "y_range_m": float(np.max(arr[:, 1]) - np.min(arr[:, 1])),
        "z_range_m": float(np.max(z) - np.min(z)),
        "z_p01_m": float(np.percentile(z, 1)),
        "z_p99_m": float(np.percentile(z, 99)),
        "occupied_xy_cells_0p2m": int(len(counts)),
        "median_points_per_0p2m_cell": float(np.median(counts)),
        "p95_points_per_0p2m_cell": float(np.percentile(counts, 95)),
        "dense_cell_ratio_ge3": float(len(dense) / len(counts)) if len(counts) else 0.0,
    }


def judgement(mapped, revisit, cloud):
    issues = []
    if mapped.get("jump_xy_count_gt_1m", 0) or mapped.get("jump_z_count_gt_0p3m", 0) or mapped.get("jump_yaw_count_gt_20deg", 0):
        issues.append("trajectory has discontinuous jumps")
    if mapped.get("z_range_m", 0.0) > 1.0:
        issues.append("large Z drift for a ground robot")
    if mapped.get("roll_max_abs_deg", 0.0) > 15.0 or mapped.get("pitch_max_abs_deg", 0.0) > 15.0:
        issues.append("large roll/pitch variation")
    if revisit.get("pair_count", 0) > 0 and revisit.get("z_offset_m_p95", 0.0) > 0.5:
        issues.append("revisited positions are inconsistent in height")
    if cloud.get("point_count", 0) < 1000:
        issues.append("map cloud is too sparse")
    return "pass_with_caution" if not issues else "fail_or_needs_calibration", issues


def write_report(path, result):
    mapped = result["aft_mapped_to_init"]
    odom = result["laser_odom_to_init"]
    revisit = result["revisit_consistency"]
    cloud = result["final_laser_cloud_map"]
    verdict = result["judgement"]["verdict"]
    issues = result["judgement"]["issues"]
    lines = [
        "# A-LOAM mapping_01 Evaluation",
        "",
        "## Conclusion",
        "",
        "- Verdict: `{}`".format(verdict),
        "- Main issues: {}".format(", ".join(issues) if issues else "none obvious from numeric checks"),
        "",
        "## Back-end Mapped Pose `/aft_mapped_to_init`",
        "",
        "- Samples: {}".format(mapped.get("count")),
        "- Duration: {:.2f} s".format(mapped.get("duration_s", 0.0)),
        "- XY path length: {:.3f} m".format(mapped.get("path_length_xy_m", 0.0)),
        "- Z range: {:.3f} m".format(mapped.get("z_range_m", 0.0)),
        "- Z final-start: {:.3f} m".format(mapped.get("z_final_minus_start_m", 0.0)),
        "- Max XY step: {:.3f} m".format(mapped.get("max_step_xy_m", 0.0)),
        "- Max Z step: {:.3f} m".format(mapped.get("max_step_z_m", 0.0)),
        "- Roll max abs: {:.3f} deg".format(mapped.get("roll_max_abs_deg", 0.0)),
        "- Pitch max abs: {:.3f} deg".format(mapped.get("pitch_max_abs_deg", 0.0)),
        "- Jump counts: xy>1m={}, z>0.3m={}, yaw>20deg={}".format(
            mapped.get("jump_xy_count_gt_1m", 0),
            mapped.get("jump_z_count_gt_0p3m", 0),
            mapped.get("jump_yaw_count_gt_20deg", 0),
        ),
        "",
        "## Front-end Odom `/laser_odom_to_init`",
        "",
        "- XY path length: {:.3f} m".format(odom.get("path_length_xy_m", 0.0)),
        "- Z range: {:.3f} m".format(odom.get("z_range_m", 0.0)),
        "- Z final-start: {:.3f} m".format(odom.get("z_final_minus_start_m", 0.0)),
        "- Roll max abs: {:.3f} deg".format(odom.get("roll_max_abs_deg", 0.0)),
        "- Pitch max abs: {:.3f} deg".format(odom.get("pitch_max_abs_deg", 0.0)),
        "",
        "## Revisit Consistency",
        "",
        "- Pair count: {}".format(revisit.get("pair_count", 0)),
        "- XY offset P95: {:.3f} m".format(revisit.get("xy_offset_m_p95", 0.0)),
        "- Z offset P95: {:.3f} m".format(revisit.get("z_offset_m_p95", 0.0)),
        "- Yaw offset P95: {:.3f} deg".format(revisit.get("yaw_offset_deg_p95", 0.0)),
        "",
        "## Final Map Cloud `/laser_cloud_map`",
        "",
        "- Points: {}".format(cloud.get("point_count", 0)),
        "- X/Y/Z range: {:.3f} / {:.3f} / {:.3f} m".format(
            cloud.get("x_range_m", 0.0), cloud.get("y_range_m", 0.0), cloud.get("z_range_m", 0.0)
        ),
        "- Z 1%-99%: {:.3f} to {:.3f} m".format(cloud.get("z_p01_m", 0.0), cloud.get("z_p99_m", 0.0)),
        "- Occupied 0.2m cells: {}".format(cloud.get("occupied_xy_cells_0p2m", 0)),
        "- Dense cell ratio >=3 pts: {:.3f}".format(cloud.get("dense_cell_ratio_ge3", 0.0)),
        "",
    ]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--out-prefix", required=True)
    args = parser.parse_args()

    mapped = load_odom(args.bag, "/aft_mapped_to_init")
    odom = load_odom(args.bag, "/laser_odom_to_init")
    result = {
        "bag": args.bag,
        "aft_mapped_to_init": summarize_pose(mapped),
        "laser_odom_to_init": summarize_pose(odom),
        "revisit_consistency": revisit_metrics(mapped),
        "final_laser_cloud_map": final_cloud_stats(args.bag, "/laser_cloud_map"),
    }
    verdict, issues = judgement(result["aft_mapped_to_init"], result["revisit_consistency"], result["final_laser_cloud_map"])
    result["judgement"] = {"verdict": verdict, "issues": issues}

    os.makedirs(os.path.dirname(args.out_prefix), exist_ok=True)
    with open(args.out_prefix + ".json", "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2, sort_keys=True)
    write_report(args.out_prefix + ".md", result)
    print(json.dumps(result["judgement"], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
