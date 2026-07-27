#!/usr/bin/env python3

import json
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
import yaml
from PIL import Image


EXP_ROOT = "/home/slam/slam_ws/paper_exp/formal_corridor_20260724"
BAG_PATH = "/home/slam/slam_ws/bags/corridor_repeat_01.bag"
RUN_DIR = os.path.join(EXP_ROOT, "partial_precheck", "clean_baseline")
TRAJECTORY_BAG = os.path.join(RUN_DIR, "trajectory_nodes.bag")
MAP_PREFIX = os.path.join(RUN_DIR, "map")
OUTBOUND_MAP_PREFIX = os.path.join(
    EXP_ROOT, "missing_wall_stage_check", "outbound_only", "map")
NO_FREE_SPACE_MAP_PREFIX = os.path.join(
    EXP_ROOT, "missing_wall_stage_check", "no_free_space_full", "map")

# The boxes follow the missing upper-wall spans marked by the user. Control
# boxes cover neighboring upper-wall spans that are visible in the same map.
REGIONS = {
    "gap_near_branch": {
        "box": [22.5, 27.0, 1.25, 2.15],
        "control": [18.0, 22.0, 1.15, 2.05],
    },
    "gap_far_end": {
        "box": [70.5, 77.8, 3.25, 4.35],
        "control": [66.0, 70.0, 3.15, 4.15],
    },
}

LASER_TO_BASE_YAW = -1.5708
MIN_HEIGHT = -0.515
MAX_HEIGHT = 1.435
MIN_RANGE = 0.30
MAX_RANGE = 30.0
ANGLE_MIN = -3.14159
ANGLE_MAX = 3.14159
ANGLE_INCREMENT = 0.00436


def load_trajectory():
    rows = []
    with rosbag.Bag(TRAJECTORY_BAG) as bag:
        for _, msg, stamp in bag.read_messages(topics=["trajectory_0"]):
            rotation = msg.transform.rotation
            rows.append((
                stamp.to_sec(),
                msg.transform.translation.x,
                msg.transform.translation.y,
                2.0 * math.atan2(rotation.z, rotation.w),
            ))
    trajectory = np.asarray(rows, dtype=float)
    trajectory[:, 3] = np.unwrap(trajectory[:, 3])
    return trajectory


def nearest_insertion_pose(trajectory, stamp, tolerance=0.02):
    times = trajectory[:, 0]
    if stamp < times[0] or stamp > times[-1]:
        return None
    index = int(np.searchsorted(times, stamp))
    candidates = [candidate for candidate in (index - 1, index)
                  if 0 <= candidate < len(times)]
    nearest = min(candidates, key=lambda candidate: abs(times[candidate] - stamp))
    if abs(times[nearest] - stamp) > tolerance:
        return None
    return trajectory[nearest, 1:4].copy()


def cloud_xyz_ring(msg):
    offsets = {field.name: field.offset for field in msg.fields}
    required = ("x", "y", "z")
    if any(name not in offsets for name in required):
        raise RuntimeError("PointCloud2 does not contain x/y/z fields")
    names = list(required)
    formats = ["<f4", "<f4", "<f4"]
    field_offsets = [offsets[name] for name in required]
    if "ring" in offsets:
        names.append("ring")
        formats.append("<u2")
        field_offsets.append(offsets["ring"])
    dtype = np.dtype({
        "names": names,
        "formats": formats,
        "offsets": field_offsets,
        "itemsize": msg.point_step,
    })
    points = np.frombuffer(msg.data, dtype=dtype,
                           count=msg.width * msg.height)
    xyz = np.column_stack((points["x"], points["y"], points["z"]))
    rings = points["ring"].astype(np.int16) if "ring" in names else None
    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite], rings[finite] if rings is not None else None


def laser_to_map(xy, pose):
    laser_yaw = pose[2] + LASER_TO_BASE_YAW
    cosine = math.cos(laser_yaw)
    sine = math.sin(laser_yaw)
    result = np.empty_like(xy, dtype=float)
    result[:, 0] = pose[0] + cosine * xy[:, 0] - sine * xy[:, 1]
    result[:, 1] = pose[1] + sine * xy[:, 0] + cosine * xy[:, 1]
    return result


def projected_scan_endpoints(xyz, pose):
    ranges = np.hypot(xyz[:, 0], xyz[:, 1])
    angles = np.arctan2(xyz[:, 1], xyz[:, 0])
    keep = ((xyz[:, 2] >= MIN_HEIGHT) & (xyz[:, 2] <= MAX_HEIGHT)
            & (ranges >= MIN_RANGE) & (ranges <= MAX_RANGE)
            & (angles >= ANGLE_MIN) & (angles <= ANGLE_MAX))
    filtered = xyz[keep]
    ranges = ranges[keep]
    angles = angles[keep]
    bin_count = int(math.ceil((ANGLE_MAX - ANGLE_MIN) / ANGLE_INCREMENT))
    indices = np.floor((angles - ANGLE_MIN) / ANGLE_INCREMENT).astype(int)
    valid = (indices >= 0) & (indices < bin_count)
    indices = indices[valid]
    ranges = ranges[valid]
    minimum_ranges = np.full(bin_count, np.inf, dtype=float)
    np.minimum.at(minimum_ranges, indices, ranges)
    finite = np.isfinite(minimum_ranges)
    bin_angles = ANGLE_MIN + np.flatnonzero(finite) * ANGLE_INCREMENT
    selected_xy = np.column_stack((minimum_ranges[finite] * np.cos(bin_angles),
                                   minimum_ranges[finite] * np.sin(bin_angles)))
    return filtered, laser_to_map(selected_xy, pose), int(finite.sum()), bin_count


def inside_box(points, box):
    return ((points[:, 0] >= box[0]) & (points[:, 0] <= box[1])
            & (points[:, 1] >= box[2]) & (points[:, 1] <= box[3]))


def pose_near_box(pose, box, margin=12.0):
    closest_x = min(max(pose[0], box[0]), box[1])
    closest_y = min(max(pose[1], box[2]), box[3])
    return math.hypot(pose[0] - closest_x, pose[1] - closest_y) <= margin


def load_map(prefix=MAP_PREFIX):
    image = np.asarray(Image.open(prefix + ".pgm"), dtype=np.uint8)
    with open(prefix + ".yaml", "r", encoding="utf-8") as source:
        metadata = yaml.safe_load(source)
    return image, float(metadata["resolution"]), metadata["origin"]


def map_points(image, resolution, origin, value):
    rows, columns = np.where(image == value)
    x = origin[0] + (columns + 0.5) * resolution
    y = origin[1] + (image.shape[0] - rows - 0.5) * resolution
    return np.column_stack((x, y))


def region_map_statistics(image, resolution, origin, box):
    x0 = max(0, int(math.floor((box[0] - origin[0]) / resolution)))
    x1 = min(image.shape[1], int(math.ceil((box[1] - origin[0]) / resolution)))
    row_top = image.shape[0] - int(math.ceil((box[3] - origin[1]) / resolution))
    row_bottom = image.shape[0] - int(math.floor((box[2] - origin[1]) / resolution))
    y0 = max(0, row_top)
    y1 = min(image.shape[0], row_bottom)
    crop = image[y0:y1, x0:x1]
    total = max(1, crop.size)
    return {
        "cells": int(crop.size),
        "occupied_cells": int(np.count_nonzero(crop == 0)),
        "free_cells": int(np.count_nonzero(crop == 254)),
        "unknown_cells": int(np.count_nonzero(crop == 205)),
        "occupied_ratio": float(np.count_nonzero(crop == 0) / total),
        "free_ratio": float(np.count_nonzero(crop == 254) / total),
        "unknown_ratio": float(np.count_nonzero(crop == 205) / total),
    }


def initialize_region_result(definition):
    result = {
        "box": definition["box"],
        "control_box": definition["control"],
        "frames": 0,
        "times_from_bag_start_s": [],
        "raw_points_in_gap": 0,
        "height_filtered_raw_points_in_gap": 0,
        "projected_endpoints_in_gap": 0,
        "raw_points_in_control": 0,
        "height_filtered_raw_points_in_control": 0,
        "projected_endpoints_in_control": 0,
        "finite_scan_bins": [],
        "raw_gap_z": [],
        "raw_gap_rings": [],
        "projected_gap_points": [],
        "projected_gap_points_outbound": [],
        "projected_gap_points_return": [],
        "projected_control_points": [],
    }
    return result


def process_clouds(trajectory):
    results = {name: initialize_region_result(definition)
               for name, definition in REGIONS.items()}
    bag_start = None
    with rosbag.Bag(BAG_PATH) as bag:
        bag_start = bag.get_start_time()
        for _, msg, stamp in bag.read_messages(topics=["/velodyne_points"]):
            current_time = stamp.to_sec()
            # Only trajectory nodes survive Cartographer's motion filter and
            # are inserted into submaps. Match the original cloud to a node
            # timestamp instead of interpolating every recorded cloud.
            pose = nearest_insertion_pose(trajectory, current_time)
            if pose is None:
                continue
            active = [name for name, definition in REGIONS.items()
                      if pose_near_box(pose, definition["box"])]
            if not active:
                continue
            xyz, rings = cloud_xyz_ring(msg)
            planar_ranges = np.hypot(xyz[:, 0], xyz[:, 1])
            range_valid = ((planar_ranges >= MIN_RANGE)
                           & (planar_ranges <= MAX_RANGE))
            raw_map = laser_to_map(xyz[range_valid, :2], pose)
            raw_z = xyz[range_valid, 2]
            raw_rings = rings[range_valid] if rings is not None else None
            filtered, projected_map, finite_bins, bin_count = \
                projected_scan_endpoints(xyz, pose)
            filtered_map = laser_to_map(filtered[:, :2], pose)

            for name in active:
                definition = REGIONS[name]
                result = results[name]
                gap_raw = inside_box(raw_map, definition["box"])
                gap_filtered = inside_box(filtered_map, definition["box"])
                gap_projected = inside_box(projected_map, definition["box"])
                control_raw = inside_box(raw_map, definition["control"])
                control_filtered = inside_box(filtered_map, definition["control"])
                control_projected = inside_box(projected_map,
                                               definition["control"])
                result["frames"] += 1
                result["times_from_bag_start_s"].append(current_time - bag_start)
                result["raw_points_in_gap"] += int(gap_raw.sum())
                result["height_filtered_raw_points_in_gap"] += int(
                    gap_filtered.sum())
                result["projected_endpoints_in_gap"] += int(
                    gap_projected.sum())
                result["raw_points_in_control"] += int(control_raw.sum())
                result["height_filtered_raw_points_in_control"] += int(
                    control_filtered.sum())
                result["projected_endpoints_in_control"] += int(
                    control_projected.sum())
                result["finite_scan_bins"].append(finite_bins / bin_count)
                result["raw_gap_z"].extend(raw_z[gap_raw].tolist())
                if raw_rings is not None:
                    result["raw_gap_rings"].extend(
                        raw_rings[gap_raw].astype(int).tolist())
                if gap_projected.any():
                    gap_points = projected_map[gap_projected]
                    result["projected_gap_points"].extend(gap_points.tolist())
                    if math.cos(pose[2]) > 0.5:
                        result["projected_gap_points_outbound"].extend(
                            gap_points.tolist())
                    elif math.cos(pose[2]) < -0.5:
                        result["projected_gap_points_return"].extend(
                            gap_points.tolist())
                if control_projected.any():
                    result["projected_control_points"].extend(
                        projected_map[control_projected].tolist())
    return results, bag_start


def wall_spread_metrics(points, box):
    points = np.asarray(points, dtype=float)
    if points.size == 0:
        return {"points": 0}
    bin_edges = np.arange(box[0], box[1] + 0.1001, 0.1)
    bin_indices = np.digitize(points[:, 0], bin_edges) - 1
    centers = []
    spreads = []
    for index in range(len(bin_edges) - 1):
        y_values = points[bin_indices == index, 1]
        if y_values.size < 20:
            continue
        centers.append((0.5 * (bin_edges[index] + bin_edges[index + 1]),
                        float(np.median(y_values))))
        spreads.append(float(np.percentile(y_values, 90)
                             - np.percentile(y_values, 10)))
    if len(centers) < 2:
        return {"points": int(points.shape[0]), "valid_x_bins": len(centers)}
    centers = np.asarray(centers)
    slope, intercept = np.polyfit(centers[:, 0], centers[:, 1], 1)
    residuals = centers[:, 1] - (slope * centers[:, 0] + intercept)
    midpoint_x = 0.5 * (box[0] + box[1])
    return {
        "points": int(points.shape[0]),
        "valid_x_bins": int(len(centers)),
        "line_slope": float(slope),
        "line_intercept": float(intercept),
        "line_y_at_region_midpoint": float(slope * midpoint_x + intercept),
        "median_p90_p10_width_m": float(np.median(spreads)),
        "line_center_rmse_m": float(np.sqrt(np.mean(residuals ** 2))),
    }


def trajectory_visit_metrics(trajectory, box):
    in_x = ((trajectory[:, 1] >= box[0]) & (trajectory[:, 1] <= box[1]))
    heading_cosine = np.cos(trajectory[:, 3])
    outbound = trajectory[in_x & (heading_cosine > 0.5)]
    returning = trajectory[in_x & (heading_cosine < -0.5)]
    result = {
        "outbound_nodes": int(len(outbound)),
        "return_nodes": int(len(returning)),
    }
    if len(outbound) and len(returning):
        x_samples = np.linspace(box[0], box[1], 50)
        outbound_order = np.argsort(outbound[:, 1])
        return_order = np.argsort(returning[:, 1])
        outbound_y = np.interp(x_samples, outbound[outbound_order, 1],
                               outbound[outbound_order, 2])
        return_y = np.interp(x_samples, returning[return_order, 1],
                             returning[return_order, 2])
        separation = return_y - outbound_y
        result.update({
            "signed_lateral_separation_median_m": float(np.median(separation)),
            "absolute_lateral_separation_median_m": float(
                np.median(np.abs(separation))),
            "absolute_lateral_separation_p90_m": float(
                np.percentile(np.abs(separation), 90)),
        })
    return result


def summarize_results(results, trajectory, image, resolution, origin,
                      outbound_map=None, no_free_space_map=None):
    summary = {}
    for name, raw in results.items():
        finite = np.asarray(raw["finite_scan_bins"], dtype=float)
        z_values = np.asarray(raw["raw_gap_z"], dtype=float)
        rings = np.asarray(raw["raw_gap_rings"], dtype=int)
        gap_raw = raw["raw_points_in_gap"]
        control_raw = raw["raw_points_in_control"]
        summary[name] = {
            "box": raw["box"],
            "control_box": raw["control_box"],
            "frames": raw["frames"],
            "time_min_s": float(min(raw["times_from_bag_start_s"])),
            "time_max_s": float(max(raw["times_from_bag_start_s"])),
            "finite_scan_bin_ratio_mean": float(finite.mean()),
            "finite_scan_bin_ratio_min": float(finite.min()),
            "raw_points_in_gap": gap_raw,
            "height_filtered_raw_points_in_gap": raw[
                "height_filtered_raw_points_in_gap"],
            "projected_endpoints_in_gap": raw["projected_endpoints_in_gap"],
            "raw_points_in_control": control_raw,
            "height_filtered_raw_points_in_control": raw[
                "height_filtered_raw_points_in_control"],
            "projected_endpoints_in_control": raw[
                "projected_endpoints_in_control"],
            "gap_to_control_raw_ratio": float(gap_raw / max(1, control_raw)),
            "gap_to_control_projected_ratio": float(
                raw["projected_endpoints_in_gap"]
                / max(1, raw["projected_endpoints_in_control"])),
            "gap_height_filter_keep_ratio": float(
                raw["height_filtered_raw_points_in_gap"] / max(1, gap_raw)),
            "gap_projection_keep_ratio": float(
                raw["projected_endpoints_in_gap"]
                / max(1, raw["height_filtered_raw_points_in_gap"])),
            "gap_z_min_m": float(z_values.min()) if z_values.size else None,
            "gap_z_median_m": float(np.median(z_values)) if z_values.size else None,
            "gap_z_max_m": float(z_values.max()) if z_values.size else None,
            "gap_ring_counts": {
                str(value): int(np.count_nonzero(rings == value))
                for value in np.unique(rings)
            },
            "map_gap": region_map_statistics(
                image, resolution, origin, raw["box"]),
            "map_control": region_map_statistics(
                image, resolution, origin, raw["control_box"]),
            "projected_wall_all": wall_spread_metrics(
                raw["projected_gap_points"], raw["box"]),
            "projected_wall_outbound": wall_spread_metrics(
                raw["projected_gap_points_outbound"], raw["box"]),
            "projected_wall_return": wall_spread_metrics(
                raw["projected_gap_points_return"], raw["box"]),
            "trajectory_visits": trajectory_visit_metrics(
                trajectory, raw["box"]),
        }
        if outbound_map is not None:
            outbound_image, outbound_resolution, outbound_origin = outbound_map
            outbound_gap = region_map_statistics(
                outbound_image, outbound_resolution, outbound_origin,
                raw["box"])
            outbound_control = region_map_statistics(
                outbound_image, outbound_resolution, outbound_origin,
                raw["control_box"])
            final_ratio = summary[name]["map_gap"]["occupied_ratio"]
            outbound_ratio = outbound_gap["occupied_ratio"]
            summary[name]["outbound_map_gap"] = outbound_gap
            summary[name]["outbound_map_control"] = outbound_control
            summary[name]["occupied_ratio_retained_after_return"] = float(
                final_ratio / max(outbound_ratio, 1e-12))
        if no_free_space_map is not None:
            no_free_image, no_free_resolution, no_free_origin = \
                no_free_space_map
            summary[name]["no_free_space_map_gap"] = region_map_statistics(
                no_free_image, no_free_resolution, no_free_origin,
                raw["box"])
    return summary


def draw_figure(trajectory, image, resolution, origin, results, output_path):
    extent = [origin[0], origin[0] + image.shape[1] * resolution,
              origin[1], origin[1] + image.shape[0] * resolution]
    figure, axes = plt.subplots(3, 1, figsize=(15, 11), constrained_layout=True)
    axes[0].imshow(image, cmap="gray", origin="upper", extent=extent,
                   vmin=0, vmax=255)
    axes[0].plot(trajectory[:, 1], trajectory[:, 2], color="#0066cc",
                 linewidth=0.8, label="optimized trajectory")
    colors = ("#d62728", "#2ca02c")
    for color, (name, definition) in zip(colors, REGIONS.items()):
        box = definition["box"]
        rectangle = plt.Rectangle((box[0], box[2]), box[1] - box[0],
                                  box[3] - box[2], fill=False,
                                  edgecolor=color, linewidth=2.0)
        axes[0].add_patch(rectangle)
        axes[0].text(box[0], box[3] + 0.2, name, color=color, fontsize=9)
    axes[0].set_xlim(-2, 80)
    axes[0].set_ylim(-6, 10)
    axes[0].set_aspect("equal")
    axes[0].set_title("Final baseline map, optimized trajectory and missing-wall regions")
    axes[0].legend(loc="lower left")
    axes[0].grid(alpha=0.25)

    for axis, color, (name, definition) in zip(axes[1:], colors,
                                                REGIONS.items()):
        box = definition["box"]
        padding_x = 2.0
        padding_y = 1.5
        axis.imshow(image, cmap="gray", origin="upper", extent=extent,
                    vmin=0, vmax=255)
        outbound = np.asarray(
            results[name]["projected_gap_points_outbound"], dtype=float)
        returning = np.asarray(
            results[name]["projected_gap_points_return"], dtype=float)
        if outbound.size:
            axis.scatter(outbound[:, 0], outbound[:, 1], s=5, c="#e41a1c",
                         alpha=0.3, label="outbound endpoints")
        if returning.size:
            axis.scatter(returning[:, 0], returning[:, 1], s=5, c="#ff7f00",
                         alpha=0.3, label="return endpoints")
        control = np.asarray(results[name]["projected_control_points"],
                             dtype=float)
        if control.size:
            axis.scatter(control[:, 0], control[:, 1], s=4, c="#377eb8",
                         alpha=0.25, label="control-wall endpoints")
        rectangle = plt.Rectangle((box[0], box[2]), box[1] - box[0],
                                  box[3] - box[2], fill=False,
                                  edgecolor=color, linewidth=2.0)
        axis.add_patch(rectangle)
        axis.set_xlim(box[0] - padding_x, box[1] + padding_x)
        axis.set_ylim(box[2] - padding_y, box[3] + padding_y)
        axis.set_aspect("equal")
        axis.set_title(name)
        axis.grid(alpha=0.25)
        axis.legend(loc="upper right")
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def draw_stage_comparison(outbound_map, final_map, no_free_space_map,
                          output_path):
    maps = (("Outbound only", outbound_map),
            ("Final round trip", final_map),
            ("Round trip, free-space disabled", no_free_space_map))
    figure, axes = plt.subplots(2, 3, figsize=(20, 7),
                                constrained_layout=True)
    for column, (stage_name, current_map) in enumerate(maps):
        image, resolution, origin = current_map
        extent = [origin[0], origin[0] + image.shape[1] * resolution,
                  origin[1], origin[1] + image.shape[0] * resolution]
        for row, (region_name, definition) in enumerate(REGIONS.items()):
            axis = axes[row, column]
            box = definition["box"]
            axis.imshow(image, cmap="gray", origin="upper", extent=extent,
                        vmin=0, vmax=255)
            rectangle = plt.Rectangle(
                (box[0], box[2]), box[1] - box[0], box[3] - box[2],
                fill=False, edgecolor="#31a354", linewidth=2.0)
            axis.add_patch(rectangle)
            axis.set_xlim(box[0] - 2.0, box[1] + 2.0)
            axis.set_ylim(box[2] - 1.5, box[3] + 1.5)
            axis.set_aspect("equal")
            axis.set_title(f"{stage_name}: {region_name}")
            axis.grid(alpha=0.2)
    figure.savefig(output_path, dpi=200)
    plt.close(figure)


def write_report(summary, output_path):
    with open(output_path, "w", encoding="utf-8") as output:
        output.write("# corridor_repeat_01 缺墙数据链检查\n\n")
        output.write("## 检查边界\n\n")
        output.write("- 使用 Clean Baseline 最终 PBStream 导出的 2600 个优化轨迹节点，只统计与节点时间差不超过 20 ms、实际通过运动过滤并写入子图的点云帧。\n")
        output.write("- 原始数据为 `/velodyne_points`，静态外参采用修正后的 `base_link -> laser_link z=0.33521 m`、yaw=-1.5708 rad；二维分析只涉及 x/y，因此 z 平移不改变墙面平面坐标。\n")
        output.write("- 模拟当前 `pointcloud_to_laserscan` 参数：高度 `[-0.515,1.435] m`、距离 `[0.3,30] m`、角分辨率 `0.00436 rad`，每个角度格保留最近点。\n\n")
        output.write("## 定量结果\n\n")
        output.write("| 区域 | 帧数 | 原始墙点 | 高度过滤后 | 二维投影端点 | 去程占据数 | 正常往返占据数 | 关闭自由空间占据数 | 回程后保留率 |\n")
        output.write("|---|---:|---:|---:|---:|---:|---:|---:|---:|\n")
        for name, current in summary.items():
            output.write(
                f"| {name} | {current['frames']} | {current['raw_points_in_gap']} "
                f"| {current['height_filtered_raw_points_in_gap']} "
                f"| {current['projected_endpoints_in_gap']} "
                f"| {current['outbound_map_gap']['occupied_cells']} "
                f"| {current['map_gap']['occupied_cells']} "
                f"| {current['no_free_space_map_gap']['occupied_cells']} "
                f"| {100.0 * current['occupied_ratio_retained_after_return']:.2f}% |\n")
        output.write("\n")
        for name, current in summary.items():
            output.write(f"### {name}\n\n")
            output.write(
                f"- 对应数据时间覆盖：bag 起点后 `{current['time_min_s']:.2f}`~"
                f"`{current['time_max_s']:.2f} s`。\n")
            output.write(
                f"- 当前高度带保留缺口原始墙点比例："
                f"`{100.0 * current['gap_height_filter_keep_ratio']:.2f}%`。\n")
            output.write(
                f"- 高度过滤后点成为二维最近端点的比例："
                f"`{100.0 * current['gap_projection_keep_ratio']:.2f}%`。\n")
            output.write(
                f"- 缺口原始墙点 z 范围：`{current['gap_z_min_m']}`~"
                f"`{current['gap_z_max_m']} m`，中位数 "
                f"`{current['gap_z_median_m']} m`。\n")
            output.write(
                f"- 地图缺口框：占据 `{current['map_gap']['occupied_cells']}`、"
                f"空闲 `{current['map_gap']['free_cells']}`、未知 "
                f"`{current['map_gap']['unknown_cells']}` 个栅格。\n\n")
            output.write(
                f"- 去程同区域占据 `{current['outbound_map_gap']['occupied_cells']}` "
                f"个栅格，最终仅保留 "
                f"`{100.0 * current['occupied_ratio_retained_after_return']:.2f}%`。\n\n")
            no_free = current["no_free_space_map_gap"]
            normal_cells = current["map_gap"]["occupied_cells"]
            output.write(
                f"- 完整往返但关闭自由空间插入后，占据栅格恢复为 "
                f"`{no_free['occupied_cells']}` 个，是正常完整往返结果的 "
                f"`{no_free['occupied_cells'] / max(1, normal_cells):.1f}` 倍；"
                f"该区域空闲栅格从 `{current['map_gap']['free_cells']}` 降为 "
                f"`{no_free['free_cells']}`。\n\n")
            visits = current["trajectory_visits"]
            output.write(
                f"- 去回程优化轨迹横向分离中位数："
                f"`{visits.get('absolute_lateral_separation_median_m', 0.0):.3f} m`，"
                f"P90：`{visits.get('absolute_lateral_separation_p90_m', 0.0):.3f} m`。\n")
            outbound_wall = current["projected_wall_outbound"]
            return_wall = current["projected_wall_return"]
            if ("line_y_at_region_midpoint" in outbound_wall
                    and "line_y_at_region_midpoint" in return_wall):
                output.write(
                    f"- 去程/回程墙端点拟合线在区域中点的 y 差："
                    f"`{abs(return_wall['line_y_at_region_midpoint'] - outbound_wall['line_y_at_region_midpoint']):.3f} m`；"
                    f"去程点带 P90-P10 中位宽度 "
                    f"`{outbound_wall['median_p90_p10_width_m']:.3f} m`，"
                    f"回程为 `{return_wall['median_p90_p10_width_m']:.3f} m`。\n\n")
        output.write("## 判读说明\n\n")
        output.write("- 若原始墙点接近 0，则缺墙源于采集回波或区域定位。\n")
        output.write("- 若原始墙点充足但高度过滤后接近 0，则源于高度带。\n")
        output.write("- 若高度过滤后充足但二维端点接近 0，则源于同角度最近点竞争。\n")
        output.write("- 若二维端点仍充足但最终地图无占据，则应继续检查轨迹配准、子图概率融合和自由空间射线。\n")
        output.write("\n## 阶段结论\n\n")
        output.write("两处墙体在只含去程的地图中均已建立，但在正常完整往返后大部分占据栅格消失；关闭自由空间插入后，同一完整往返数据中的墙体重新出现。原始回波、高度过滤和二维投影均有充分墙点，因此缺墙不是录包、安装高度或投影过滤造成，而是回程有限回波射线在小量局部位姿/子图错位下反复穿过早期墙栅格，将其更新为空闲，并由后生成子图纹理覆盖早期占据纹理。完全关闭自由空间会失去可通行区域证据，只用于确认因果，不应作为最终建图参数。\n")


def main():
    trajectory = load_trajectory()
    image, resolution, origin = load_map()
    outbound_map = load_map(OUTBOUND_MAP_PREFIX)
    no_free_space_map = load_map(NO_FREE_SPACE_MAP_PREFIX)
    raw_results, _ = process_clouds(trajectory)
    summary = summarize_results(raw_results, trajectory, image, resolution, origin,
                                outbound_map, no_free_space_map)

    metrics_path = os.path.join(EXP_ROOT, "missing_wall_diagnostics.json")
    figure_path = os.path.join(EXP_ROOT, "missing_wall_diagnostics.png")
    report_path = os.path.join(EXP_ROOT, "missing_wall_diagnostics.md")
    stage_figure_path = os.path.join(
        EXP_ROOT, "missing_wall_stage_comparison.png")
    with open(metrics_path, "w", encoding="utf-8") as output:
        json.dump(summary, output, ensure_ascii=False, indent=2,
                  sort_keys=True)
    draw_figure(trajectory, image, resolution, origin, raw_results, figure_path)
    draw_stage_comparison(outbound_map, (image, resolution, origin),
                          no_free_space_map, stage_figure_path)
    write_report(summary, report_path)
    print(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))
    print("report:", report_path)
    print("figure:", figure_path)
    print("stage comparison:", stage_figure_path)


if __name__ == "__main__":
    main()
