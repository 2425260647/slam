#!/usr/bin/env python3
"""Evaluate Cartographer and M3DGR Mocap trajectories with standard SE(2) metrics.

The estimator and reference are associated by timestamp, then the estimator is
aligned by one planar rigid transform. APE is reported at matched timestamps;
RPE uses the standard relative-pose error
inv(T_ref_i_j) * T_est_i_j, with translation and yaw reported separately.
"""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def _first_present(row, names):
    for name in names:
        if name in row and row[name] not in ("", None):
            return row[name]
    raise KeyError(f"None of the columns are present: {names}")


def _optional_present(row, names):
    for name in names:
        if name in row and row[name] not in ("", None):
            return row[name]
    return None


def wrap_angle(angle):
    """Return an angle in [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        raise ValueError("zero-norm quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def read_pose_csv(path):
    timestamps = []
    positions = []
    yaws = []
    with Path(path).open("r", encoding="utf-8", newline="") as source:
        reader = csv.DictReader(source)
        for row in reader:
            try:
                raw_time = _first_present(row, ("%time", "time", "field.header.stamp"))
                timestamp = float(raw_time)
                if timestamp > 1.0e12:
                    timestamp *= 1.0e-9
                elif timestamp > 1.0e9:
                    timestamp *= 1.0e-9
                position = [
                    float(_first_present(row, ("field.pose.position.x", "pose.position.x"))),
                    float(_first_present(row, ("field.pose.position.y", "pose.position.y"))),
                    float(_first_present(row, ("field.pose.position.z", "pose.position.z"))),
                ]
                quaternion = [
                    _optional_present(row, ("field.pose.orientation.x", "pose.orientation.x")),
                    _optional_present(row, ("field.pose.orientation.y", "pose.orientation.y")),
                    _optional_present(row, ("field.pose.orientation.z", "pose.orientation.z")),
                    _optional_present(row, ("field.pose.orientation.w", "pose.orientation.w")),
                ]
                if any(value is None for value in quaternion):
                    raise ValueError("missing orientation columns")
                yaw = quaternion_to_yaw(*(float(value) for value in quaternion))
            except (KeyError, TypeError, ValueError):
                continue
            if not timestamps or timestamp >= timestamps[-1]:
                timestamps.append(timestamp)
                positions.append(position)
                yaws.append(yaw)
    if len(timestamps) < 2:
        raise RuntimeError(f"Not enough valid SE(2) poses in {path}")
    return (
        np.asarray(timestamps, dtype=np.float64),
        np.asarray(positions, dtype=np.float64),
        np.asarray(yaws, dtype=np.float64),
    )


def nearest_association(estimate_times, estimate_positions, estimate_yaws,
                        reference_times, reference_positions, reference_yaws,
                        max_delta):
    indices = np.searchsorted(reference_times, estimate_times, side="left")
    matched_estimate = []
    matched_reference = []
    matched_estimate_yaws = []
    matched_reference_yaws = []
    matched_deltas = []
    matched_times = []
    for estimate_index, right_index in enumerate(indices):
        candidates = []
        if right_index < len(reference_times):
            candidates.append(right_index)
        if right_index > 0:
            candidates.append(right_index - 1)
        if not candidates:
            continue
        reference_index = min(
            candidates,
            key=lambda index: abs(reference_times[index] - estimate_times[estimate_index]),
        )
        delta = abs(reference_times[reference_index] - estimate_times[estimate_index])
        if delta <= max_delta:
            matched_estimate.append(estimate_positions[estimate_index])
            matched_reference.append(reference_positions[reference_index])
            matched_estimate_yaws.append(estimate_yaws[estimate_index])
            matched_reference_yaws.append(reference_yaws[reference_index])
            matched_deltas.append(delta)
            matched_times.append(estimate_times[estimate_index])
    if len(matched_estimate) < 3:
        raise RuntimeError(
            f"Only {len(matched_estimate)} associated poses; "
            "check simulated time and topic capture."
        )
    return (
        np.asarray(matched_estimate, dtype=np.float64),
        np.asarray(matched_reference, dtype=np.float64),
        np.asarray(matched_estimate_yaws, dtype=np.float64),
        np.asarray(matched_reference_yaws, dtype=np.float64),
        np.asarray(matched_deltas, dtype=np.float64),
        np.asarray(matched_times, dtype=np.float64),
    )


def rigid_align_se2(source_positions, source_yaws, target_positions):
    """Align source to target with a planar rigid transform."""
    source_xy = source_positions[:, :2]
    target_xy = target_positions[:, :2]
    source_center = source_xy.mean(axis=0)
    target_center = target_xy.mean(axis=0)
    covariance = (source_xy - source_center).T @ (target_xy - target_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    aligned_xy = source_xy @ rotation.T + translation
    alignment_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    aligned_yaws = np.asarray(
        [wrap_angle(yaw + alignment_yaw) for yaw in source_yaws],
        dtype=np.float64,
    )
    aligned_positions = source_positions.copy()
    aligned_positions[:, :2] = aligned_xy
    return aligned_positions, aligned_yaws, rotation, translation, alignment_yaw


def _relative_pose(position_i, yaw_i, position_j, yaw_j):
    rotation_i = np.array(
        [[math.cos(yaw_i), -math.sin(yaw_i)],
         [math.sin(yaw_i), math.cos(yaw_i)]],
        dtype=np.float64,
    )
    delta_translation = rotation_i.T @ (position_j[:2] - position_i[:2])
    return delta_translation, wrap_angle(yaw_j - yaw_i)


def _compose_pose(translation_a, yaw_a, translation_b, yaw_b):
    rotation_a = np.array(
        [[math.cos(yaw_a), -math.sin(yaw_a)],
         [math.sin(yaw_a), math.cos(yaw_a)]],
        dtype=np.float64,
    )
    return translation_a + rotation_a @ translation_b, wrap_angle(yaw_a + yaw_b)


def _inverse_pose(translation, yaw):
    rotation = np.array(
        [[math.cos(yaw), -math.sin(yaw)],
         [math.sin(yaw), math.cos(yaw)]],
        dtype=np.float64,
    )
    return -(rotation.T @ translation), wrap_angle(-yaw)


def _stats(values):
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        return {}
    return {
        "rmse": float(np.sqrt(np.mean(np.square(values)))),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "std": float(np.std(values)),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
    }


def relative_error_metrics(times, estimate_positions, estimate_yaws,
                           reference_positions, reference_yaws,
                           delta_seconds, tolerance):
    translation_errors = []
    yaw_errors = []
    for index, timestamp in enumerate(times):
        target_time = timestamp + delta_seconds
        right_index = int(np.searchsorted(times, target_time, side="left"))
        candidates = []
        if right_index < len(times):
            candidates.append(right_index)
        if right_index > 0:
            candidates.append(right_index - 1)
        if not candidates:
            continue
        target_index = min(candidates, key=lambda item: abs(times[item] - target_time))
        if target_index <= index or abs(times[target_index] - target_time) > tolerance:
            continue

        estimate_delta, estimate_yaw = _relative_pose(
            estimate_positions[index], estimate_yaws[index],
            estimate_positions[target_index], estimate_yaws[target_index])
        reference_delta, reference_yaw = _relative_pose(
            reference_positions[index], reference_yaws[index],
            reference_positions[target_index], reference_yaws[target_index])
        inverse_reference_delta, inverse_reference_yaw = _inverse_pose(
            reference_delta, reference_yaw)
        error_delta, error_yaw = _compose_pose(
            inverse_reference_delta, inverse_reference_yaw,
            estimate_delta, estimate_yaw)
        translation_errors.append(float(np.linalg.norm(error_delta)))
        yaw_errors.append(abs(error_yaw))

    return {
        "pair_count": int(len(translation_errors)),
        "se2_translation_meters": _stats(translation_errors),
        "se2_rotation_radians": _stats(yaw_errors),
        "se2_rotation_degrees": _stats(np.degrees(yaw_errors)),
    }


def world_displacement_error_metrics(times, estimate_positions,
                                     reference_positions, delta_seconds,
                                     tolerance):
    errors = []
    for index, timestamp in enumerate(times):
        target_time = timestamp + delta_seconds
        right_index = int(np.searchsorted(times, target_time, side="left"))
        candidates = []
        if right_index < len(times):
            candidates.append(right_index)
        if right_index > 0:
            candidates.append(right_index - 1)
        if not candidates:
            continue
        target_index = min(candidates, key=lambda item: abs(times[item] - target_time))
        if target_index <= index or abs(times[target_index] - target_time) > tolerance:
            continue
        estimate_delta = (estimate_positions[target_index, :2] -
                          estimate_positions[index, :2])
        reference_delta = (reference_positions[target_index, :2] -
                           reference_positions[index, :2])
        errors.append(float(np.linalg.norm(estimate_delta - reference_delta)))
    return int(len(errors)), _stats(errors)


def evaluate(estimate_path, reference_path, max_delta, rpe_delta, rpe_tolerance,
             reference_body_yaw_offset=None):
    estimate_times, estimate_positions, estimate_yaws = read_pose_csv(estimate_path)
    reference_times, reference_positions, reference_yaws = read_pose_csv(reference_path)
    (estimate, reference, estimate_yaws, reference_yaws, deltas,
     associated_times) = nearest_association(
         estimate_times, estimate_positions, estimate_yaws,
         reference_times, reference_positions, reference_yaws, max_delta)

    aligned, aligned_yaws, rotation, translation, alignment_yaw = rigid_align_se2(
        estimate, estimate_yaws, reference)
    position_errors = np.linalg.norm(aligned[:, :2] - reference[:, :2], axis=1)
    calibrated_reference_yaws = np.asarray(
        [wrap_angle(yaw + reference_body_yaw_offset)
         for yaw in reference_yaws], dtype=np.float64,
    ) if reference_body_yaw_offset is not None else reference_yaws
    yaw_errors = None
    if reference_body_yaw_offset is not None:
        yaw_errors = np.asarray(
            [abs(wrap_angle(a - b))
             for a, b in zip(aligned_yaws, calibrated_reference_yaws)],
            dtype=np.float64,
        )
    relative_pose_errors = relative_error_metrics(
        associated_times, aligned, aligned_yaws, reference,
        calibrated_reference_yaws, rpe_delta, rpe_tolerance)
    world_pair_count, world_translation_errors = world_displacement_error_metrics(
        associated_times, aligned, reference, rpe_delta, rpe_tolerance)
    full_se2_valid = reference_body_yaw_offset is not None
    return {
        "evaluation_protocol": {
            "pose_group": "SE(2)",
            "alignment": "single planar rigid transform (xy + yaw), no scale",
            "rpe": "inv(T_ref_i_j) * T_est_i_j",
            "full_se2_valid": full_se2_valid,
            "full_se2_blocker": None if full_se2_valid else
                "missing calibrated Mocap-rigid-body to base_footprint yaw offset",
        },
        "association": {
            "reference_pose_count": int(len(reference_times)),
            "estimate_pose_count": int(len(estimate_times)),
            "matched_pose_count": int(len(estimate)),
            "max_time_difference_seconds": float(np.max(deltas)),
            "mean_absolute_time_difference_seconds": float(np.mean(deltas)),
        },
        "alignment": {
            "type": "rigid_SE2_without_scale",
            "rotation_matrix_2d": rotation.tolist(),
            "yaw_offset_radians": float(alignment_yaw),
            "translation_xy": translation.tolist(),
            "scale": 1.0,
            "reference_body_yaw_offset_radians": reference_body_yaw_offset,
        },
        # Keep the historical key names for existing reports, but their values
        # are now explicitly planar SE(2) translation errors.
        "ape_translation_meters": _stats(position_errors),
        "ape_planar_translation_meters": _stats(position_errors),
        "ape_rotation_radians": _stats(yaw_errors) if full_se2_valid else {},
        "ape_rotation_degrees": (
            _stats(np.degrees(yaw_errors)) if full_se2_valid else {}),
        "rpe": {
            "delta_seconds": rpe_delta,
            "pair_tolerance_seconds": rpe_tolerance,
            "pair_count": relative_pose_errors["pair_count"],
            "se2_translation_meters": (
                relative_pose_errors["se2_translation_meters"]
                if full_se2_valid else {}),
            # Relative yaw cancels a constant rigid-body yaw offset in 2D and
            # remains valid even before the static body calibration is known.
            "se2_rotation_radians": relative_pose_errors["se2_rotation_radians"],
            "se2_rotation_degrees": relative_pose_errors["se2_rotation_degrees"],
            "frame_invariant_world_displacement_pair_count": world_pair_count,
            "frame_invariant_world_displacement_meters": world_translation_errors,
        },
        "limitations": [
            "M3DGR does not publish an explicit Mocap-rigid-body to base_footprint lever-arm calibration in the dataset bag.",
            "Full SE(2) yaw APE and body-frame translation RPE are withheld unless --reference-body-yaw-offset-radians is supplied from a defensible calibration.",
            "SE(2) APE/RPE are valid only for runs with continuous Mocap pose; Corridor02 is therefore excluded from these metrics.",
            "The single planar alignment removes global xy/yaw gauge error but does not remove drift.",
        ],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--estimate", required=True)
    parser.add_argument("--reference", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--max-time-delta", type=float, default=0.02)
    parser.add_argument("--rpe-delta", type=float, default=1.0)
    parser.add_argument("--rpe-tolerance", type=float, default=0.05)
    parser.add_argument(
        "--reference-body-yaw-offset-radians", type=float, default=None,
        help=("Static yaw from the Mocap rigid-body frame to base_footprint. "
              "Without this calibration, full SE(2) yaw APE and body-frame "
              "translation RPE are intentionally withheld."))
    args = parser.parse_args()
    metrics = evaluate(
        args.estimate,
        args.reference,
        args.max_time_delta,
        args.rpe_delta,
        args.rpe_tolerance,
        args.reference_body_yaw_offset_radians,
    )
    with Path(args.output).open("w", encoding="utf-8") as output:
        json.dump(metrics, output, ensure_ascii=False, indent=2)
    print(json.dumps(metrics, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
