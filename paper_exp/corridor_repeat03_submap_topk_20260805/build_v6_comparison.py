#!/usr/bin/env python3
"""Build fixed-scope v4/v5/v6/Cartographer corridor audit artifacts."""

import json
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from PIL import Image


ROOT = "/home/slam/slam_ws"
EXPERIMENT = os.path.join(
    ROOT, "paper_exp/corridor_repeat03_submap_topk_20260805")
CASES = {
    "v4": os.path.join(EXPERIMENT, "full/lightweight_v4_observable"),
    "v5": os.path.join(EXPERIMENT, "full/lightweight_v5_scan_graph"),
    "v6": os.path.join(EXPERIMENT, "full/lightweight_v6_anisotropic"),
    "Cartographer": os.path.join(
        ROOT, "paper_exp/corridor_repeat03_threeway_20260805/full/cartographer"),
}


def yaw(quaternion):
    return 2.0 * math.atan2(quaternion.z, quaternion.w)


def cartographer_connection(topic, datatype, md5sum, definition, header):
    del topic, datatype, md5sum, definition
    caller = header.get("callerid", b"")
    if isinstance(caller, bytes):
        caller = caller.decode("utf-8")
    return caller == "/cartographer_clean"


def read_trajectory(label, run_dir):
    kwargs = {}
    if label == "Cartographer":
        kwargs["connection_filter"] = cartographer_connection
    poses = []
    with rosbag.Bag(os.path.join(run_dir, "outputs.bag")) as bag:
        for _, message, _ in bag.read_messages(
                topics=["/tracked_pose"], **kwargs):
            poses.append((message.pose.position.x, message.pose.position.y,
                          yaw(message.pose.orientation)))
    poses = np.asarray(poses, dtype=float)
    origin = poses[0]
    cosine = math.cos(origin[2])
    sine = math.sin(origin[2])
    delta = poses[:, :2] - origin[:2]
    aligned = np.empty_like(delta)
    aligned[:, 0] = cosine * delta[:, 0] + sine * delta[:, 1]
    aligned[:, 1] = -sine * delta[:, 0] + cosine * delta[:, 1]
    return aligned


def crop_map(run_dir):
    image = np.asarray(Image.open(os.path.join(run_dir, "map.pgm")),
                       dtype=np.uint8)
    rows, columns = np.where(image != 205)
    pad = 20
    y0 = max(0, int(rows.min()) - pad)
    y1 = min(image.shape[0], int(rows.max()) + pad + 1)
    x0 = max(0, int(columns.min()) - pad)
    x1 = min(image.shape[1], int(columns.max()) + pad + 1)
    return image[y0:y1, x0:x1]


def load_summary(run_dir):
    with open(os.path.join(run_dir, "summary.json"), encoding="utf-8") as stream:
        return json.load(stream)


def main():
    summaries = {label: load_summary(path) for label, path in CASES.items()}
    with open(os.path.join(EXPERIMENT, "v6_map_audit.json"),
              encoding="utf-8") as stream:
        map_audit = json.load(stream)

    figure, axes = plt.subplots(1, 4, figsize=(20, 5), constrained_layout=True)
    for axis, (label, run_dir) in zip(axes, CASES.items()):
        closure = summaries[label]["trajectory_proxy"][
            "start_end_translation_m"]
        axis.imshow(crop_map(run_dir), cmap="gray", vmin=0, vmax=255)
        axis.set_title("{}\nclosure proxy {:.3f} m".format(label, closure))
        axis.axis("off")
    figure.savefig(os.path.join(EXPERIMENT, "v6_maps.png"), dpi=180,
                   bbox_inches="tight")
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(10, 7), constrained_layout=True)
    for label, run_dir in CASES.items():
        trajectory = read_trajectory(label, run_dir)
        stride = max(1, len(trajectory) // 6000)
        axis.plot(trajectory[::stride, 0], trajectory[::stride, 1],
                  linewidth=1.15, label=label)
        axis.scatter(trajectory[0, 0], trajectory[0, 1], s=28)
        axis.scatter(trajectory[-1, 0], trajectory[-1, 1], s=28, marker="x")
    axis.set_aspect("equal", adjustable="datalim")
    axis.set_xlabel("start-aligned x (m)")
    axis.set_ylabel("start-aligned y (m)")
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.savefig(os.path.join(EXPERIMENT, "v6_trajectories.png"), dpi=180,
                   bbox_inches="tight")
    plt.close(figure)

    audit_keys = {
        "v4": "lightweight_v4_observable",
        "v5": "lightweight_v5_scan_graph",
        "v6": "lightweight_v6_anisotropic",
        "Cartographer": "cartographer",
    }
    result = {}
    for label, summary in summaries.items():
        final = summary.get("diagnostics", {}).get("final_values", {})
        result[label] = {
            "closure_proxy_m": summary["trajectory_proxy"][
                "start_end_translation_m"],
            "closure_yaw_proxy_rad": summary["trajectory_proxy"][
                "start_end_yaw_rad"],
            "maximum_pose_step_m": summary["trajectory_proxy"][
                "maximum_pose_step_m"],
            "loop_constraints": final.get("loop_constraints"),
            "anisotropic_constraints": final.get(
                "loop_anisotropic_constraints"),
            "map_audit": map_audit[audit_keys[label]],
        }
    result["metric_boundary"] = (
        "No continuous ground truth is present. Closure and map structure "
        "values are engineering proxies, not ATE or RPE.")
    with open(os.path.join(EXPERIMENT, "v6_comparison.json"), "w",
              encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")


if __name__ == "__main__":
    main()
