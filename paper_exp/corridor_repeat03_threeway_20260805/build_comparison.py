#!/usr/bin/env python3
import json
import math
import os

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from PIL import Image


ROOT = '/home/slam/slam_ws/paper_exp/corridor_repeat03_threeway_20260805'
ALGORITHMS = ('lightweight', 'cartographer', 'gmapping')


def yaw(quaternion):
    return 2.0 * math.atan2(quaternion.z, quaternion.w)


def caller_matches(expected):
    def predicate(topic, datatype, md5sum, message_definition, header):
        del topic, datatype, md5sum, message_definition
        caller = header.get('callerid', b'')
        if isinstance(caller, bytes):
            caller = caller.decode('utf-8')
        return caller == expected
    return predicate


def read_trajectory(algorithm):
    path = os.path.join(ROOT, 'full', algorithm, 'outputs.bag')
    kwargs = {}
    if algorithm == 'cartographer':
        kwargs['connection_filter'] = caller_matches('/cartographer_clean')
    poses = []
    with rosbag.Bag(path) as bag:
        for _, message, _ in bag.read_messages(
                topics=['/tracked_pose'], **kwargs):
            poses.append((message.pose.position.x, message.pose.position.y,
                          yaw(message.pose.orientation)))
    poses = np.asarray(poses, dtype=float)
    origin = poses[0].copy()
    cosine = math.cos(origin[2])
    sine = math.sin(origin[2])
    delta = poses[:, :2] - origin[:2]
    aligned = np.empty_like(delta)
    aligned[:, 0] = cosine * delta[:, 0] + sine * delta[:, 1]
    aligned[:, 1] = -sine * delta[:, 0] + cosine * delta[:, 1]
    return aligned


def crop_map(algorithm):
    path = os.path.join(ROOT, 'full', algorithm, 'map.pgm')
    image = np.asarray(Image.open(path), dtype=np.uint8)
    known = image != 205
    rows, columns = np.where(known)
    pad = 20
    y0 = max(0, int(rows.min()) - pad)
    y1 = min(image.shape[0], int(rows.max()) + pad + 1)
    x0 = max(0, int(columns.min()) - pad)
    x1 = min(image.shape[1], int(columns.max()) + pad + 1)
    return image[y0:y1, x0:x1]


def main():
    summaries = {}
    for algorithm in ALGORITHMS:
        path = os.path.join(ROOT, 'full', algorithm, 'summary.json')
        with open(path, encoding='utf-8') as stream:
            summaries[algorithm] = json.load(stream)

    figure, axes = plt.subplots(1, 3, figsize=(15, 5), constrained_layout=True)
    for axis, algorithm in zip(axes, ALGORITHMS):
        closure = summaries[algorithm]['trajectory_proxy'][
            'start_end_translation_m']
        axis.imshow(crop_map(algorithm), cmap='gray', vmin=0, vmax=255)
        axis.set_title('{}\nclosure proxy {:.3f} m'.format(
            algorithm, closure))
        axis.axis('off')
    map_path = os.path.join(ROOT, 'threeway_maps.png')
    figure.savefig(map_path, dpi=180, bbox_inches='tight')
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(9, 6), constrained_layout=True)
    for algorithm in ALGORITHMS:
        trajectory = read_trajectory(algorithm)
        stride = max(1, len(trajectory) // 6000)
        axis.plot(trajectory[::stride, 0], trajectory[::stride, 1],
                  linewidth=1.2, label=algorithm)
        axis.scatter(trajectory[0, 0], trajectory[0, 1], s=28)
        axis.scatter(trajectory[-1, 0], trajectory[-1, 1], s=28, marker='x')
    axis.set_aspect('equal', adjustable='datalim')
    axis.set_xlabel('start-aligned x (m)')
    axis.set_ylabel('start-aligned y (m)')
    axis.grid(True, alpha=0.25)
    axis.legend()
    trajectory_path = os.path.join(ROOT, 'threeway_trajectories.png')
    figure.savefig(trajectory_path, dpi=180, bbox_inches='tight')
    plt.close(figure)

    comparison = {
        algorithm: {
            'runtime': summaries[algorithm]['runtime'],
            'trajectory_proxy': summaries[algorithm]['trajectory_proxy'],
            'odometry_proxy': summaries[algorithm]['odometry_proxy'],
            'diagnostics': summaries[algorithm]['diagnostics'],
            'map': summaries[algorithm]['map'],
        } for algorithm in ALGORITHMS
    }
    comparison['metric_boundary'] = (
        'No external truth is present. Closure, trajectory and map values are '
        'engineering proxies and must not be reported as ATE or RPE.')
    with open(os.path.join(ROOT, 'threeway_summary.json'), 'w',
              encoding='utf-8') as stream:
        json.dump(comparison, stream, ensure_ascii=False, indent=2,
                  sort_keys=True)
        stream.write('\n')


if __name__ == '__main__':
    main()
