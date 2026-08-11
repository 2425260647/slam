#!/usr/bin/env python3
import argparse
import json
import math
import os
import re

import numpy as np
import rosbag
import yaml
from PIL import Image


def yaw_from_quaternion(quaternion):
    return 2.0 * math.atan2(quaternion.z, quaternion.w)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def message_counts(path):
    result = {}
    with rosbag.Bag(path) as bag:
        for topic, count in bag.get_type_and_topic_info().topics.items():
            result[topic] = int(count.message_count)
    return result


def caller_matches(expected):
    def predicate(topic, datatype, md5sum, message_definition, header):
        del topic, datatype, md5sum, message_definition
        caller = header.get('callerid', b'')
        if isinstance(caller, bytes):
            caller = caller.decode('utf-8')
        return caller == expected
    return predicate


def trajectory_metrics(path, pose_caller_id=None):
    poses = []
    with rosbag.Bag(path) as bag:
        kwargs = {}
        if pose_caller_id:
            kwargs['connection_filter'] = caller_matches(pose_caller_id)
        for _, message, stamp in bag.read_messages(
                topics=['/tracked_pose'], **kwargs):
            poses.append((stamp.to_sec(), message.pose.position.x,
                          message.pose.position.y,
                          yaw_from_quaternion(message.pose.orientation)))
    result = {
        'pose_messages': len(poses),
        'pose_caller_id': pose_caller_id,
    }
    if not poses:
        return result
    path_length = 0.0
    maximum_step = 0.0
    for previous, current in zip(poses, poses[1:]):
        step = math.hypot(current[1] - previous[1],
                          current[2] - previous[2])
        path_length += step
        maximum_step = max(maximum_step, step)
    first = poses[0]
    last = poses[-1]
    result.update({
        'pose_duration_sec': last[0] - first[0],
        'path_length_m': path_length,
        'maximum_pose_step_m': maximum_step,
        'start_pose': list(first[1:]),
        'end_pose': list(last[1:]),
        'start_end_translation_m': math.hypot(last[1] - first[1],
                                               last[2] - first[2]),
        'start_end_yaw_rad': abs(normalize_angle(last[3] - first[3])),
    })
    return result


def odometry_metrics(path):
    poses = []
    with rosbag.Bag(path) as bag:
        for _, message, stamp in bag.read_messages(topics=['/odom']):
            poses.append((stamp.to_sec(), message.pose.pose.position.x,
                          message.pose.pose.position.y,
                          yaw_from_quaternion(message.pose.pose.orientation)))
    result = {'messages': len(poses)}
    if not poses:
        return result
    path_length = 0.0
    for previous, current in zip(poses, poses[1:]):
        path_length += math.hypot(current[1] - previous[1],
                                  current[2] - previous[2])
    first = poses[0]
    last = poses[-1]
    result.update({
        'duration_sec': last[0] - first[0],
        'path_length_m': path_length,
        'start_end_translation_m': math.hypot(last[1] - first[1],
                                               last[2] - first[2]),
        'start_end_yaw_rad': abs(normalize_angle(last[3] - first[3])),
    })
    return result


def diagnostic_metrics(path):
    status_counts = {}
    numeric_values = {}
    final_values = {}
    count = 0
    with rosbag.Bag(path) as bag:
        for _, message, _ in bag.read_messages(topics=['/slam_diagnostics']):
            if not message.status:
                continue
            count += 1
            status = message.status[0]
            status_counts[status.message] = status_counts.get(
                status.message, 0) + 1
            final_values = {entry.key: entry.value for entry in status.values}
            for key, value in final_values.items():
                try:
                    numeric_values.setdefault(key, []).append(float(value))
                except ValueError:
                    pass
    result = {
        'messages': count,
        'status_counts': status_counts,
        'final_values': final_values,
    }
    for key in ('processing_ms', 'coarse_score', 'residual_rms_m',
                'scan_quality', 'correspondences'):
        values = numeric_values.get(key, [])
        if values:
            result[key] = {
                'mean': float(np.mean(values)),
                'p95': float(np.percentile(values, 95)),
                'maximum': float(np.max(values)),
            }
    return result


def map_metrics(run_dir):
    image = np.asarray(Image.open(os.path.join(run_dir, 'map.pgm')),
                       dtype=np.uint8)
    with open(os.path.join(run_dir, 'map.yaml'), encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream)
    occupied = image == 0
    free = image == 254
    known = occupied | free
    return {
        'width_px': int(image.shape[1]),
        'height_px': int(image.shape[0]),
        'resolution_m': float(metadata['resolution']),
        'origin': metadata['origin'],
        'occupied_cells': int(occupied.sum()),
        'free_cells': int(free.sum()),
        'known_cells': int(known.sum()),
        'unknown_cells': int((~known).sum()),
        'occupied_ratio_known': float(occupied.sum() / max(1, known.sum())),
    }


def wallclock_metrics(path):
    with open(path, encoding='utf-8') as stream:
        text = stream.read()
    match = re.search(r'play_wall_duration_sec=([0-9.]+)', text)
    return {'play_wall_duration_sec': float(match.group(1)) if match else None}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--algorithm', required=True)
    parser.add_argument('--mode', required=True)
    parser.add_argument('--input-bag', required=True)
    parser.add_argument('--run-dir', required=True)
    args = parser.parse_args()

    pose_caller_id = ('/cartographer_clean'
                      if args.algorithm == 'cartographer' else None)
    summary = {
        'algorithm': args.algorithm,
        'mode': args.mode,
        'input_bag': os.path.abspath(args.input_bag),
        'input_counts': message_counts(args.input_bag),
        'output_counts': message_counts(
            os.path.join(args.run_dir, 'outputs.bag')),
        'trajectory_proxy': trajectory_metrics(
            os.path.join(args.run_dir, 'outputs.bag'), pose_caller_id),
        'odometry_proxy': odometry_metrics(args.input_bag),
        'diagnostics': diagnostic_metrics(
            os.path.join(args.run_dir, 'outputs.bag')),
        'map': map_metrics(args.run_dir),
        'runtime': wallclock_metrics(
            os.path.join(args.run_dir, 'wallclock.txt')),
        'truth_available': False,
        'truth_used_for_slam': False,
        'metric_boundary': (
            'Start/end closure and map statistics are no-ground-truth '
            'engineering proxies, not ATE or RPE.'),
    }
    output_path = os.path.join(args.run_dir, 'summary.json')
    with open(output_path, 'w', encoding='utf-8') as stream:
        json.dump(summary, stream, indent=2, ensure_ascii=False, sort_keys=True)
        stream.write('\n')
    print(json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True))


if __name__ == '__main__':
    main()
