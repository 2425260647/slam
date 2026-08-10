#!/usr/bin/env python3

import argparse
import bisect
import csv
import json
import math
import statistics
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def pose_tuple(pose):
    return pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)


def relative_pose(origin, pose):
    dx = pose[0] - origin[0]
    dy = pose[1] - origin[1]
    cosine = math.cos(origin[2])
    sine = math.sin(origin[2])
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        normalize_angle(pose[2] - origin[2]),
    )


def percentile(values, fraction):
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1,
                       int(math.ceil(fraction * len(ordered))) - 1))
    return ordered[index]


class ReplayEvaluator:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.truth = []
        self.estimates = []
        self.processing_ms = []
        self.final_diagnostics = {}
        self.diagnostic_numeric = {
            'coarse_score': [],
            'correspondences': [],
            'residual_rms_m': [],
            'scan_quality': [],
        }
        self.tracking_status_counts = {}
        self.first_tracking_diagnostics = []
        self.last_message_wall = None
        self.truth_subscriber = rospy.Subscriber(
            args.truth_topic, ModelStates, self.truth_callback, queue_size=200
        )
        self.pose_subscriber = rospy.Subscriber(
            args.pose_topic, PoseStamped, self.pose_callback, queue_size=200
        )
        self.diagnostics_subscriber = None
        if args.diagnostics_topic:
            self.diagnostics_subscriber = rospy.Subscriber(
                args.diagnostics_topic, DiagnosticArray,
                self.diagnostics_callback, queue_size=100
            )

    def truth_callback(self, message):
        try:
            index = message.name.index(self.args.model_name)
        except ValueError:
            return
        sample = (rospy.Time.now().to_sec(), pose_tuple(message.pose[index]))
        with self.lock:
            self.truth.append(sample)
            self.last_message_wall = time.monotonic()

    def pose_callback(self, message):
        stamp = message.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = rospy.Time.now().to_sec()
        with self.lock:
            self.estimates.append((stamp, pose_tuple(message.pose)))
            self.last_message_wall = time.monotonic()

    def diagnostics_callback(self, message):
        if not message.status:
            return
        values = {item.key: item.value for item in message.status[0].values}
        with self.lock:
            self.final_diagnostics = values
            tracking_status = message.status[0].message
            self.tracking_status_counts[tracking_status] = (
                self.tracking_status_counts.get(tracking_status, 0) + 1
            )
            numeric_sample = {}
            for key in self.diagnostic_numeric:
                try:
                    numeric_value = float(values[key])
                except (KeyError, ValueError):
                    continue
                self.diagnostic_numeric[key].append(numeric_value)
                numeric_sample[key] = numeric_value
            if len(self.first_tracking_diagnostics) < 20:
                self.first_tracking_diagnostics.append({
                    'stamp': message.header.stamp.to_sec(),
                    'status': tracking_status,
                    **numeric_sample,
                })
            try:
                self.processing_ms.append(float(values['processing_ms']))
            except (KeyError, ValueError):
                pass

    def wait_until_complete(self):
        deadline = time.monotonic() + self.args.wait_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                received = bool(self.truth and self.estimates)
                last_message = self.last_message_wall
            if (received and last_message is not None and
                    time.monotonic() - last_message >= self.args.idle_timeout):
                return
            time.sleep(0.05)
        raise RuntimeError('timed out waiting for replay completion')

    def associate(self):
        with self.lock:
            truth = sorted(self.truth)
            estimates = sorted(self.estimates)
        truth_times = [sample[0] for sample in truth]
        rows = []
        for stamp, estimate in estimates:
            index = bisect.bisect_left(truth_times, stamp)
            candidates = []
            if index < len(truth):
                candidates.append(truth[index])
            if index > 0:
                candidates.append(truth[index - 1])
            if not candidates:
                continue
            truth_sample = min(candidates, key=lambda item: abs(item[0] - stamp))
            delta = abs(truth_sample[0] - stamp)
            if delta <= self.args.max_time_delta:
                rows.append((stamp, delta, truth_sample[1], estimate))
        return truth, estimates, rows

    def summarize(self):
        truth, estimates, rows = self.associate()
        if len(rows) < self.args.min_samples:
            raise RuntimeError(
                'only {} associated samples, need {}'.format(
                    len(rows), self.args.min_samples
                )
            )
        truth_origin = rows[0][2]
        estimate_origin = rows[0][3]
        output_rows = []
        translation_errors = []
        yaw_errors = []
        time_deltas = []
        for stamp, delta, truth_pose, estimate_pose in rows:
            truth_relative = relative_pose(truth_origin, truth_pose)
            estimate_relative = relative_pose(estimate_origin, estimate_pose)
            translation_error = math.hypot(
                estimate_relative[0] - truth_relative[0],
                estimate_relative[1] - truth_relative[1],
            )
            yaw_error = abs(normalize_angle(
                estimate_relative[2] - truth_relative[2]
            ))
            translation_errors.append(translation_error)
            yaw_errors.append(yaw_error)
            time_deltas.append(delta)
            output_rows.append((
                stamp, delta, *truth_relative, *estimate_relative,
                translation_error, yaw_error,
            ))

        def metric(values):
            return {
                'rmse': math.sqrt(statistics.mean(v * v for v in values)),
                'mean': statistics.mean(values),
                'p95': percentile(values, 0.95),
                'maximum': max(values),
                'final': values[-1],
            }

        with open(self.args.output_csv, 'w', newline='') as output_file:
            writer = csv.writer(output_file)
            writer.writerow([
                'ros_time', 'association_delta_s',
                'truth_x', 'truth_y', 'truth_yaw',
                'estimate_x', 'estimate_y', 'estimate_yaw',
                'translation_error_m', 'yaw_error_rad',
            ])
            writer.writerows(output_rows)

        summary = {
            'algorithm': self.args.algorithm,
            'truth_messages': len(truth),
            'estimate_messages': len(estimates),
            'associated_samples': len(rows),
            'max_association_delta_s': max(time_deltas),
            'translation_error_m': metric(translation_errors),
            'yaw_error_rad': metric(yaw_errors),
            'evaluation_mode': 'online_pose_start_aligned_se2',
            'truth_used_for_slam': False,
            'csv': self.args.output_csv,
        }
        with self.lock:
            processing_ms = list(self.processing_ms)
            final_diagnostics = dict(self.final_diagnostics)
            diagnostic_numeric = {
                key: list(values)
                for key, values in self.diagnostic_numeric.items()
            }
            tracking_status_counts = dict(self.tracking_status_counts)
            first_tracking_diagnostics = list(self.first_tracking_diagnostics)
        if processing_ms:
            summary['processing_ms'] = {
                'samples': len(processing_ms),
                'mean': statistics.mean(processing_ms),
                'p95': percentile(processing_ms, 0.95),
                'maximum': max(processing_ms),
            }
            summary['final_diagnostics'] = final_diagnostics
        summary['tracking_status_counts'] = tracking_status_counts
        summary['first_tracking_diagnostics'] = first_tracking_diagnostics
        summary['diagnostic_metrics'] = {
            key: metric(values)
            for key, values in diagnostic_numeric.items()
            if values
        }
        with open(self.args.output_json, 'w') as output_file:
            json.dump(summary, output_file, indent=2, sort_keys=True)
            output_file.write('\n')
        print(json.dumps(summary, indent=2, sort_keys=True))


def parse_args():
    parser = argparse.ArgumentParser(
        description='Evaluate online map-frame poses against replayed Gazebo truth.'
    )
    parser.add_argument('--algorithm', required=True)
    parser.add_argument('--truth-topic', default='/gazebo/model_states')
    parser.add_argument('--pose-topic', default='/tracked_pose')
    parser.add_argument('--diagnostics-topic', default='')
    parser.add_argument('--model-name', default='scout_mini')
    parser.add_argument('--output-csv', required=True)
    parser.add_argument('--output-json', required=True)
    parser.add_argument('--wait-timeout', type=float, default=300.0)
    parser.add_argument('--idle-timeout', type=float, default=3.0)
    parser.add_argument('--max-time-delta', type=float, default=0.08)
    parser.add_argument('--min-samples', type=int, default=100)
    return parser.parse_args(rospy.myargv()[1:])


if __name__ == '__main__':
    rospy.init_node('evaluate_replay_trajectory', anonymous=True)
    evaluator = ReplayEvaluator(parse_args())
    evaluator.wait_until_complete()
    evaluator.summarize()
