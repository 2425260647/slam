#!/usr/bin/env python3

import argparse
import csv
import json
import math
import statistics
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z),
    )


def pose_tuple(pose):
    return (
        pose.position.x,
        pose.position.y,
        yaw_from_quaternion(pose.orientation),
    )


def relative_pose(start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    cosine = math.cos(start[2])
    sine = math.sin(start[2])
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        normalize_angle(end[2] - start[2]),
    )


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1,
                int(math.ceil(fraction * len(ordered))) - 1)
    return ordered[max(0, index)]


class SquareValidator:
    def __init__(self, arguments):
        self.arguments = arguments
        self.lock = threading.Lock()
        self.latest = {"gazebo": None, "odom": None, "slam": None}
        self.rows = []
        self.processing_ms = []
        self.last_diagnostics = {}
        self.last_slam_wall = None

        self.command_publisher = rospy.Publisher(
            arguments.command_topic, Twist, queue_size=1
        )
        self.subscribers = [
            rospy.Subscriber(
                arguments.model_states_topic,
                ModelStates,
                self.model_states_callback,
                queue_size=10,
            ),
            rospy.Subscriber(
                arguments.odom_topic,
                Odometry,
                self.odom_callback,
                queue_size=50,
            ),
            rospy.Subscriber(
                arguments.slam_pose_topic,
                PoseStamped,
                self.slam_pose_callback,
                queue_size=20,
            ),
            rospy.Subscriber(
                arguments.diagnostics_topic,
                DiagnosticArray,
                self.diagnostics_callback,
                queue_size=20,
            ),
        ]

    def model_states_callback(self, message):
        try:
            index = message.name.index(self.arguments.model_name)
        except ValueError:
            return
        self.update_pose("gazebo", pose_tuple(message.pose[index]))

    def odom_callback(self, message):
        self.update_pose("odom", pose_tuple(message.pose.pose))

    def slam_pose_callback(self, message):
        self.update_pose("slam", pose_tuple(message.pose))
        with self.lock:
            self.last_slam_wall = time.monotonic()

    def diagnostics_callback(self, message):
        if not message.status:
            return
        values = {
            item.key: item.value for item in message.status[0].values
        }
        with self.lock:
            self.last_diagnostics = values
            try:
                self.processing_ms.append(float(values["processing_ms"]))
            except (KeyError, ValueError):
                pass

    def update_pose(self, source, pose):
        with self.lock:
            self.latest[source] = pose

    def snapshot(self):
        with self.lock:
            if any(value is None for value in self.latest.values()):
                return None
            return dict(self.latest)

    def wait_for_sources(self):
        deadline = time.monotonic() + self.arguments.wait_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.snapshot() is not None:
                return
            time.sleep(0.05)
        missing = [
            name for name, value in self.latest.items() if value is None
        ]
        raise RuntimeError("Timed out waiting for: " + ", ".join(missing))

    def stop(self):
        command = Twist()
        for _ in range(5):
            self.command_publisher.publish(command)
            time.sleep(0.05)

    def run_phase(self, phase, duration, linear=0.0, angular=0.0):
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        deadline = time.monotonic() + duration
        rate = rospy.Rate(self.arguments.sample_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.command_publisher.publish(command)
            snapshot = self.snapshot()
            if snapshot is not None:
                self.rows.append(
                    (
                        phase,
                        time.monotonic(),
                        rospy.Time.now().to_sec(),
                        snapshot["gazebo"],
                        snapshot["odom"],
                        snapshot["slam"],
                    )
                )
            rate.sleep()
        self.stop()

    def execute(self):
        self.wait_for_sources()
        if self.arguments.replay:
            self.execute_replay()
            return
        self.run_phase("initial_settle", self.arguments.settle_duration)
        for side in range(1, 5):
            self.run_phase(
                "side_{}_straight".format(side),
                self.arguments.straight_duration,
                linear=self.arguments.linear_velocity,
            )
            self.run_phase(
                "side_{}_straight_settle".format(side),
                self.arguments.settle_duration,
            )
            self.run_phase(
                "side_{}_turn".format(side),
                self.arguments.rotation_duration,
                angular=self.arguments.angular_velocity,
            )
            self.run_phase(
                "side_{}_turn_settle".format(side),
                self.arguments.settle_duration,
            )
        self.run_phase("final_settle", self.arguments.settle_duration)
        self.finish()

    def execute_replay(self):
        period = 1.0 / self.arguments.sample_rate
        while not rospy.is_shutdown():
            with self.lock:
                last_slam_wall = self.last_slam_wall
            if (last_slam_wall is not None and
                    time.monotonic() - last_slam_wall >=
                    self.arguments.replay_idle_timeout):
                break
            snapshot = self.snapshot()
            if snapshot is not None:
                self.rows.append(
                    (
                        "replay",
                        time.monotonic(),
                        rospy.Time.now().to_sec(),
                        snapshot["gazebo"],
                        snapshot["odom"],
                        snapshot["slam"],
                    )
                )
            time.sleep(period)
        self.finish()

    def finish(self):
        self.write_csv()
        summary = self.build_summary()
        with open(self.arguments.summary, "w") as output_file:
            json.dump(summary, output_file, indent=2, sort_keys=True)
            output_file.write("\n")
        print(json.dumps(summary, indent=2, sort_keys=True))

    def write_csv(self):
        with open(self.arguments.output, "w", newline="") as output_file:
            writer = csv.writer(output_file)
            writer.writerow(
                [
                    "phase", "wall_time", "ros_time",
                    "gazebo_x", "gazebo_y", "gazebo_yaw",
                    "odom_x", "odom_y", "odom_yaw",
                    "slam_x", "slam_y", "slam_yaw",
                ]
            )
            for phase, wall_time, ros_time, gazebo, odom, slam in self.rows:
                writer.writerow(
                    [phase, wall_time, ros_time, *gazebo, *odom, *slam]
                )

    def build_summary(self):
        if not self.rows:
            raise RuntimeError("No synchronized pose samples were recorded")
        starts = {
            source: self.rows[0][index]
            for source, index in (("gazebo", 3), ("odom", 4), ("slam", 5))
        }
        translation_errors = {"odom": [], "slam": []}
        yaw_errors = {"odom": [], "slam": []}
        relative_final = {}
        for row in self.rows:
            relative = {
                source: relative_pose(starts[source], row[index])
                for source, index in (
                    ("gazebo", 3), ("odom", 4), ("slam", 5)
                )
            }
            for source in ("odom", "slam"):
                translation_errors[source].append(
                    math.hypot(
                        relative[source][0] - relative["gazebo"][0],
                        relative[source][1] - relative["gazebo"][1],
                    )
                )
                yaw_errors[source].append(
                    abs(normalize_angle(
                        relative[source][2] - relative["gazebo"][2]
                    ))
                )
            relative_final = relative

        def error_summary(values):
            return {
                "rmse": math.sqrt(statistics.mean(
                    value * value for value in values
                )),
                "maximum": max(values),
                "p95": percentile(values, 0.95),
            }

        with self.lock:
            diagnostics = dict(self.last_diagnostics)
            processing_ms = list(self.processing_ms)
        return {
            "samples": len(self.rows),
            "csv": self.arguments.output,
            "trajectory_error_vs_gazebo": {
                source: {
                    "translation_m": error_summary(
                        translation_errors[source]
                    ),
                    "yaw_rad": error_summary(yaw_errors[source]),
                }
                for source in ("odom", "slam")
            },
            "final_relative_pose": {
                source: {
                    "x_m": relative_final[source][0],
                    "y_m": relative_final[source][1],
                    "yaw_rad": relative_final[source][2],
                    "closure_distance_m": math.hypot(
                        relative_final[source][0],
                        relative_final[source][1],
                    ),
                }
                for source in ("gazebo", "odom", "slam")
            },
            "final_error_vs_gazebo": {
                source: {
                    "translation_m": translation_errors[source][-1],
                    "yaw_rad": yaw_errors[source][-1],
                }
                for source in ("odom", "slam")
            },
            "processing_ms": {
                "samples": len(processing_ms),
                "mean": statistics.mean(processing_ms)
                if processing_ms else 0.0,
                "p95": percentile(processing_ms, 0.95),
                "maximum": max(processing_ms) if processing_ms else 0.0,
            },
            "final_diagnostics": diagnostics,
        }


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Drive and evaluate a four-sided Gazebo SLAM loop."
    )
    parser.add_argument("--model-name", default="scout_mini")
    parser.add_argument("--model-states-topic", default="/gazebo/model_states")
    parser.add_argument(
        "--odom-topic", default="/scout_mini_velocity_controller/odom"
    )
    parser.add_argument("--slam-pose-topic", default="/tracked_pose")
    parser.add_argument("--diagnostics-topic", default="/slam_diagnostics")
    parser.add_argument(
        "--command-topic",
        default="/scout_mini_velocity_controller/cmd_vel",
    )
    parser.add_argument("--output", required=True)
    parser.add_argument("--summary", required=True)
    parser.add_argument("--wait-timeout", type=float, default=30.0)
    parser.add_argument("--sample-rate", type=float, default=20.0)
    parser.add_argument("--settle-duration", type=float, default=1.0)
    parser.add_argument("--straight-duration", type=float, default=4.0)
    parser.add_argument("--rotation-duration", type=float, default=4.0)
    parser.add_argument("--linear-velocity", type=float, default=0.15)
    parser.add_argument("--angular-velocity", type=float, default=0.45)
    parser.add_argument(
        "--replay",
        action="store_true",
        help="Record replayed pose topics until tracked poses become idle.",
    )
    parser.add_argument("--replay-idle-timeout", type=float, default=3.0)
    return parser.parse_args(rospy.myargv()[1:])


def main():
    rospy.init_node("lightweight_square_validator", anonymous=True)
    validator = SquareValidator(parse_arguments())
    try:
        validator.execute()
    finally:
        validator.stop()


if __name__ == "__main__":
    main()
