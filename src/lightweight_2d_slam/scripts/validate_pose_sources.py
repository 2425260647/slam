#!/usr/bin/env python3

import argparse
import csv
import json
import math
import threading
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_tuple(pose):
    return (pose.position.x, pose.position.y,
            yaw_from_quaternion(pose.orientation))


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


def source_error(estimate, ground_truth):
    return {
        "translation_m": math.hypot(
            estimate[0] - ground_truth[0], estimate[1] - ground_truth[1]
        ),
        "yaw_rad": abs(normalize_angle(estimate[2] - ground_truth[2])),
    }


class PoseSourceValidator:
    def __init__(self, arguments):
        self.arguments = arguments
        self.lock = threading.Lock()
        self.latest = {"gazebo": None, "odom": None, "slam": None}
        self.rows = []

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
                arguments.odom_topic, Odometry, self.odom_callback, queue_size=50
            ),
            rospy.Subscriber(
                arguments.slam_pose_topic,
                PoseStamped,
                self.slam_pose_callback,
                queue_size=20,
            ),
        ]

    def model_states_callback(self, message):
        try:
            index = message.name.index(self.arguments.model_name)
        except ValueError:
            return
        self.update("gazebo", pose_tuple(message.pose[index]))

    def odom_callback(self, message):
        self.update("odom", pose_tuple(message.pose.pose))

    def slam_pose_callback(self, message):
        self.update("slam", pose_tuple(message.pose))

    def update(self, source, pose):
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
            rospy.sleep(0.05)
        missing = [name for name, value in self.latest.items() if value is None]
        raise RuntimeError("Timed out waiting for: " + ", ".join(missing))

    def stop(self):
        stop_command = Twist()
        for _ in range(5):
            self.command_publisher.publish(stop_command)
            rospy.sleep(0.05)

    def run_phase(self, name, duration, linear_velocity=0.0,
                  angular_velocity=0.0):
        command = Twist()
        command.linear.x = linear_velocity
        command.angular.z = angular_velocity
        deadline = time.monotonic() + duration
        rate = rospy.Rate(self.arguments.sample_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.command_publisher.publish(command)
            snapshot = self.snapshot()
            if snapshot is not None:
                self.rows.append(
                    [
                        name,
                        time.monotonic(),
                        rospy.Time.now().to_sec(),
                        *snapshot["gazebo"],
                        *snapshot["odom"],
                        *snapshot["slam"],
                    ]
                )
            rate.sleep()
        self.stop()
        return self.snapshot()

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
            writer.writerows(self.rows)

    @staticmethod
    def summarize_segment(start, end):
        relative = {
            source: relative_pose(start[source], end[source])
            for source in ("gazebo", "odom", "slam")
        }
        return {
            "relative_pose": {
                source: {
                    "x_m": pose[0], "y_m": pose[1], "yaw_rad": pose[2]
                }
                for source, pose in relative.items()
            },
            "error_vs_gazebo": {
                source: source_error(relative[source], relative["gazebo"])
                for source in ("odom", "slam")
            },
        }

    def execute(self):
        self.wait_for_sources()
        baseline = self.run_phase("baseline", self.arguments.settle_duration)
        straight_end = self.run_phase(
            "straight",
            self.arguments.straight_duration,
            linear_velocity=self.arguments.linear_velocity,
        )
        straight_settled = self.run_phase(
            "straight_settle", self.arguments.settle_duration
        )
        rotation_end = self.run_phase(
            "rotation",
            self.arguments.rotation_duration,
            angular_velocity=self.arguments.angular_velocity,
        )
        final = self.run_phase("final_settle", self.arguments.settle_duration)
        self.write_csv()

        summary = {
            "csv": self.arguments.output,
            "straight_motion": self.summarize_segment(
                baseline, straight_settled or straight_end
            ),
            "rotation_motion": self.summarize_segment(straight_settled, final),
            "total_motion": self.summarize_segment(baseline, final or rotation_end),
            "samples": len(self.rows),
        }
        print(json.dumps(summary, indent=2, sort_keys=True))


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Compare Gazebo, wheel odometry and SLAM pose increments."
    )
    parser.add_argument("--model-name", default="scout_mini")
    parser.add_argument("--model-states-topic", default="/gazebo/model_states")
    parser.add_argument(
        "--odom-topic", default="/scout_mini_velocity_controller/odom"
    )
    parser.add_argument("--slam-pose-topic", default="/tracked_pose")
    parser.add_argument(
        "--command-topic",
        default="/scout_mini_velocity_controller/cmd_vel",
    )
    parser.add_argument(
        "--output", default="/tmp/lightweight_pose_validation.csv"
    )
    parser.add_argument("--wait-timeout", type=float, default=20.0)
    parser.add_argument("--sample-rate", type=float, default=20.0)
    parser.add_argument("--settle-duration", type=float, default=1.0)
    parser.add_argument("--straight-duration", type=float, default=4.0)
    parser.add_argument("--rotation-duration", type=float, default=4.0)
    parser.add_argument("--linear-velocity", type=float, default=0.30)
    parser.add_argument("--angular-velocity", type=float, default=0.40)
    return parser.parse_args(rospy.myargv()[1:])


def main():
    rospy.init_node("lightweight_pose_source_validator", anonymous=True)
    validator = PoseSourceValidator(parse_arguments())
    try:
        validator.execute()
    finally:
        validator.stop()


if __name__ == "__main__":
    main()
