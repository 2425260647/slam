#!/usr/bin/env python3

import argparse
import csv
import json
import math
import threading
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def pose_tuple(pose):
    return pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)


class TruthGuidedController:
    """Drive a reproducible waypoint loop using Gazebo pose feedback only."""

    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.pose = None
        self.initial_pose = None
        self.rows = []
        self.publisher = rospy.Publisher(args.command_topic, Twist, queue_size=1)
        self.subscriber = rospy.Subscriber(
            args.model_states_topic, ModelStates, self.model_states_callback,
            queue_size=10,
        )
        self.waypoints = self.parse_waypoints(args.waypoints)

    def parse_waypoints(self, text):
        points = []
        for item in text.split(';'):
            values = [float(value.strip()) for value in item.split(',')]
            if len(values) != 2:
                raise ValueError("waypoint must be x,y: {}".format(item))
            points.append((values[0], values[1]))
        if len(points) < 2:
            raise ValueError("at least two waypoints are required")
        return points

    def model_states_callback(self, message):
        try:
            index = message.name.index(self.args.model_name)
        except ValueError:
            return
        with self.lock:
            self.pose = pose_tuple(message.pose[index])

    def get_pose(self):
        with self.lock:
            return self.pose

    def publish_stop(self):
        command = Twist()
        for _ in range(5):
            self.publisher.publish(command)
            time.sleep(0.05)

    def record_row(self, phase, target, distance, heading_error, command):
        pose = self.get_pose()
        if pose is None:
            return
        self.rows.append([
            phase, rospy.Time.now().to_sec(), pose[0], pose[1], pose[2],
            target[0], target[1], distance, heading_error,
            command.linear.x, command.angular.z,
        ])

    def wait_for_pose(self):
        deadline = time.monotonic() + self.args.wait_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.get_pose() is not None:
                return
            time.sleep(0.05)
        raise RuntimeError("timed out waiting for /gazebo/model_states")

    def unpause(self):
        if not self.args.unpause:
            return
        try:
            rospy.wait_for_service('/gazebo/unpause_physics', self.args.wait_timeout)
            rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
        except (rospy.ROSException, rospy.ServiceException) as error:
            raise RuntimeError("cannot unpause Gazebo: {}".format(error))

    def drive_to(self, target, index):
        phase = "waypoint_{:02d}".format(index)
        deadline = time.monotonic() + self.args.waypoint_timeout
        rate = rospy.Rate(self.args.control_rate)
        last_progress = time.monotonic()
        best_distance = float('inf')
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            pose = self.get_pose()
            if pose is None:
                rate.sleep()
                continue
            dx = target[0] - pose[0]
            dy = target[1] - pose[1]
            distance = math.hypot(dx, dy)
            desired_heading = math.atan2(dy, dx)
            heading_error = normalize_angle(desired_heading - pose[2])
            if distance <= self.args.position_tolerance:
                self.publish_stop()
                return distance

            command = Twist()
            command.angular.z = max(
                -self.args.max_angular,
                min(self.args.max_angular, self.args.angular_gain * heading_error),
            )
            if abs(heading_error) <= self.args.heading_gate:
                command.linear.x = min(
                    self.args.max_linear,
                    self.args.linear_gain * distance,
                )
                command.linear.x = max(self.args.min_linear, command.linear.x)
            self.publisher.publish(command)
            self.record_row(phase, target, distance, heading_error, command)
            if distance <= best_distance - self.args.progress_distance:
                best_distance = distance
                last_progress = time.monotonic()
            elif time.monotonic() - last_progress > self.args.progress_timeout:
                self.publish_stop()
                raise RuntimeError("no progress toward waypoint {}".format(index))
            rate.sleep()
        self.publish_stop()
        raise RuntimeError("timed out reaching waypoint {}".format(index))

    def align_yaw(self, target_yaw):
        deadline = time.monotonic() + self.args.waypoint_timeout
        rate = rospy.Rate(self.args.control_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            pose = self.get_pose()
            if pose is None:
                rate.sleep()
                continue
            error = normalize_angle(target_yaw - pose[2])
            if abs(error) <= self.args.yaw_tolerance:
                self.publish_stop()
                return abs(error)
            command = Twist()
            command.angular.z = max(
                -self.args.max_angular,
                min(self.args.max_angular, self.args.angular_gain * error),
            )
            self.publisher.publish(command)
            self.record_row('final_yaw_alignment', (pose[0], pose[1]), 0.0,
                            error, command)
            rate.sleep()
        self.publish_stop()
        raise RuntimeError("timed out aligning final yaw")

    def settle(self, phase):
        self.publish_stop()
        deadline = time.monotonic() + self.args.settle_duration
        rate = rospy.Rate(self.args.control_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            pose = self.get_pose()
            if pose is not None:
                self.rows.append([phase, rospy.Time.now().to_sec(), pose[0], pose[1], pose[2],
                                  '', '', '', '', 0.0, 0.0])
            rate.sleep()

    def write_outputs(self):
        with open(self.args.output_csv, 'w', newline='') as output_file:
            writer = csv.writer(output_file)
            writer.writerow([
                'phase', 'ros_time', 'gazebo_x', 'gazebo_y', 'gazebo_yaw',
                'target_x', 'target_y', 'distance_m', 'heading_error_rad',
                'command_linear_x', 'command_angular_z',
            ])
            writer.writerows(self.rows)
        final_pose = self.get_pose()
        closure = None
        if self.initial_pose is not None and final_pose is not None:
            closure = {
                'translation_m': math.hypot(
                    final_pose[0] - self.initial_pose[0],
                    final_pose[1] - self.initial_pose[1],
                ),
                'yaw_rad': abs(normalize_angle(
                    final_pose[2] - self.initial_pose[2]
                )),
            }
        summary = {
            'waypoints': self.waypoints,
            'rows': len(self.rows),
            'initial_pose': self.initial_pose,
            'final_pose': final_pose,
            'closure_error': closure,
            'position_tolerance_m': self.args.position_tolerance,
            'world': self.args.world_name,
            'model_states_topic': self.args.model_states_topic,
            'truth_used_for_slam': False,
        }
        with open(self.args.output_json, 'w') as output_file:
            json.dump(summary, output_file, indent=2, sort_keys=True)
            output_file.write('\n')
        print(json.dumps(summary, indent=2, sort_keys=True))

    def run(self):
        self.unpause()
        self.wait_for_pose()
        self.initial_pose = self.get_pose()
        self.settle('initial_settle')
        for index, waypoint in enumerate(self.waypoints, start=1):
            self.drive_to(waypoint, index)
            self.settle('waypoint_{:02d}_settle'.format(index))
        self.align_yaw(self.args.final_yaw)
        self.settle('final_yaw_settle')
        self.write_outputs()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-name', default='scout_mini')
    parser.add_argument('--model-states-topic', default='/gazebo/model_states')
    parser.add_argument('--command-topic', default='/scout_mini_velocity_controller/cmd_vel')
    parser.add_argument('--world-name', default='clearpath_playpen.world')
    parser.add_argument(
        '--waypoints',
        default=('0,-2;-5.5,-2;-8,0;-8,8.3;-1,8.3;-1,7.2;'
                 '4,7.2;5.5,8.3;5.5,0;4,-2;0,-3;0,-2;0,0'),
        help='semicolon-separated x,y waypoints in the Gazebo world frame',
    )
    parser.add_argument('--output-csv', required=True)
    parser.add_argument('--output-json', required=True)
    parser.add_argument('--wait-timeout', type=float, default=30.0)
    parser.add_argument('--control-rate', type=float, default=20.0)
    parser.add_argument('--settle-duration', type=float, default=1.0)
    parser.add_argument('--waypoint-timeout', type=float, default=90.0)
    parser.add_argument('--progress-timeout', type=float, default=12.0)
    parser.add_argument('--position-tolerance', type=float, default=0.18)
    parser.add_argument('--final-yaw', type=float, default=0.0)
    parser.add_argument('--yaw-tolerance', type=float, default=0.035)
    parser.add_argument('--progress-distance', type=float, default=0.25)
    parser.add_argument('--heading-gate', type=float, default=0.55)
    parser.add_argument('--linear-gain', type=float, default=0.65)
    parser.add_argument('--angular-gain', type=float, default=1.8)
    parser.add_argument('--min-linear', type=float, default=0.06)
    parser.add_argument('--max-linear', type=float, default=0.35)
    parser.add_argument('--max-angular', type=float, default=0.8)
    parser.add_argument('--unpause', action='store_true')
    return parser.parse_args(rospy.myargv()[1:])


def main():
    rospy.init_node('truth_guided_closed_loop', anonymous=True)
    controller = TruthGuidedController(parse_args())
    try:
        controller.run()
    finally:
        controller.publish_stop()


if __name__ == '__main__':
    main()
