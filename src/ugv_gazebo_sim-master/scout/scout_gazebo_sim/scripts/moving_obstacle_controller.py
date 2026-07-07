#!/usr/bin/env python3
import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Quaternion


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class MovingObstacleController:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "dynamic_person")
        self.speed = rospy.get_param("~speed", 0.45)
        path_x = rospy.get_param("~path_x", [1.5, 1.5, -1.5, -1.5])
        path_y = rospy.get_param("~path_y", [-1.5, 1.5, 1.5, -1.5])
        self.height_center = rospy.get_param("~height_center", 0.85)
        self.waypoints = list(zip(path_x, path_y))
        if len(self.waypoints) < 2:
            raise RuntimeError("moving obstacle requires at least two waypoints")

        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.segment = 0
        self.x, self.y = self.waypoints[0]
        self.last_time = rospy.Time.now()

    def step(self, event):
        now = event.current_real
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        if dt <= 0.0:
            return

        target = self.waypoints[(self.segment + 1) % len(self.waypoints)]
        dx = target[0] - self.x
        dy = target[1] - self.y
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            self.segment = (self.segment + 1) % len(self.waypoints)
            return

        step = min(self.speed * dt, dist)
        yaw = math.atan2(dy, dx)
        self.x += step * dx / dist
        self.y += step * dy / dist
        if step >= dist - 1e-4:
            self.segment = (self.segment + 1) % len(self.waypoints)

        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = "world"
        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.position.z = self.height_center
        state.pose.orientation = yaw_to_quaternion(yaw)
        try:
            self.set_state(state)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "Failed to move obstacle: %s", exc)


if __name__ == "__main__":
    rospy.init_node("moving_obstacle_controller")
    controller = MovingObstacleController()
    rospy.Timer(rospy.Duration(0.1), controller.step)
    rospy.spin()
