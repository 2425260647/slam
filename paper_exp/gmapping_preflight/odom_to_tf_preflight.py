#!/usr/bin/env python3
"""Publish odom->base TF from a recorded nav_msgs/Odometry stream.

This adapter is intentionally limited to the GMapping preflight. It does not
rewrite the bag or alter the odometry message; it only exposes the TF contract
expected by the ROS1 GMapping wrapper.
"""

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class OdomToTf:
    def __init__(self):
        self.parent = rospy.get_param("~parent_frame", "odom")
        self.child = rospy.get_param("~child_frame", "base_footprint")
        self.topic = rospy.get_param("~odom_topic", "/odom")
        self.br = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber(self.topic, Odometry, self.callback, queue_size=100)

    def callback(self, msg):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.parent
        tf_msg.child_frame_id = self.child
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(tf_msg)


if __name__ == "__main__":
    rospy.init_node("odom_to_tf_preflight")
    OdomToTf()
    rospy.spin()
