#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class OdomToTf:
    def __init__(self):
        self.parent_frame = rospy.get_param('~parent_frame', 'odom')
        self.child_frame = rospy.get_param('~child_frame', 'base_link')
        odom_topic = rospy.get_param(
            '~odom_topic', '/scout_mini_velocity_controller/odom'
        )
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.subscriber = rospy.Subscriber(
            odom_topic, Odometry, self.callback, queue_size=100
        )

    def callback(self, message):
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


if __name__ == '__main__':
    rospy.init_node('odom_to_tf')
    OdomToTf()
    rospy.spin()
