#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped


class TfPosePublisher:
    def __init__(self):
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        output_topic = rospy.get_param('~output_topic', '/tracked_pose')
        publish_rate = rospy.get_param('~publish_rate', 20.0)
        self.publisher = rospy.Publisher(
            output_topic, PoseStamped, queue_size=20
        )
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / publish_rate), self.publish_pose
        )

    def publish_pose(self, event):
        try:
            transform = self.buffer.lookup_transform(
                self.map_frame, self.base_frame, rospy.Time(0),
                rospy.Duration(0.01)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.publisher.publish(pose)


if __name__ == '__main__':
    rospy.init_node('tf_pose_publisher')
    TfPosePublisher()
    rospy.spin()
