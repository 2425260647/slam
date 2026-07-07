#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud, PointCloud2


class PointCloudToPointCloud2:
    def __init__(self):
        input_topic = rospy.get_param("~input", "/velodyne_points_raw")
        output_topic = rospy.get_param("~output", "/velodyne_points")
        queue_size = rospy.get_param("~queue_size", 3)

        self.publisher = rospy.Publisher(output_topic, PointCloud2, queue_size=queue_size)
        self.subscriber = rospy.Subscriber(input_topic, PointCloud, self.callback, queue_size=queue_size)

    def callback(self, msg):
        points = [(p.x, p.y, p.z) for p in msg.points]
        cloud2 = pc2.create_cloud_xyz32(msg.header, points)
        self.publisher.publish(cloud2)


if __name__ == "__main__":
    rospy.init_node("pointcloud_to_pointcloud2")
    PointCloudToPointCloud2()
    rospy.spin()
