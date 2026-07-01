#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""Filter PointCloud2 before pointcloud_to_laserscan for source-side projection optimization."""

from __future__ import print_function

import math

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2 as pc2


class ProjectionOptimizer(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/velodyne_points")
        self.output_topic = rospy.get_param("~output_topic", "/velodyne_points_optimized")
        self.z_ground_band = rospy.get_param("~z_ground_band", 0.06)
        self.z_obstacle_min = rospy.get_param("~z_obstacle_min", 0.08)
        self.z_obstacle_max = rospy.get_param("~z_obstacle_max", 1.60)
        self.z_ground_percentile = rospy.get_param("~z_ground_percentile", 10.0)
        self.voxel_leaf_xy = rospy.get_param("~voxel_leaf_xy", 0.05)
        self.voxel_leaf_z = rospy.get_param("~voxel_leaf_z", 0.10)
        self.min_points_per_voxel = rospy.get_param("~min_points_per_voxel", 2)
        self.max_sample_points = rospy.get_param("~max_sample_points", 120000)
        self.frame_id = rospy.get_param("~frame_id", "")
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self.callback, queue_size=1)
        self.seq = 0

    def voxel_key(self, x, y, z):
        return (
            int(math.floor(x / self.voxel_leaf_xy)),
            int(math.floor(y / self.voxel_leaf_xy)),
            int(math.floor(z / self.voxel_leaf_z)),
        )

    def callback(self, msg):
        pts = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pts.append((float(p[0]), float(p[1]), float(p[2])))
            if len(pts) >= self.max_sample_points:
                break

        if not pts:
            return

        zs = np.array([p[2] for p in pts], dtype=np.float32)
        z_ground = float(np.percentile(zs, self.z_ground_percentile))
        filtered = []
        voxel_counts = {}
        voxel_points = {}

        for x, y, z in pts:
            z_rel = z - z_ground
            if z_rel < self.z_ground_band:
                continue
            if z_rel < self.z_obstacle_min or z_rel > self.z_obstacle_max:
                continue
            key = self.voxel_key(x, y, z)
            voxel_counts[key] = voxel_counts.get(key, 0) + 1
            voxel_points.setdefault(key, []).append((x, y, z))

        for key, count in voxel_counts.items():
            if count < self.min_points_per_voxel:
                continue
            arr = voxel_points[key]
            xs = [p[0] for p in arr]
            ys = [p[1] for p in arr]
            zs = [p[2] for p in arr]
            filtered.append((sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs)))

        if not filtered:
            return

        header = msg.header
        if self.frame_id:
            header.frame_id = self.frame_id
        cloud = pc2.create_cloud_xyz32(header, filtered)
        self.pub.publish(cloud)
        self.seq += 1
        if self.seq % 20 == 0:
            rospy.loginfo("projection optimizer: raw=%d filtered=%d z_ground=%.3f", len(pts), len(filtered), z_ground)


def main():
    rospy.init_node("pointcloud_projection_optimizer")
    ProjectionOptimizer()
    rospy.spin()


if __name__ == "__main__":
    main()
