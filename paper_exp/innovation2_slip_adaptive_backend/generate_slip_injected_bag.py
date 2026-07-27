#!/usr/bin/env python3
import argparse
import copy
import math
import os

import rosbag
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def smoothstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def offset_profile(t):
    # [Innovation 2] Two mild artificial slip windows. Only /odom is modified;
    # the LiDAR point cloud is copied unchanged, so LiDAR geometry remains the
    # same across all experiment groups. The offsets are intentionally small:
    # lateral <= 0.35 m and yaw <= 0.10 rad.
    y = 0.0
    yaw = 0.0
    if t >= 90.0:
        s = smoothstep((min(t, 90.1) - 90.0) / 0.1)
        y += 0.25 * s
        yaw += 0.07 * s
    if t >= 190.0:
        s = smoothstep((min(t, 190.1) - 190.0) / 0.1)
        y += -0.35 * s
        yaw += -0.10 * s
    return y, yaw


def perturb_odometry(msg, elapsed):
    out = copy.deepcopy(msg)
    lateral_offset, yaw_bias = offset_profile(elapsed)
    out.pose.pose.position.y += lateral_offset

    q = out.pose.pose.orientation
    roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    nq = quaternion_from_euler(roll, pitch, yaw + yaw_bias)
    out.pose.pose.orientation.x = nq[0]
    out.pose.pose.orientation.y = nq[1]
    out.pose.pose.orientation.z = nq[2]
    out.pose.pose.orientation.w = nq[3]
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--cloud-topic", default="/velodyne_points")
    args = parser.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    first_time = None
    odom_count = 0
    cloud_count = 0
    with rosbag.Bag(args.input, "r") as in_bag, rosbag.Bag(args.output, "w",
                                                          compression="bz2") as out_bag:
        for topic, msg, stamp in in_bag.read_messages(
                topics=[args.odom_topic, args.cloud_topic]):
            if first_time is None:
                first_time = stamp
            elapsed = (stamp - first_time).to_sec()
            if topic == args.odom_topic:
                msg = perturb_odometry(msg, elapsed)
                odom_count += 1
            elif topic == args.cloud_topic:
                cloud_count += 1
            out_bag.write(topic, msg, stamp)

    print("Wrote slip-injected bag:", args.output)
    print("odom messages:", odom_count)
    print("cloud messages:", cloud_count)
    print("slip windows: 90-90.1s ramp +0.25m/+0.07rad, 190-190.1s ramp -0.35m/-0.10rad")


if __name__ == "__main__":
    main()
