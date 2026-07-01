#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""Save a stable /map snapshot after bag replay completes."""

from __future__ import print_function

import array
import hashlib
import math
import os

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from PIL import Image


class StableMapSaver(object):
    def __init__(self):
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.output_prefix = rospy.get_param("~output_prefix")
        self.min_runtime = float(rospy.get_param("~min_runtime", 180.0))
        self.settle_seconds = float(rospy.get_param("~settle_seconds", 10.0))
        self.check_period = float(rospy.get_param("~check_period", 1.0))
        self.min_known_ratio = float(rospy.get_param("~min_known_ratio", 0.05))

        self.first_map_time = None
        self.last_change_time = None
        self.last_hash = None
        self.last_known_ratio = 0.0
        self.saved = False
        self.latest_map = None

        self.sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(self.check_period), self.timer_cb)
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("[stable_map_saver] wait for stable map: topic=%s output=%s min_runtime=%.1fs settle=%.1fs",
                      self.map_topic, self.output_prefix, self.min_runtime, self.settle_seconds)

    def map_cb(self, msg):
        now = rospy.Time.now().to_sec()
        if self.first_map_time is None:
            self.first_map_time = now

        self.latest_map = msg
        payload = array.array('b', [int(v) for v in msg.data]).tostring()
        digest = hashlib.md5(payload).hexdigest()
        self.last_known_ratio = float(sum(1 for v in msg.data if v != -1)) / float(len(msg.data) or 1)

        if digest != self.last_hash:
            self.last_hash = digest
            self.last_change_time = now
            rospy.loginfo_throttle(5.0,
                                   "[stable_map_saver] map changed known_ratio=%.3f",
                                   self.last_known_ratio)

    def timer_cb(self, _event):
        if self.saved or self.first_map_time is None or self.last_change_time is None:
            return

        now = rospy.Time.now().to_sec()
        if now - self.first_map_time < self.min_runtime:
            return
        if now - self.last_change_time < self.settle_seconds:
            return
        if self.last_known_ratio < self.min_known_ratio:
            rospy.loginfo_throttle(5.0,
                                   "[stable_map_saver] wait more: known_ratio=%.3f < %.3f",
                                   self.last_known_ratio, self.min_known_ratio)
            return

        self.save_map()

    def save_map(self):
        if self.saved:
            return
        if self.latest_map is None:
            rospy.logwarn("[stable_map_saver] no map received, skip save")
            return
        self.saved = True

        out_dir = os.path.dirname(self.output_prefix)
        if out_dir and not os.path.isdir(out_dir):
            os.makedirs(out_dir)

        rospy.loginfo("[stable_map_saver] saving map to %s", self.output_prefix)
        try:
            self.write_map_files(self.latest_map)
            rospy.loginfo("[stable_map_saver] map saved")
        except Exception as exc:
            rospy.logerr("[stable_map_saver] failed to save map: %s", exc)
        finally:
            rospy.signal_shutdown("stable map saved")

    def write_map_files(self, msg):
        width = int(msg.info.width)
        height = int(msg.info.height)
        data = list(msg.data)
        img = np.ones((height, width), dtype=np.uint8) * 205
        for y in range(height):
            row = y * width
            for x in range(width):
                value = int(data[row + x])
                if value < 0:
                    img[y, x] = 205
                elif value >= 65:
                    img[y, x] = 0
                else:
                    img[y, x] = 254

        out_pgm = self.output_prefix + ".pgm"
        out_yaml = self.output_prefix + ".yaml"
        Image.fromarray(img).save(out_pgm)
        with open(out_yaml, "w") as f:
            image_name = os.path.basename(out_pgm)
            origin = msg.info.origin
            q = origin.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            f.write("image: %s\n" % image_name)
            f.write("resolution: %.6f\n" % msg.info.resolution)
            f.write("origin: [%.6f, %.6f, %.6f]\n" %
                    (origin.position.x, origin.position.y, yaw))
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.196\n")

    def on_shutdown(self):
        if self.saved:
            return
        if self.first_map_time is None:
            return
        rospy.loginfo("[stable_map_saver] shutdown fallback save")
        self.save_map()


def main():
    rospy.init_node("stable_map_saver")
    StableMapSaver()
    rospy.spin()


if __name__ == "__main__":
    main()
