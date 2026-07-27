#!/usr/bin/env python3
import argparse
import os
import sys

import rospy
from nav_msgs.msg import OccupancyGrid


def write_map(msg, output_prefix):
    width = msg.info.width
    height = msg.info.height
    if width == 0 or height == 0:
        raise RuntimeError("Received empty OccupancyGrid.")

    pgm_path = output_prefix + ".pgm"
    yaml_path = output_prefix + ".yaml"
    os.makedirs(os.path.dirname(os.path.abspath(output_prefix)), exist_ok=True)

    with open(pgm_path, "wb") as pgm:
        pgm.write(("P5\n# CREATOR: innovation1 save_occupancy_grid.py %.3f m/pix\n%d %d\n255\n" %
                   (msg.info.resolution, width, height)).encode("ascii"))
        # ROS OccupancyGrid stores row 0 at the map origin. PGM viewers expect
        # the first row to be the top of the image, so write rows flipped in Y.
        for y in range(height - 1, -1, -1):
            row = bytearray()
            for x in range(width):
                value = msg.data[y * width + x]
                if value < 0:
                    row.append(205)
                elif value >= 65:
                    row.append(0)
                elif value <= 19:
                    row.append(254)
                else:
                    row.append(205)
            pgm.write(row)

    origin = msg.info.origin
    with open(yaml_path, "w", encoding="utf-8") as yaml:
        yaml.write("image: %s\n" % os.path.basename(pgm_path))
        yaml.write("resolution: %.12g\n" % msg.info.resolution)
        yaml.write("origin: [%.12g, %.12g, %.12g]\n" %
                   (origin.position.x, origin.position.y, 0.0))
        yaml.write("negate: 0\n")
        yaml.write("occupied_thresh: 0.65\n")
        yaml.write("free_thresh: 0.196\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/map")
    parser.add_argument("--output-prefix", required=True)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    rospy.init_node("innovation1_save_occupancy_grid", anonymous=True)
    msg = rospy.wait_for_message(args.topic, OccupancyGrid,
                                 timeout=args.timeout)
    write_map(msg, args.output_prefix)
    print("Saved %s.pgm and %s.yaml" %
          (args.output_prefix, args.output_prefix))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("save_occupancy_grid failed: %s" % exc, file=sys.stderr)
        sys.exit(1)
