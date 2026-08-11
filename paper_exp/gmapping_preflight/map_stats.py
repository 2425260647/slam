#!/usr/bin/env python3
import sys
import rosbag


with rosbag.Bag(sys.argv[1]) as bag:
    maps = list(bag.read_messages(topics=['/map']))
    print('MAP_MESSAGES=%d' % len(maps))
    for _, msg, _ in maps[-2:]:
        occupied = sum(1 for value in msg.data if value >= 50)
        free = sum(1 for value in msg.data if 0 <= value < 50)
        unknown = sum(1 for value in msg.data if value < 0)
        print('MAP frame=%s size=%dx%d occupied=%d free=%d unknown=%d' %
              (msg.header.frame_id, msg.info.width, msg.info.height,
               occupied, free, unknown))
