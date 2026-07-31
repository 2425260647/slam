# Livox publish pointcloud message used by the M3DGR MID-360 bags.
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
