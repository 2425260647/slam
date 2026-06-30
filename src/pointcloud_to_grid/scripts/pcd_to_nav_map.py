#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import math
import os
import struct
import sys

import numpy as np


class MapParams(object):
    def __init__(self,
                 resolution=0.05,
                 z_ground_percentile=10.0,
                 z_ground_band=0.08,
                 z_obstacle_min=0.15,
                 z_obstacle_max=1.80,
                 min_points_per_cell=2,
                 wall_close_cells=2,
                 obstacle_inflate_cells=1,
                 free_inflate_cells=1,
                 occupied_thresh=0.65,
                 free_thresh=0.196):
        self.resolution = resolution
        self.z_ground_percentile = z_ground_percentile
        self.z_ground_band = z_ground_band
        self.z_obstacle_min = z_obstacle_min
        self.z_obstacle_max = z_obstacle_max
        self.min_points_per_cell = min_points_per_cell
        self.wall_close_cells = wall_close_cells
        self.obstacle_inflate_cells = obstacle_inflate_cells
        self.free_inflate_cells = free_inflate_cells
        self.occupied_thresh = occupied_thresh
        self.free_thresh = free_thresh


class OccupancyGrid(object):
    def __init__(self, width, height, resolution, origin_x, origin_y, data):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.data = data


# ── PCD I/O ──────────────────────────────────────────────────────────────────

def load_pcd(path):
    with open(path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PCD header ended unexpectedly")
            try:
                text = _decode_ascii(line).strip()
            except UnicodeDecodeError:
                raise ValueError("Binary data before DATA line")
            header_lines.append(text)
            if text.startswith("DATA "):
                data_mode = text.split(None, 1)[1].strip()
                break
        header = _parse_pcd_header(header_lines)
        points = _read_pcd_points(f, header, data_mode)
    return points, header


def _decode_ascii(line):
    return line.decode("utf-8") if hasattr(line, "decode") else line


def _byte_value(b):
    return b if isinstance(b, int) else ord(b)


def _as_bytes(value):
    return str(value) if bytes is str else bytes(value)


def _parse_pcd_header(lines):
    h = {"fields": [], "size": [], "type": [], "count": []}
    for raw in lines:
        if not raw or raw.startswith("#"):
            continue
        parts = raw.split()
        key, rest = parts[0].upper(), parts[1:]
        if key == "FIELDS":
            h["fields"] = rest
        elif key == "SIZE":
            h["size"] = [int(v) for v in rest]
        elif key == "TYPE":
            h["type"] = rest
        elif key == "COUNT":
            h["count"] = [int(v) for v in rest]
        elif key in ("WIDTH", "HEIGHT", "POINTS"):
            h[key.lower()] = int(rest[0])
        elif key == "DATA":
            h["data"] = rest[0]
    for field in ("fields", "size", "type", "count", "width", "height", "points", "data"):
        if field not in h:
            raise ValueError("Missing PCD header field: %s" % field)
    return h


def _read_pcd_points(f, header, data_mode):
    fields = header["fields"]
    if list(fields) != ["x", "y", "z", "intensity"]:
        raise ValueError("Expected x y z intensity PCD, got %r" % (fields,))
    n = int(header["points"])
    if header["size"] != [4,4,4,4] or header["type"] != ["F","F","F","F"] or header["count"] != [1,1,1,1]:
        raise ValueError("Unsupported PCD layout")

    if data_mode == "ascii":
        rows = []
        for line in f:
            text = _decode_ascii(line).strip()
            if text:
                rows.append([float(v) for v in text.split()[:4]])
        arr = np.asarray(rows, dtype=np.float32)
        return arr[:n, :3]

    if data_mode == "binary":
        arr = np.frombuffer(f.read(n * 16), dtype=np.float32).reshape((-1, 4))
        return arr[:n, :3].copy()

    if data_mode == "binary_compressed":
        raw = f.read()
        if len(raw) < 8:
            raise ValueError("Compressed PCD payload too small")
        comp_sz, uncomp_sz = struct.unpack("<II", raw[:8])
        payload = raw[8:8 + comp_sz]
        data = _decompress_lzf(payload, uncomp_sz)
        block = n * 4
        arrays = [np.frombuffer(data[i*block:(i+1)*block], dtype=np.float32)
                  for i in range(len(fields))]
        arr = np.stack(arrays, axis=1)
        return arr[:n, :3].copy()

    raise ValueError("Unsupported PCD data mode: %s" % data_mode)


def _decompress_lzf(data, expected_size):
    out = bytearray()
    i = 0
    while i < len(data):
        ctrl = _byte_value(data[i]); i += 1
        if ctrl < 32:
            n = ctrl + 1
            out.extend(data[i:i + n]); i += n
        else:
            length = ctrl >> 5
            ref = len(out) - ((ctrl & 0x1F) << 8) - 1
            if length == 7:
                length += _byte_value(data[i]); i += 1
            ref -= _byte_value(data[i]); i += 1
            length += 2
            for _ in range(length):
                out.append(out[ref]); ref += 1
    if len(out) != expected_size:
        raise ValueError("LZF size mismatch: %d != %d" % (len(out), expected_size))
    return _as_bytes(out)


# ── Bag PointCloud2 I/O ────────────────────────────────────────────────────────

_PC2_NUMPY_TYPE = {
    (1, 1): np.int8,   (2, 1): np.uint8,
    (3, 2): np.int16,  (4, 2): np.uint16,
    (5, 4): np.int32,  (6, 4): np.uint32,
    (7, 4): np.float32, (8, 8): np.float64,
}


def _parse_pointcloud2(msg):
    """Extract finite (x, y, z) points from a sensor_msgs/PointCloud2 message.

    Reads the raw byte buffer with the stride declared by point_step and slices
    the x/y/z float fields directly, so it works for any field layout the driver
    emits (xyz, xyzi, xyzir, etc.).
    """
    offsets = {}
    for field in msg.fields:
        if field.name in ("x", "y", "z"):
            offsets[field.name] = (field.offset, field.datatype)
    if not all(k in offsets for k in ("x", "y", "z")):
        return None

    count = msg.width * msg.height
    if count == 0:
        return None

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    raw = raw[: count * msg.point_step].reshape(count, msg.point_step)

    pts = np.zeros((count, 3), dtype=np.float32)
    for axis, name in enumerate(("x", "y", "z")):
        offset, datatype = offsets[name]
        np_type = _PC2_NUMPY_TYPE[(datatype, 4)] if datatype == 7 else _PC2_NUMPY_TYPE.get(
            (datatype, np.dtype(np.float32).itemsize), np.float32)
        size = np.dtype(np_type).itemsize
        column = raw[:, offset:offset + size].copy().view(np_type).reshape(-1)
        pts[:, axis] = column.astype(np.float32)

    finite = np.isfinite(pts).all(axis=1)
    return pts[finite]


def _quaternion_to_matrix(qx, qy, qz, qw):
    """Build a 3x3 rotation matrix from a quaternion."""
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-9:
        return np.eye(3, dtype=np.float64)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float64)


def _transform_to_matrix(transform):
    """Convert a geometry_msgs/Transform into a 4x4 homogeneous matrix."""
    t = transform.translation
    r = transform.rotation
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = _quaternion_to_matrix(r.x, r.y, r.z, r.w)
    mat[:3, 3] = (t.x, t.y, t.z)
    return mat


def load_bag_pointcloud(bag_path, cloud_topic, target_frame, sensor_frame,
                        voxel_size=0.05, max_clouds=0):
    """Accumulate all PointCloud2 scans from a bag into one target-frame cloud.

    A first pass loads every TF (static and dynamic) into a tf2 buffer. A second
    pass reads each cloud, looks up target_frame<-cloud_frame at the cloud stamp,
    applies it, and concatenates the result. Points are voxel-downsampled to keep
    the accumulated array bounded.
    """
    import rosbag
    import rospy
    import tf2_ros

    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(1e9))
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, _ in bag.read_messages(topics=["/tf", "/tf_static"]):
            is_static = topic == "/tf_static"
            for transform in msg.transforms:
                if is_static:
                    tf_buffer.set_transform_static(transform, "bag")
                else:
                    tf_buffer.set_transform(transform, "bag")

        chunks = []
        used = 0
        for _, msg, _ in bag.read_messages(topics=[cloud_topic]):
            pts = _parse_pointcloud2(msg)
            if pts is None or pts.shape[0] == 0:
                continue

            cloud_frame = msg.header.frame_id or sensor_frame
            try:
                tf_stamped = tf_buffer.lookup_transform(
                    target_frame, cloud_frame, msg.header.stamp, rospy.Duration(0.0))
            except Exception:
                try:
                    tf_stamped = tf_buffer.lookup_transform(
                        target_frame, cloud_frame, rospy.Time(0), rospy.Duration(0.0))
                except Exception:
                    continue

            mat = _transform_to_matrix(tf_stamped.transform)
            homo = np.empty((pts.shape[0], 4), dtype=np.float64)
            homo[:, :3] = pts
            homo[:, 3] = 1.0
            world = homo.dot(mat.T)[:, :3].astype(np.float32)
            chunks.append(world)
            used += 1
            if max_clouds and used >= max_clouds:
                break

    if not chunks:
        raise ValueError("No transformable clouds on topic %s" % cloud_topic)

    points = np.vstack(chunks)
    return _voxel_downsample(points, voxel_size)


def _voxel_downsample(points, voxel_size):
    """Keep one representative point per voxel to bound memory."""
    if voxel_size <= 0.0 or points.shape[0] == 0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, unique_idx = np.unique(
        keys[:, 0] * 73856093 ^ keys[:, 1] * 19349663 ^ keys[:, 2] * 83492791,
        return_index=True)
    return points[unique_idx]


# ── Morphological operations ──────────────────────────────────────────────────

def _inflate_mask(mask, radius):
    """Binary dilation: expand True regions by radius cells."""
    if radius <= 0:
        return mask.copy()
    h, w = mask.shape
    out = mask.copy()
    ys, xs = np.where(mask)
    for y, x in zip(ys.tolist(), xs.tolist()):
        out[max(0, y-radius):min(h, y+radius+1),
            max(0, x-radius):min(w, x+radius+1)] = True
    return out


def _erode_mask(mask, radius):
    """Binary erosion via 2D integral image (O(H*W) complexity).

    A cell survives only if its entire (2*radius+1)^2 neighbourhood is True.
    Border cells that extend outside the image are treated as False (zero padding),
    so they are eroded away.
    """
    if radius <= 0:
        return mask.copy()
    h, w = mask.shape
    r = radius
    kernel_area = (2 * r + 1) ** 2
    # Zero-pad on all sides so border cells fail the full-neighbourhood check
    padded = np.zeros((h + 2*r, w + 2*r), dtype=np.float32)
    padded[r:r+h, r:r+w] = mask.astype(np.float32)
    # Integral image (II[i,j] = sum of padded[0..i-1, 0..j-1])
    II = np.zeros((h + 2*r + 1, w + 2*r + 1), dtype=np.float32)
    II[1:, 1:] = padded.cumsum(axis=0).cumsum(axis=1)
    # Box sum for each original cell (y, x) covers padded rows [y, y+2r]
    box_sum = (II[2*r+1:h+2*r+1, 2*r+1:w+2*r+1]
             - II[0:h,           2*r+1:w+2*r+1]
             - II[2*r+1:h+2*r+1, 0:w          ]
             + II[0:h,           0:w           ])
    return (box_sum >= kernel_area - 0.5) & mask


def _morphological_close(mask, radius):
    """Fill gaps narrower than 2*radius cells (dilation then erosion)."""
    if radius <= 0:
        return mask.copy()
    return _erode_mask(_inflate_mask(mask, radius), radius)


# ── Noise filters ─────────────────────────────────────────────────────────────

def _remove_small_components(mask, min_area):
    """Remove connected obstacle blobs smaller than min_area cells."""
    h, w = mask.shape
    visited = np.zeros((h, w), dtype=np.uint8)
    out = np.zeros((h, w), dtype=bool)
    for sy in range(h):
        for sx in range(w):
            if not mask[sy, sx] or visited[sy, sx]:
                continue
            stack = [(sy, sx)]
            visited[sy, sx] = 1
            cells = []
            while stack:
                y, x = stack.pop()
                cells.append((y, x))
                for ny in range(max(0, y-1), min(h, y+2)):
                    for nx in range(max(0, x-1), min(w, x+2)):
                        if mask[ny, nx] and not visited[ny, nx]:
                            visited[ny, nx] = 1
                            stack.append((ny, nx))
            if len(cells) >= min_area:
                for y, x in cells:
                    out[y, x] = True
    return out


def _majority_filter(mask, min_neighbors=3):
    """Smooth isolated noise: remove lone True cells, fill dense False cells."""
    h, w = mask.shape
    out = mask.copy()
    for y in range(h):
        for x in range(w):
            y0, y1 = max(0, y-1), min(h, y+2)
            x0, x1 = max(0, x-1), min(w, x+2)
            nbrs = int(mask[y0:y1, x0:x1].sum()) - int(mask[y, x])
            if mask[y, x]:
                if nbrs < 1:
                    out[y, x] = False
            else:
                if nbrs >= min_neighbors:
                    out[y, x] = True
    return out


# ── Core pipeline ─────────────────────────────────────────────────────────────

def build_occupancy_grid(points, params):
    if points.size == 0:
        raise ValueError("Point cloud is empty")

    xs, ys, zs = points[:, 0], points[:, 1], points[:, 2]

    # Estimate ground plane from lowest percentile of z values
    z_ground = float(np.percentile(zs, params.z_ground_percentile))
    obstacle_mask = ((zs >= z_ground + params.z_obstacle_min) &
                     (zs <= z_ground + params.z_obstacle_max))
    ground_mask = zs <= (z_ground + params.z_ground_band)

    margin = params.resolution * 2.0
    min_x, max_x = float(xs.min()) - margin, float(xs.max()) + margin
    min_y, max_y = float(ys.min()) - margin, float(ys.max()) + margin
    width  = int(math.ceil((max_x - min_x) / params.resolution)) + 1
    height = int(math.ceil((max_y - min_y) / params.resolution)) + 1

    # Vectorised grid voting
    u = np.clip(np.floor((xs - min_x) / params.resolution).astype(np.int32), 0, width  - 1)
    v = np.clip(np.floor((ys - min_y) / params.resolution).astype(np.int32), 0, height - 1)

    occupied_count = np.zeros((height, width), dtype=np.int32)
    free_count     = np.zeros((height, width), dtype=np.int32)
    np.add.at(occupied_count, (v[obstacle_mask], u[obstacle_mask]), 1)
    np.add.at(free_count,     (v[ground_mask],   u[ground_mask]),   1)

    occupied = occupied_count >= params.min_points_per_cell
    free     = free_count     >= params.min_points_per_cell

    # Close wall gaps: fills discontinuities narrower than 2*wall_close_cells
    if params.wall_close_cells > 0:
        occupied = _morphological_close(occupied, params.wall_close_cells)

    # Noise removal and smoothing
    occupied = _remove_small_components(occupied, max(4, params.min_points_per_cell))
    occupied = _majority_filter(occupied, 3)

    # Inflate obstacles for navigation clearance, expand free space
    occupied = _inflate_mask(occupied, params.obstacle_inflate_cells)
    free     = _inflate_mask(free,     params.free_inflate_cells)
    free     = free & ~occupied

    data = np.full((height, width), -1, dtype=np.int8)
    data[free]     = 0
    data[occupied] = 100

    return OccupancyGrid(width, height, params.resolution, min_x, min_y, data.reshape(-1))


# ── Map output ────────────────────────────────────────────────────────────────

def save_map(grid, output_prefix):
    out_dir = os.path.dirname(os.path.abspath(output_prefix))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir)

    image_path = _replace_suffix(output_prefix, ".pgm")
    yaml_path  = _replace_suffix(output_prefix, ".yaml")

    image = np.full((grid.height, grid.width), 205, dtype=np.uint8)
    grid2d = grid.data.reshape((grid.height, grid.width))
    image[grid2d == 0]   = 254   # free  -> white
    image[grid2d == 100] = 0     # wall  -> black
    _write_pgm(image_path, image)

    defaults = MapParams()
    yaml_text = (
        "image: {img}\n"
        "resolution: {res:.6f}\n"
        "origin: [{ox:.6f}, {oy:.6f}, 0.000000]\n"
        "negate: 0\n"
        "occupied_thresh: {occ:.3f}\n"
        "free_thresh: {free:.3f}\n"
    ).format(
        img=os.path.basename(image_path),
        res=grid.resolution,
        ox=grid.origin_x, oy=grid.origin_y,
        occ=defaults.occupied_thresh,
        free=defaults.free_thresh,
    )
    with open(yaml_path, "w") as f:
        f.write(yaml_text)

    return image_path, yaml_path


def _replace_suffix(path, suffix):
    return os.path.splitext(path)[0] + suffix


def _write_pgm(path, image):
    h, w = image.shape
    with open(path, "wb") as f:
        f.write(b"P5\n# generated by pointcloud_to_grid\n")
        f.write(("%d %d\n255\n" % (w, h)).encode("ascii"))
        f.write(image.tostring(order="C"))


# ── CLI ───────────────────────────────────────────────────────────────────────

def _script_dir():
    return os.path.dirname(os.path.abspath(__file__))

def _ancestor(path, levels):
    p = os.path.abspath(path)
    for _ in range(levels):
        p = os.path.dirname(p)
    return p

def _default_pcd():
    return os.path.join(_ancestor(_script_dir(), 4), "output_downsampled.pcd")

def _default_output():
    return os.path.join(_ancestor(_script_dir(), 2), "my_navigation", "maps", "output_map")

def _make_parser():
    p = argparse.ArgumentParser(description="Convert 3D point cloud to ROS map_server occupancy grid")
    src = p.add_mutually_exclusive_group(required=False)
    src.add_argument("--pcd",  default=None,
                     help="path to a PCD file (A-LOAM / map_saver output)")
    src.add_argument("--bag",  default=None,
                     help="path to a ROS bag file containing /velodyne_points")
    p.add_argument("--cloud-topic",   default="/velodyne_points",
                   help="PointCloud2 topic inside the bag (default: /velodyne_points)")
    p.add_argument("--target-frame",  default="map",
                   help="TF frame into which clouds are accumulated (default: map)")
    p.add_argument("--sensor-frame",  default="laser_link",
                   help="fallback frame when cloud header.frame_id is empty (default: laser_link)")
    p.add_argument("--voxel-size",    type=float, default=0.05,
                   help="voxel down-sample size in metres before grid building (default: 0.05)")
    p.add_argument("--output",                   default=_default_output())
    p.add_argument("--resolution",               type=float, default=0.05)
    p.add_argument("--z-ground-percentile",      type=float, default=10.0)
    p.add_argument("--z-ground-band",            type=float, default=0.08)
    p.add_argument("--z-obstacle-min",           type=float, default=0.15)
    p.add_argument("--z-obstacle-max",           type=float, default=1.80)
    p.add_argument("--min-points-per-cell",      type=int,   default=2)
    p.add_argument("--wall-close-cells",         type=int,   default=2,
                   help="morphological closing radius to fill wall gaps (cells)")
    p.add_argument("--obstacle-inflate-cells",   type=int,   default=1)
    p.add_argument("--free-inflate-cells",       type=int,   default=1)
    return p

def _strip_rosargs(argv):
    return [a for a in argv if ":=" not in a]

def main(argv=None):
    args = _make_parser().parse_args(_strip_rosargs(sys.argv[1:] if argv is None else argv))
    params = MapParams(
        resolution=args.resolution,
        z_ground_percentile=args.z_ground_percentile,
        z_ground_band=args.z_ground_band,
        z_obstacle_min=args.z_obstacle_min,
        z_obstacle_max=args.z_obstacle_max,
        min_points_per_cell=args.min_points_per_cell,
        wall_close_cells=args.wall_close_cells,
        obstacle_inflate_cells=args.obstacle_inflate_cells,
        free_inflate_cells=args.free_inflate_cells,
    )

    if args.bag is not None:
        import rospy
        if not rospy.core.is_initialized():
            rospy.init_node("pcd_to_nav_map", anonymous=True, disable_signals=True)
        points = load_bag_pointcloud(
            args.bag,
            cloud_topic=args.cloud_topic,
            target_frame=args.target_frame,
            sensor_frame=args.sensor_frame,
            voxel_size=args.voxel_size,
        )
        source_desc = args.bag
        n_points = points.shape[0]
    else:
        pcd_path = args.pcd if args.pcd else _default_pcd()
        points, header = load_pcd(pcd_path)
        source_desc = pcd_path
        n_points = header["points"]

    grid = build_occupancy_grid(points, params)
    image_path, yaml_path = save_map(grid, args.output)
    print("Loaded %d points from %s" % (n_points, source_desc))
    print("Output map:  %s" % image_path)
    print("Output yaml: %s" % yaml_path)
    print("Grid size:   %dx%d @ %.3fm/cell" % (grid.width, grid.height, grid.resolution))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
