#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import math
import threading

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Image
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped conversions)

try:
    from cv_bridge import CvBridge, CvBridgeError
except ImportError:  # Keep range-topic-only deployments usable on minimal Melodic images.
    CvBridge = None
    CvBridgeError = Exception


def _try_import_ultralytics(ultralytics_repo_dir: str):
    """Import ultralytics YOLO.

    Supports two cases:
    1) ultralytics already installed in environment
    2) ultralytics source tree exists in workspace; add to sys.path
    """
    try:
        from ultralytics import YOLO  # noqa: F401
        return YOLO
    except Exception:
        pass

    if ultralytics_repo_dir and os.path.isdir(ultralytics_repo_dir):
        sys.path.insert(0, ultralytics_repo_dir)

    from ultralytics import YOLO  # type: ignore

    return YOLO


def main():
    rospy.init_node("object_detection_pub", anonymous=False)

    # ---- Params ----
    camera_index = int(rospy.get_param("~camera_index", 0))
    hfov_deg = float(rospy.get_param("~hfov_deg", 66.1))

    # Default ultralytics dir.  In a devel space __file__ points into the
    # source tree; after `catkin_make_isolated --install` it points into
    # install_isolated/lib, so also try the sibling workspace src directory.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_candidates = [
        os.path.abspath(os.path.join(script_dir, "..", "..", "ultralytics_8_4_47")),
        os.path.abspath(os.path.join(script_dir, "..", "..", "..", "src",
                                     "ultralytics_8_4_47")),
    ]
    default_ultralytics_dir = next(
        (candidate for candidate in repo_candidates if os.path.isdir(candidate)),
        repo_candidates[0],
    )
    ultralytics_repo_dir = str(rospy.get_param("~ultralytics_repo_dir", default_ultralytics_dir))

    default_weights = os.path.join(
        default_ultralytics_dir, "runs", "detect", "train-2", "weights", "best.pt")
    weights = str(rospy.get_param(
        "~weights",
        default_weights,
    ))

    publish_rate_hz = float(rospy.get_param("~rate", 15.0))
    min_confidence = float(rospy.get_param("~min_confidence", 0.25))

    topic_angle = str(rospy.get_param("~topic_object_direction", "/object_direction"))
    topic_x = str(rospy.get_param("~topic_object_x", "/object_x"))
    topic_pose = str(rospy.get_param("~topic_object_detected", "/object_detected"))
    depth_topic = str(rospy.get_param("~depth_topic", "/camera/depth/image_raw"))
    range_topic = str(rospy.get_param("~range_topic", "/object_range"))
    output_frame = str(rospy.get_param("~camera_frame", "camera_link"))
    depth_scale = float(rospy.get_param("~depth_scale", 0.001))
    depth_timeout = float(rospy.get_param("~depth_timeout", 0.30))
    min_distance = float(rospy.get_param("~min_object_distance", 0.30))
    max_distance = float(rospy.get_param("~max_object_distance", 20.0))

    # ---- Publishers ----
    angle_pub = rospy.Publisher(topic_angle, Float32, queue_size=1)
    x_pub = rospy.Publisher(topic_x, Float32, queue_size=1)
    pose_pub = rospy.Publisher(topic_pose, PoseStamped, queue_size=1)

    # Depth/range are optional inputs.  A PoseStamped is published only when a
    # fresh, valid metric distance is available; a bearing alone is not a map
    # position and must never be converted using a guessed distance.
    state_lock = threading.Lock()
    depth_bridge = CvBridge() if CvBridge is not None else None
    depth_state = {"image": None, "stamp": rospy.Time(0), "frame": ""}
    range_state = {"value": None, "time": rospy.Time(0)}
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    def depth_callback(msg):
        try:
            image = depth_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "[YOLO-DEPTH] conversion failed: %s", exc)
            return
        with state_lock:
            depth_state["image"] = np.asarray(image)
            depth_state["stamp"] = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
            depth_state["frame"] = msg.header.frame_id

    def range_callback(msg):
        value = float(msg.data)
        if not np.isfinite(value):
            return
        with state_lock:
            range_state["value"] = value
            range_state["time"] = rospy.Time.now()

    depth_sub = None
    range_sub = None
    if depth_topic and depth_bridge is not None:
        depth_sub = rospy.Subscriber(depth_topic, Image, depth_callback, queue_size=1,
                                     buff_size=2 ** 24)
    elif depth_topic:
        rospy.logwarn("[YOLO-DEPTH] cv_bridge is unavailable; using range topic only.")
    if range_topic:
        range_sub = rospy.Subscriber(range_topic, Float32, range_callback, queue_size=1)

    # ---- Imports (cv2 + ultralytics) ----
    try:
        import cv2  # noqa: F401
    except Exception as e:
        rospy.logerr("Import cv2 failed: %s", e)
        return

    try:
        YOLO = _try_import_ultralytics(ultralytics_repo_dir)
    except Exception as e:
        rospy.logerr("Import ultralytics failed (repo_dir=%s): %s", ultralytics_repo_dir, e)
        return

    # ---- Model + Camera ----
    if not os.path.isfile(weights):
        rospy.logerr("Weights not found: %s", weights)
        return

    rospy.loginfo("Loading YOLO weights: %s", weights)
    model = YOLO(weights)

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        rospy.logerr("Cannot open camera index=%d", camera_index)
        return

    tan_half_hfov = math.tan(math.radians(hfov_deg / 2.0))

    rate = rospy.Rate(publish_rate_hz)
    rospy.loginfo("Publishing %s (deg), %s (pixel x), and %s (PoseStamped)",
                  topic_angle, topic_x, topic_pose)
    rospy.loginfo("Metric target inputs: depth=%s range=%s frame=%s", depth_topic,
                  range_topic, output_frame)

    while not rospy.is_shutdown():
        ok, frame = cap.read()
        if not ok or frame is None:
            rospy.logwarn_throttle(2.0, "Camera read failed")
            rate.sleep()
            continue

        img_h, img_w = frame.shape[:2]
        cx = img_w / 2.0

        results = model(frame, verbose=False)
        boxes = results[0].boxes if results and len(results) > 0 else None

        if boxes is None or len(boxes) == 0:
            rate.sleep()
            continue

        # Pick the highest-confidence detection above the configured threshold.
        try:
            confs = boxes.conf.cpu().numpy()
            xyxy = boxes.xyxy.cpu().numpy()
        except Exception:
            # Fallback for potential API differences
            confs = boxes.conf
            xyxy = boxes.xyxy

        valid_indices = [i for i, value in enumerate(confs) if float(value) >= min_confidence]
        if not valid_indices:
            rate.sleep()
            continue
        best_i = max(valid_indices, key=lambda i: float(confs[i]))
        x1, y1, x2, y2 = [float(v) for v in xyxy[best_i]]

        x_center = (x1 + x2) / 2.0

        # Convert center-x to horizontal bearing angle (deg)
        offset_ratio = (x_center - cx) / (img_w / 2.0)
        angle_rad = math.atan(offset_ratio * tan_half_hfov)
        angle_deg = math.degrees(angle_rad)

        angle_pub.publish(Float32(data=float(angle_deg)))
        x_pub.publish(Float32(data=float(x_center)))

        # Prefer a fresh depth ROI.  Depth images are commonly uint16 in mm;
        # floating-point images are interpreted as metres.
        distance_m = None
        distance_source = "none"
        depth_frame = ""
        depth_stamp = rospy.Time(0)
        with state_lock:
            depth_image = depth_state["image"]
            depth_stamp = depth_state["stamp"]
            depth_frame = depth_state["frame"]
            range_value = range_state["value"]
            range_time = range_state["time"]

        depth_age = (rospy.Time.now() - depth_stamp).to_sec() if not depth_stamp.is_zero() else -1.0
        if depth_image is not None and 0.0 <= depth_age <= depth_timeout:
            dh, dw = depth_image.shape[:2]
            dx1 = max(0, min(dw - 1, int(round(x1 * dw / float(img_w)))))
            dx2 = max(dx1 + 1, min(dw, int(round(x2 * dw / float(img_w)))))
            dy1 = max(0, min(dh - 1, int(round(y1 * dh / float(img_h)))))
            dy2 = max(dy1 + 1, min(dh, int(round(y2 * dh / float(img_h)))))
            # Avoid edges, which often contain background or the glass panel.
            rx1 = dx1 + int(0.25 * max(1, dx2 - dx1))
            rx2 = dx2 - int(0.25 * max(1, dx2 - dx1))
            ry1 = dy1 + int(0.25 * max(1, dy2 - dy1))
            ry2 = dy2 - int(0.25 * max(1, dy2 - dy1))
            roi = np.asarray(depth_image[ry1:max(ry1 + 1, ry2),
                                         rx1:max(rx1 + 1, rx2)])
            values = roi.astype(np.float32)
            if np.issubdtype(depth_image.dtype, np.integer):
                values *= depth_scale
            values = values[np.isfinite(values)]
            values = values[(values >= min_distance) & (values <= max_distance)]
            if values.size:
                distance_m = float(np.median(values))
                distance_source = "depth"

        range_age = (rospy.Time.now() - range_time).to_sec() if not range_time.is_zero() else -1.0
        if distance_m is None and range_value is not None and 0.0 <= range_age <= depth_timeout:
            if min_distance <= range_value <= max_distance:
                distance_m = float(range_value)
                distance_source = "range_topic"

        if distance_m is not None:
            pose = PoseStamped()
            if distance_source == "depth" and depth_frame:
                # Image/depth frames follow the ROS optical convention:
                # z-forward and x-right.  Build the point in that frame even
                # when depth_frame == output_frame; treating an optical frame
                # as x-forward would rotate the target by roughly 90 degrees.
                point = PointStamped()
                point.header.stamp = depth_stamp
                point.header.frame_id = depth_frame
                point.point.x = distance_m * math.tan(angle_rad)
                point.point.y = 0.0
                point.point.z = distance_m
                try:
                    if depth_frame == output_frame:
                        point_out = point
                    else:
                        point_out = tf_buffer.transform(point, output_frame,
                                                        rospy.Duration(0.05))
                    pose.header.stamp = depth_stamp
                    pose.header.frame_id = point_out.header.frame_id
                    pose.pose.position.x = point_out.point.x
                    pose.pose.position.y = point_out.point.y
                    pose.pose.position.z = point_out.point.z
                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                        tf2_ros.ConnectivityException) as exc:
                    rospy.logwarn_throttle(5.0,
                        "[YOLO-TF] Cannot transform depth point %s -> %s: %s",
                        depth_frame, output_frame, exc)
                    distance_m = None
            elif distance_source == "depth":
                rospy.logwarn_throttle(
                    5.0,
                    "[YOLO-DEPTH] Depth image has no frame_id; metric target is not published.")
                distance_m = None
            else:
                # Float32 range messages carry no frame.  The configured
                # camera_frame therefore follows the mobile-base convention:
                # x-forward and y-left.  Image bearings are positive to the
                # right, so the ROS y component uses the opposite sign.
                pose.header.stamp = range_time
                pose.header.frame_id = output_frame
                pose.pose.position.x = distance_m * math.cos(angle_rad)
                pose.pose.position.y = -distance_m * math.sin(angle_rad)
                pose.pose.position.z = 0.0

            if distance_m is not None:
                pose.pose.orientation.w = 1.0
                pose_pub.publish(pose)
                rospy.loginfo_throttle(
                    1.0,
                    "[YOLO-POSE] frame=%s source=%s distance=%.2f bearing=%.2f deg point=(%.2f,%.2f)",
                    pose.header.frame_id, distance_source, distance_m, angle_deg,
                    pose.pose.position.x, pose.pose.position.y)
        else:
            rospy.logwarn_throttle(
                5.0,
                "[YOLO-POSE] Detection has no fresh metric distance; %s is not published.",
                topic_pose)

        rate.sleep()

    cap.release()


if __name__ == "__main__":
    main()
