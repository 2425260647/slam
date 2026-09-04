#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import sys
import threading
from pathlib import Path

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
try:
    from cv_bridge import CvBridge, CvBridgeError
except ImportError:
    CvBridge = None
    CvBridgeError = Exception


# Camera parameters: image width 640 px, horizontal FOV 66.1 deg.
HFOV = 66.1
W = 640
cx = W / 2.0
tan_half_hfov = math.tan(math.radians(HFOV / 2.0))

ROOT_DIR = Path(__file__).resolve().parent
MODEL_PATH = ROOT_DIR / "runs" / "detect" / "train-2" / "weights" / "best.pt"


def load_yolo(repo_dir):
    """Load the pinned workspace copy before falling back to site-packages.

    The trained weights are tied to the bundled 8.4.47 implementation.  A
    Conda environment may also contain an unrelated Ultralytics version, so
    prefer the explicit package path supplied by the launch file for
    reproducible inference.
    """
    if repo_dir and os.path.isdir(os.path.join(repo_dir, "ultralytics")):
        sys.path.insert(0, repo_dir)
        try:
            from ultralytics import YOLO  # type: ignore
            return YOLO
        except Exception:
            # Remove only the path we added; the environment fallback below
            # remains available when optional bundled dependencies are absent.
            if sys.path and sys.path[0] == repo_dir:
                sys.path.pop(0)

    from ultralytics import YOLO  # type: ignore
    return YOLO


def as_bool(value):
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def decode_depth_image(msg):
    """Decode common depth encodings without loading cv_bridge's C++ module.

    ROS Melodic installations commonly provide cv_bridge only for Python2.
    The raw sensor_msgs/Image payload is sufficient for the ROI median used
    here, so keep this path pure Python3/numpy.
    """
    encoding = str(msg.encoding).lower()
    if encoding in ("16uc1", "mono16"):
        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
    elif encoding in ("32fc1", "32fc"):
        dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
    else:
        raise ValueError("unsupported depth encoding: {}".format(msg.encoding))

    width = int(msg.width)
    height = int(msg.height)
    row_step = int(msg.step) if int(msg.step) > 0 else width * dtype.itemsize
    required = (height - 1) * row_step + width * dtype.itemsize
    payload = msg.data if isinstance(msg.data, (bytes, bytearray, memoryview)) else bytes(msg.data)
    if width <= 0 or height <= 0 or len(payload) < required:
        raise ValueError("invalid depth payload size")

    rows = [np.frombuffer(payload, dtype=dtype, count=width,
                          offset=row * row_step)
            for row in range(height)]
    return np.asarray(rows)


def main():
    rospy.init_node("ultralytics_object_detector", anonymous=False)

    camera_index = int(rospy.get_param("~camera_index", 0))
    model_path = rospy.get_param("~weights", str(MODEL_PATH))
    ultralytics_repo_dir = rospy.get_param("~ultralytics_repo_dir", str(ROOT_DIR))
    hfov_deg = float(rospy.get_param("~hfov_deg", HFOV))
    min_confidence = float(rospy.get_param("~min_confidence", 0.25))
    target_class = str(rospy.get_param("~target_class", "mouse")).strip().lower()
    target_class_id = int(rospy.get_param("~target_class_id", -1))
    show_window = as_bool(rospy.get_param("~show_window", False))
    direction_topic = rospy.get_param("~topic_object_direction", "/object_direction")
    x_topic = rospy.get_param("~topic_object_x", "/object_x")
    angle_pub = rospy.Publisher(direction_topic, Float32, queue_size=1)
    x_pub = rospy.Publisher(x_topic, Float32, queue_size=1)
    area_topic = rospy.get_param("~topic_object_area", "/object_area")
    area_pub = rospy.Publisher(area_topic, Float32, queue_size=1)
    pose_topic = rospy.get_param("~topic_object_detected", "/object_detected")
    pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=1)
    depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
    range_topic = rospy.get_param("~range_topic", "/object_range")
    camera_frame = rospy.get_param("~camera_frame", "camera_link")
    depth_scale = float(rospy.get_param("~depth_scale", 0.001))
    depth_timeout = float(rospy.get_param("~depth_timeout", 0.30))
    min_distance = float(rospy.get_param("~min_object_distance", 0.30))
    max_distance = float(rospy.get_param("~max_object_distance", 20.0))
    state_lock = threading.Lock()
    depth_bridge = CvBridge() if CvBridge is not None else None
    depth_state = {"image": None, "stamp": rospy.Time(0), "frame": ""}
    latest_range = {"value": None, "time": rospy.Time(0)}

    def depth_callback(msg):
        image = None
        try:
            if depth_bridge is not None:
                image = depth_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            else:
                image = decode_depth_image(msg)
        except (CvBridgeError, ValueError, TypeError) as exc:
            rospy.logwarn_throttle(5.0, "[YOLO-DEPTH] conversion failed: %s", exc)
            return
        with state_lock:
            depth_state["image"] = np.asarray(image)
            depth_state["stamp"] = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
            depth_state["frame"] = msg.header.frame_id

    def range_callback(msg):
        value = float(msg.data)
        if math.isfinite(value) and min_distance <= value <= max_distance:
            latest_range["value"] = value
            latest_range["time"] = rospy.Time.now()

    if range_topic:
        rospy.Subscriber(range_topic, Float32, range_callback, queue_size=1)
    if depth_topic:
        rospy.Subscriber(depth_topic, Image, depth_callback, queue_size=1,
                         buff_size=2 ** 24)
    if depth_topic and depth_bridge is None:
        rospy.logwarn("[YOLO-DEPTH] cv_bridge unavailable; using pure numpy Image decoding.")

    try:
        YOLO = load_yolo(ultralytics_repo_dir)
    except Exception as exc:
        rospy.logerr("[YOLO-PUB] failed to import Ultralytics: %s", exc)
        return

    if not os.path.isfile(str(model_path)):
        rospy.logerr("[YOLO-PUB] weights not found: %s", model_path)
        return
    rospy.loginfo("[YOLO-PUB] loading model: %s", model_path)
    model = YOLO(str(model_path))
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        rospy.logerr("[YOLO-PUB] failed to open camera index %d", camera_index)
        return

    rospy.loginfo("[YOLO-PUB] publishing angle_deg on %s", direction_topic)
    rospy.loginfo("[YOLO-PUB] publishing x_center on %s", x_topic)
    rospy.loginfo("[YOLO-PUB] publishing bbox/image area ratio on %s", area_topic)
    rospy.loginfo("[YOLO-PUB] target class=%s%s confidence>=%.2f display=%s",
                  target_class,
                  " (id={})".format(target_class_id) if target_class_id >= 0 else "",
                  min_confidence, show_window)
    rospy.loginfo("[YOLO-PUB] publishing metric poses on %s only when %s is fresh", pose_topic, range_topic)

    frame_count = 0
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("[YOLO-PUB] camera frame read failed")
            break
        frame_count += 1

        results = model(frame, verbose=False)
        result = results[0] if results else None
        boxes = result.boxes if result is not None else None

        current_detections = []

        model_names = getattr(model, "names", None)
        if model_names is None:
            model_names = getattr(result, "names", {}) if result is not None else {}

        def class_name(class_id):
            if isinstance(model_names, dict):
                return str(model_names.get(int(class_id), ""))
            if isinstance(model_names, (list, tuple)) and 0 <= int(class_id) < len(model_names):
                return str(model_names[int(class_id)])
            return ""

        if boxes is not None:
            class_ids = boxes.cls.cpu().numpy().astype(int)
            confs = boxes.conf.cpu().numpy()
            xyxy = boxes.xyxy.cpu().numpy()

            for cls_id, conf, (x1, y1, x2, y2) in zip(class_ids, confs, xyxy):
                if float(conf) < min_confidence:
                    continue
                name = class_name(cls_id).strip().lower()
                if target_class_id >= 0:
                    if int(cls_id) != target_class_id:
                        continue
                elif name != target_class:
                    continue
                detection = {
                    "class_id": int(cls_id),
                    "class_name": name,
                    "confidence": float(conf),
                    "x1": float(x1),
                    "y1": float(y1),
                    "x2": float(x2),
                    "y2": float(y2),
                }
                current_detections.append(detection)

        if len(current_detections) != 0:
            det = max(current_detections, key=lambda item: item["confidence"])
            x_center = (det["x1"] + det["x2"]) / 2.0
            y_center = (det["y1"] + det["y2"]) / 2.0

            frame_width = float(frame.shape[1])
            frame_cx = frame_width / 2.0
            frame_tan_half_hfov = math.tan(math.radians(hfov_deg / 2.0))
            offset_ratio = (x_center - frame_cx) / (frame_width / 2.0)
            angle_rad = math.atan(offset_ratio * frame_tan_half_hfov)
            angle_deg = math.degrees(angle_rad)
            bbox_area = max(0.0, (det["x2"] - det["x1"]) *
                            (det["y2"] - det["y1"]))
            image_area = max(1.0, float(frame.shape[0] * frame.shape[1]))
            area_ratio = bbox_area / image_area

            angle_pub.publish(Float32(data=float(angle_deg)))
            x_pub.publish(Float32(data=float(x_center)))
            area_pub.publish(Float32(data=float(area_ratio)))

            with state_lock:
                depth_image = depth_state["image"]
                depth_stamp = depth_state["stamp"]
                depth_frame = depth_state["frame"]
                range_value = latest_range["value"]
                range_time = latest_range["time"]

            distance_m = None
            distance_source = "none"
            depth_age = ((rospy.Time.now() - depth_stamp).to_sec()
                         if not depth_stamp.is_zero() else -1.0)
            if depth_image is not None and 0.0 <= depth_age <= depth_timeout:
                dh, dw = depth_image.shape[:2]
                dx1 = max(0, min(dw - 1, int(round(det["x1"] * dw / float(frame.shape[1])))))
                dx2 = max(dx1 + 1, min(dw, int(round(det["x2"] * dw / float(frame.shape[1])))))
                dy1 = max(0, min(dh - 1, int(round(det["y1"] * dh / float(frame.shape[0])))))
                dy2 = max(dy1 + 1, min(dh, int(round(det["y2"] * dh / float(frame.shape[0])))))
                roi = np.asarray(depth_image[dy1:dy2, dx1:dx2]).astype(np.float32)
                if np.issubdtype(depth_image.dtype, np.integer):
                    roi *= depth_scale
                values = roi[np.isfinite(roi)]
                values = values[(values >= min_distance) & (values <= max_distance)]
                if values.size:
                    distance_m = float(np.median(values))
                    distance_source = "depth"

            range_age = ((rospy.Time.now() - range_time).to_sec()
                         if not range_time.is_zero() else -1.0)
            if distance_m is None and range_value is not None and 0.0 <= range_age <= depth_timeout:
                if min_distance <= range_value <= max_distance:
                    distance_m = float(range_value)
                    distance_source = "range_topic"

            if distance_m is not None:
                pose = PoseStamped()
                if distance_source == "depth" and depth_frame:
                    # ROS optical frame: z-forward, x-right.  Positive image
                    # bearing is to the right, so x is positive here.  Keep
                    # the original depth frame; navigation_driver (C++)
                    # performs the TF transform to map without Python2
                    # tf2_py being loaded in this Python3 process.
                    pose.header.stamp = depth_stamp
                    pose.header.frame_id = depth_frame
                    pose.pose.position.x = distance_m * math.tan(angle_rad)
                    pose.pose.position.y = 0.0
                    pose.pose.position.z = distance_m
                else:
                    pose.header.stamp = range_time
                    pose.header.frame_id = camera_frame
                    pose.pose.position.x = distance_m * math.cos(angle_rad)
                    # Image x grows right while REP-103 y grows left.
                    pose.pose.position.y = -distance_m * math.sin(angle_rad)
                    pose.pose.position.z = 0.0

                if distance_m is not None:
                    pose.pose.orientation.w = 1.0
                    pose_pub.publish(pose)
                    rospy.loginfo_throttle(
                        1.0,
                        "[YOLO-POSE] frame=%s source=%s distance=%.2f bearing=%.2f deg point=(%.2f,%.2f)",
                        pose.header.frame_id, distance_source, distance_m,
                        angle_deg, pose.pose.position.x, pose.pose.position.y)
            else:
                rospy.logwarn_throttle(
                    5.0, "[YOLO-POSE] No fresh metric distance; %s not published.",
                    pose_topic)

            print("center x={:.1f}, horizontal angle: {:.2f} deg".format(x_center, angle_deg))
            rospy.loginfo_throttle(
                1.0,
                "[YOLO-PUB] frame=%d detections=%d angle_deg=%.2f x_center=%.1f y_center=%.1f",
                frame_count,
                len(current_detections),
                angle_deg,
                x_center,
                y_center,
            )
            rospy.loginfo_throttle(
                1.0,
                "[YOLO-PUB] area_ratio=%.4f bbox_area=%.0f image_area=%.0f",
                area_ratio, bbox_area, image_area)

        if show_window and result is not None:
            annotated = result.plot()
            cv2.imshow("Detection", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    if show_window:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
