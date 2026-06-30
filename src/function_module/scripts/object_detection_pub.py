#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import math

import rospy
from std_msgs.msg import Float32


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

    # Default ultralytics dir: <ws>/src/ultralytics-8.4.47
    default_ultralytics_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "ultralytics-8.4.47")
    )
    ultralytics_repo_dir = str(rospy.get_param("~ultralytics_repo_dir", default_ultralytics_dir))

    weights = str(rospy.get_param(
        "~weights",
        os.path.join(ultralytics_repo_dir, "runs", "detect", "train-2", "weights", "best.pt"),
    ))

    publish_rate_hz = float(rospy.get_param("~rate", 15.0))

    topic_angle = str(rospy.get_param("~topic_object_direction", "/object_direction"))
    topic_x = str(rospy.get_param("~topic_object_x", "/object_x"))

    # ---- Publishers ----
    angle_pub = rospy.Publisher(topic_angle, Float32, queue_size=1)
    x_pub = rospy.Publisher(topic_x, Float32, queue_size=1)

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
    rospy.loginfo("Publishing %s (deg) and %s (pixel x)", topic_angle, topic_x)

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

        # Pick the highest-confidence detection
        try:
            confs = boxes.conf.cpu().numpy()
            xyxy = boxes.xyxy.cpu().numpy()
        except Exception:
            # Fallback for potential API differences
            confs = boxes.conf
            xyxy = boxes.xyxy

        best_i = int(confs.argmax()) if hasattr(confs, "argmax") else 0
        x1, y1, x2, y2 = [float(v) for v in xyxy[best_i]]

        x_center = (x1 + x2) / 2.0

        # Convert center-x to horizontal bearing angle (deg)
        offset_ratio = (x_center - cx) / (img_w / 2.0)
        angle_rad = math.atan(offset_ratio * tan_half_hfov)
        angle_deg = math.degrees(angle_rad)

        angle_pub.publish(Float32(data=float(angle_deg)))
        x_pub.publish(Float32(data=float(x_center)))

        rate.sleep()

    cap.release()


if __name__ == "__main__":
    main()
