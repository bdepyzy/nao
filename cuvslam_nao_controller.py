#!/usr/bin/env python3
"""Python 3 cuVSLAM viewer and NAO keyboard controller."""

import json
import socket
import time

import cv2
import cuvslam
import numpy as np

from config import VIDEO_URL


BRIDGE = ("127.0.0.1", 8765)
HFOV_DEG = 60.9
VFOV_DEG = 47.6
FORWARD = 0.45
STRAFE = 0.30
TURN = 0.45
FREQUENCY = 0.60


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_motion(x, y, theta):
    frequency = 0.0 if x == 0.0 and y == 0.0 and theta == 0.0 else FREQUENCY
    sock.sendto(
        json.dumps({"x": x, "y": y, "theta": theta, "frequency": frequency}).encode("utf-8"),
        BRIDGE,
    )


def make_tracker(width, height):
    fx = width / (2.0 * np.tan(np.deg2rad(HFOV_DEG) / 2.0))
    fy = height / (2.0 * np.tan(np.deg2rad(VFOV_DEG) / 2.0))

    camera = cuvslam.Camera(
        size=[width, height],
        principal=[width / 2.0, height / 2.0],
        focal=[fx, fy],
        rig_from_camera=cuvslam.Pose([0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0]),
        distortion=cuvslam.Distortion(cuvslam.Distortion.Pinhole),
    )
    rig = cuvslam.Rig([camera])

    odom = cuvslam.Tracker.OdometryConfig()
    odom.odometry_mode = cuvslam.Tracker.OdometryMode.Mono
    odom.use_gpu = True

    slam = cuvslam.Tracker.SlamConfig()
    slam.use_gpu = True
    slam.sync_mode = False

    print("cuVSLAM intrinsics: fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}".format(
        fx, fy, width / 2.0, height / 2.0
    ))
    return cuvslam.Tracker(rig, odom, slam)


def motion_from_keys(keys):
    now = time.time()
    keys = {k: t for k, t in keys.items() if now - t < 0.15}

    x = FORWARD if "w" in keys else -FORWARD if "s" in keys else 0.0
    y = STRAFE if "a" in keys else -STRAFE if "d" in keys else 0.0
    theta = TURN if "q" in keys else -TURN if "e" in keys else 0.0
    return keys, x, y, theta


def pose_text(odom_pose, slam_pose):
    if odom_pose.world_from_rig is None:
        return "odom: no pose", ""

    t = odom_pose.world_from_rig.pose.translation
    odom = "odom x={:.2f} y={:.2f} z={:.2f}".format(t[0], t[1], t[2])

    slam = ""
    if slam_pose is not None:
        t = slam_pose.translation
        slam = "slam x={:.2f} y={:.2f} z={:.2f}".format(t[0], t[1], t[2])
    return odom, slam


def draw(frame, motion, odom, slam):
    x, y, theta = motion
    status = "STOPPED" if (x, y, theta) == (0.0, 0.0, 0.0) else (
        "MOVE x={:.1f} y={:.1f} turn={:.1f}".format(x, y, theta)
    )
    lines = [status, odom, slam, "W/S A/D Q/E move  SPACE stop  ESC quit"]

    y0 = 28
    for line in [line for line in lines if line]:
        cv2.putText(frame, line, (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        y0 += 28


def main():
    cuvslam.set_verbosity(1)

    cap = cv2.VideoCapture(VIDEO_URL)
    print("Video:", VIDEO_URL)

    tracker = None
    keys = {}

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.02)
                continue

            if tracker is None:
                h, w = frame.shape[:2]
                tracker = make_tracker(w, h)

            keys, x, y, theta = motion_from_keys(keys)
            send_motion(x, y, theta)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            odom_pose, slam_pose = tracker.track(time.time_ns(), [np.ascontiguousarray(gray)])
            odom, slam = pose_text(odom_pose, slam_pose)

            draw(frame, (x, y, theta), odom, slam)
            cv2.imshow("NAO cuVSLAM", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            if key == 32:
                keys.clear()
                send_motion(0.0, 0.0, 0.0)
            elif 0 <= key < 256 and chr(key).lower() in "wasdqe":
                keys[chr(key).lower()] = time.time()
    finally:
        send_motion(0.0, 0.0, 0.0)
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
