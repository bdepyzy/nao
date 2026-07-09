#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Python 2 NAO camera viewer and keyboard controller."""

import time

import cv2
import numpy as np
import qi

from config import ROBOT_IP, ROBOT_PORT, VIDEO_URL


FORWARD = 0.5
STRAFE = 0.35
TURN = 0.5
KEY_TIMEOUT_S = 0.15


class Controller(object):
    def __init__(self):
        session = qi.Session()
        session.connect("tcp://{}:{}".format(ROBOT_IP, ROBOT_PORT))
        self.motion = session.service("ALMotion")
        self.motion.wakeUp()

        print("Video:", VIDEO_URL)
        self.cap = cv2.VideoCapture(VIDEO_URL)
        self.keys = {}

    def command(self):
        now = time.time()
        self.keys = {key: at for key, at in self.keys.items() if now - at < KEY_TIMEOUT_S}

        x = FORWARD if "w" in self.keys else -FORWARD if "s" in self.keys else 0.0
        y = STRAFE if "a" in self.keys else -STRAFE if "d" in self.keys else 0.0
        theta = TURN if "q" in self.keys else -TURN if "e" in self.keys else 0.0
        self.motion.setWalkTargetVelocity(x, y, theta, 0.6 if x or y or theta else 0.0)
        return x, y, theta

    def handle_key(self, key):
        if key == 27:
            return False
        if key == 32:
            self.keys.clear()
            self.motion.setWalkTargetVelocity(0.0, 0.0, 0.0, 0.0)
            return True
        if 0 <= key < 256 and chr(key).lower() in "wasdqe":
            self.keys[chr(key).lower()] = time.time()
        return True

    def frame(self):
        if self.cap.isOpened():
            ok, frame = self.cap.read()
            if ok:
                return frame

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(frame, "No video stream", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        return frame

    def run(self):
        while True:
            frame = self.frame()
            x, y, theta = self.command()
            status = "STOPPED" if not (x or y or theta) else "MOVE x={:.1f} y={:.1f} turn={:.1f}".format(x, y, theta)

            cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, "W/S A/D Q/E move  SPACE stop  ESC quit", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("NAO Control", frame)

            key = cv2.waitKey(1) & 0xFF
            if key != 255 and not self.handle_key(key):
                break

        self.motion.setWalkTargetVelocity(0.0, 0.0, 0.0, 0.0)
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    Controller().run()
