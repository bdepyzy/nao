#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""NAO Robot Controller with Video Stream

Control the robot with WASD keys while viewing the camera feed.

Usage:
  python slam_controller.py

Movement:
  W/S: forward/backward
  A/D: strafe left/right
  Q/E: turn left/right
  SPACE: stop
  ESC: quit
"""

import qi
import cv2
import time
import sys
import numpy as np
from config import ROBOT_IP, ROBOT_PORT, VIDEO_URL


class SLAMController:
    """NAO controller with video feed"""

    def __init__(self):
        self.cap = None

        try:
            # Connect to NAOqi
            self.session = qi.Session()
            self.session.connect("tcp://{}:{}".format(ROBOT_IP, ROBOT_PORT))
            self.motion = self.session.service("ALMotion")
            self.motion.wakeUp()

            # Connect to video stream
            print("Connecting to video stream at {}...".format(VIDEO_URL))
            self.cap = cv2.VideoCapture(VIDEO_URL)
            if not self.cap.isOpened():
                print("WARNING: Could not open video stream!")
                print("Make sure robot_streamer.py is running on the NAO")
            else:
                print("Video stream connected!")

            # Track held keys
            self.held_keys = {}
            self.last_update = time.time()

            print("\n=== NAO Controller ===")
            print("Movement: W/S (fwd/back), A/D (strafe), Q/E (turn), SPACE stop")
            print("Press ESC to quit\n")

        except Exception as e:
            print("ERROR: Cannot connect to NAO: {}".format(e))
            sys.exit(1)

    def update_movement(self):
        """Check held keys and send movement"""
        now = time.time()

        # Clear keys that haven't been pressed in 150ms
        timeout = 0.15
        self.held_keys = {k: t for k, t in self.held_keys.items() if now - t < timeout}

        # Movement keys
        move_keys = {"w", "s", "a", "d", "q", "e"}
        held_move = set(self.held_keys.keys()) & move_keys

        x = y = theta = 0.0
        if "w" in held_move:
            x = 0.5
        elif "s" in held_move:
            x = -0.5
        if "a" in held_move:
            y = 0.5
        elif "d" in held_move:
            y = -0.5
        if "q" in held_move:
            theta = 0.5
        elif "e" in held_move:
            theta = -0.5

        # Send movement
        self.motion.setWalkTargetVelocity(x, y * 0.7, theta, 0)

        return x, y, theta

    def handle_key(self, key):
        """Process keyboard input"""
        if key == 27:  # ESC
            return False

        k = chr(key).lower()

        # Track movement keys
        move_keys = {"w", "s", "a", "d", "q", "e"}
        if k in move_keys:
            self.held_keys[k] = time.time()

        return True

    def run(self):
        """Main loop with video and controls"""
        print("Starting controller...")

        while True:
            # Get video frame
            if self.cap and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    # Convert RGB to BGR for OpenCV display
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                    # Add overlay text
                    x, y, theta = self.update_movement()

                    status = "STOPPED"
                    if x != 0 or y != 0 or theta != 0:
                        status = "MOVING: x={:.1f} y={:.1f} turn={:.1f}".format(
                            x, y, theta
                        )

                    # Draw status on frame
                    cv2.putText(
                        frame,
                        status,
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )
                    cv2.putText(
                        frame,
                        "WASD:move Q/E:turn SPACE:stop ESC:quit",
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                    )

                    cv2.imshow("NAO Camera + Control", frame)
                else:
                    # No frame - just update movement
                    self.update_movement()
                    # Create blank frame
                    blank = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(
                        blank,
                        "No video - check streamer",
                        (50, 240),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 0, 255),
                        2,
                    )
                    cv2.imshow("NAO Camera + Control", blank)
            else:
                # No video connection
                self.update_movement()
                blank = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(
                    blank,
                    "No video stream",
                    (50, 240),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2,
                )
                cv2.imshow("NAO Camera + Control", blank)

            # Handle keys
            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                if not self.handle_key(key):
                    break

        # Cleanup
        print("\nStopping robot...")
        self.motion.setWalkTargetVelocity(0, 0, 0, 0)
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()


def main():
    SLAMController().run()


if __name__ == "__main__":
    main()
