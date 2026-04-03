#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""NAO Robot Controller for SLAM

Run this alongside SLAM to control the robot with WASDQE keys.

Usage:
  Terminal 1: ./run_slam.sh mono
  Terminal 2: python slam_controller.py

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

ROBOT_IP = "169.254.81.31"
ROBOT_PORT = 9559


class SLAMController:
    """Minimal NAO controller for SLAM"""

    def __init__(self):
        try:
            self.session = qi.Session()
            self.session.connect("tcp://{}:{}".format(ROBOT_IP, ROBOT_PORT))
            self.motion = self.session.service("ALMotion")

            # Wake up and ready for walking
            self.motion.wakeUp()

            # Track held keys with timestamps
            self.held_keys = {}
            self.last_update = time.time()

            print("Connected to NAO at {}".format(ROBOT_IP))
            print("\n=== SLAM Controller ===")
            print("Movement: W/S (fwd/back), A/D (strafe), Q/E (turn), SPACE stop")
            print("Press ESC to quit\n")

        except Exception as e:
            print("ERROR: Cannot connect to NAO: {}".format(e))
            sys.exit(1)

    def update_movement(self):
        """Check held keys and send movement - stops instantly when nothing pressed"""
        now = time.time()

        # Clear keys that haven't been pressed in 150ms
        timeout = 0.15
        self.held_keys = {k: t for k, t in self.held_keys.items() if now - t < timeout}

        # Movement keys
        move_keys = {'w', 's', 'a', 'd', 'q', 'e'}
        held_move = set(self.held_keys.keys()) & move_keys

        x = y = theta = 0.0
        if 'w' in held_move: x = 0.5
        elif 's' in held_move: x = -0.5
        if 'a' in held_move: y = 0.5
        elif 'd' in held_move: y = -0.5
        if 'q' in held_move: theta = 0.5
        elif 'e' in held_move: theta = -0.5

        # Send movement (0,0,0 when nothing held = instant stop)
        self.motion.setWalkTargetVelocity(x, y * 0.7, theta, 0)

        return x, y, theta

    def handle_key(self, key):
        """Process keyboard input"""
        if key == 27:  # ESC
            return False

        k = chr(key).lower()

        # Track movement keys
        move_keys = {'w', 's', 'a', 'd', 'q', 'e'}
        if k in move_keys:
            self.held_keys[k] = time.time()

        return True

    def run(self):
        """Main loop - runs alongside SLAM viewer"""
        blank = 255 * np.ones((100, 400, 3), dtype=np.uint8)

        while True:
            # Update movement
            x, y, theta = self.update_movement()

            # Non-blocking key check
            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                if not self.handle_key(key):
                    break

            # Update display
            display = blank.copy()
            status = "STOPPED"
            if x != 0 or y != 0 or theta != 0:
                status = "MOVING: x={:.1f} y={:.1f} th={:.1f}".format(x, y, theta)

            cv2.putText(display, status, (10, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv2.putText(display, "WASD move | Q/E turn | SPACE stop | ESC quit",
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
            cv2.imshow("NAO Controller", display)

        # Cleanup
        print("\nStopping robot...")
        self.motion.setWalkTargetVelocity(0, 0, 0, 0)
        cv2.destroyAllWindows()


def main():
    SLAMController().run()


if __name__ == "__main__":
    main()
