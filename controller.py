# -*- coding: future_fstrings -*-
"""NAO Robot Controller with FAST HTTP Video Stream

Movement:
  W/S: forward/backward
  A/D: strafe left/right
  Q/E: turn left/right
  I/K: head up/down
  J/L: head left/right

Hands (COMBAT):
  R: close right hand  | T: open right hand
  F: close left hand   | G: open left hand
  1: GUARD UP (arms ready)
  2: FIGHT STANCE (jab)
  3: ARMS FORWARD
  0: Neutral (arms down)

Direct arm control:
  U/J: Left arm up/down
  I/K: Right arm up/down

ESC: quit

Video: 300 FPS HTTP MJPEG stream!
"""

import qi
import cv2
import numpy as np
import threading
import sys
import time

# Config
ROBOT_IP = "169.254.81.31"
ROBOT_PORT = 9559
VIDEO_URL = "http://{}:8080/stream".format(ROBOT_IP)


class FastNAOController:
    """Fast NAO controller with HTTP video stream"""

    def __init__(self):
        # Video from HTTP stream (50 FPS!)
        self.cap = cv2.VideoCapture(VIDEO_URL)
        if not self.cap.isOpened():
            print("ERROR: Cannot connect to video stream!")
            print("Make sure robot_streamer.py is running on the robot")
            sys.exit(1)

        self.latest_frame = None
        self.video_thread = threading.Thread(target=self._video_loop)
        self.video_thread.daemon = True
        self.video_thread.start()

        # Motion control via NAOqi
        self.session = qi.Session()
        self.session.connect("tcp://{}:{}".format(ROBOT_IP, ROBOT_PORT))
        self.motion = self.session.service("ALMotion")
        self.posture = self.session.service("ALRobotPosture")

        self.motion.wakeUp()
        self.posture.goToPosture("Crouch", 0.5)

        # Head position state
        self.h_yaw = 0.0
        self.h_pitch = 0.0
        self.h_speed = 0.1

        # Key state - track what's held down
        self.held_keys = {}
        self.last_key_time = 0

        # Hand state
        self.left_hand_closed = False
        self.right_hand_closed = False

        # FPS tracking
        self.last_time = time.time()
        self.frame_count = 0
        self.current_fps = 0.0

    def _video_loop(self):
        """Background thread - keeps video flowing"""
        while True:
            ret, frame = self.cap.read()
            if ret:
                # Fix color: JPEG is RGB, OpenCV wants BGR
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                # Resize for larger GUI
                frame = cv2.resize(frame, (640, 480))
                self.latest_frame = frame

    def update_movement(self):
        """Check held keys and send movement - stops instantly when nothing pressed"""
        now = time.time()

        # Clear keys that haven't been pressed in 100ms (key released)
        timeout = 0.1
        self.held_keys = {k: t for k, t in self.held_keys.items() if now - t < timeout}

        # Movement keys
        move_keys = {'w', 's', 'a', 'd', 'q', 'e'}
        held_move = set(self.held_keys.keys()) & move_keys

        x = y = t = 0.0
        if 'w' in held_move: x = 0.5
        elif 's' in held_move: x = -0.5
        if 'a' in held_move: y = 0.5
        elif 'd' in held_move: y = -0.5
        if 'q' in held_move: t = 0.5
        elif 'e' in held_move: t = -0.5

        # Send movement (0,0,0,0 when nothing held = instant stop!)
        self.motion.setWalkTargetVelocity(x, y * 0.7, t, 0)

    def handle_input(self, key):
        """Process keyboard input - return False to quit"""
        if key == 27:  # ESC
            return False

        k = chr(key).lower()

        # Track movement keys with timestamp (for release detection)
        move_keys = {'w', 's', 'a', 'd', 'q', 'e'}
        if k in move_keys:
            self.held_keys[k] = time.time()

        # Head (one-shot movement)
        if k == 'i':
            self.h_pitch = max(self.h_pitch - self.h_speed, -0.6)
        elif k == 'k':
            self.h_pitch = min(self.h_pitch + self.h_speed, 0.6)
        elif k == 'j':
            self.h_yaw = min(self.h_yaw + self.h_speed, 2.0)
        elif k == 'l':
            self.h_yaw = max(self.h_yaw - self.h_speed, -2.0)
        # Direct arm control (shoulder pitch - arms up/down)
        elif k == 'u':  # Left arm up
            try:
                current = self.motion.getAngles("LShoulderPitch", True)[0]
                self.motion.setAngles("LShoulderPitch", max(current - 0.2, -0.5), 0.1)
                print("Left arm up")
            except: pass
        elif k == 'j':  # Left arm down (wait, j is head... let's use different keys)
            pass  # j is used for head, skip
        elif k == 'o':  # Right arm up
            try:
                current = self.motion.getAngles("RShoulderPitch", True)[0]
                self.motion.setAngles("RShoulderPitch", max(current - 0.2, -0.5), 0.1)
                print("Right arm up")
            except: pass
        elif k == 'l':  # Right arm down
            pass  # l is used for head, skip

        self.motion.setAngles(["HeadYaw", "HeadPitch"], [self.h_yaw, self.h_pitch], 0.3)

        # Hand controls
        if k == 'r':  # Close right hand
            print("Closing right hand...")
            try:
                self.motion.setAngles("RHand", 1.0, 0.5)
                self.right_hand_closed = True
                print("Right hand CLOSED")
            except Exception as e:
                print("Error closing right hand: {}".format(e))
        elif k == 't':  # Open right hand
            print("Opening right hand...")
            try:
                self.motion.setAngles("RHand", 0.0, 0.5)
                self.right_hand_closed = False
                print("Right hand OPEN")
            except Exception as e:
                print("Error opening right hand: {}".format(e))
        elif k == 'f':  # Close left hand
            print("Closing left hand...")
            try:
                self.motion.setAngles("LHand", 1.0, 0.5)
                self.left_hand_closed = True
                print("Left hand CLOSED")
            except Exception as e:
                print("Error closing left hand: {}".format(e))
        elif k == 'g':  # Open left hand
            print("Opening left hand...")
            try:
                self.motion.setAngles("LHand", 0.0, 0.5)
                self.left_hand_closed = False
                print("Left hand OPEN")
            except Exception as e:
                print("Error opening left hand: {}".format(e))

        # Combat poses
        elif k == '1':  # Guard up
            print("GUARD UP!")
            self._guard_pose()
        elif k == '2':  # Fight stance
            print("FIGHT STANCE!")
            self._fight_stance()
        elif k == '3':  # Arms forward
            print("ARMS FORWARD!")
            self._arms_forward()
        elif k == '0':  # Neutral
            print("Neutral pose")
            self._neutral_pose()

        # Direct arm control (using keys that don't conflict)
        elif k == 'z':  # Left arm up
            self._move_arm("LShoulderPitch", -0.2)
        elif k == 'x':  # Left arm down
            self._move_arm("LShoulderPitch", 0.2)
        elif k == 'c':  # Right arm up
            self._move_arm("RShoulderPitch", -0.2)
        elif k == 'v':  # Right arm down
            self._move_arm("RShoulderPitch", 0.2)
        elif k == 'b':  # Both arms up
            self._move_arm("LShoulderPitch", -0.2)
            self._move_arm("RShoulderPitch", -0.2)
        elif k == 'n':  # Both arms down
            self._move_arm("LShoulderPitch", 0.2)
            self._move_arm("RShoulderPitch", 0.2)

        return True

    def _guard_pose(self):
        """Guard up - arms raised like boxing guard"""
        try:
            # Arms up in front of face, elbows tucked in
            # ShoulderPitch: arms forward, ShoulderRoll: arms out to sides
            # ElbowRoll: bend elbow (positive), ElbowYaw: rotation
            self.motion.setAngles(["LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw",
                                   "RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw"],
                                  [1.3, 0.5, 1.4, -1.5,   # Left arm - up and in
                                   1.3, -0.5, -1.4, 1.5],  # Right arm - up and in
                                  0.2)
            # Close fists
            self.motion.setAngles(["LHand", "RHand"], [1.0, 1.0], 0.2)
            print("Guard pose set!")
        except Exception as e:
            print("Error in guard pose: {}".format(e))

    def _fight_stance(self):
        """Fight stance - left hand forward jab, right guarding"""
        try:
            # Left arm extended forward, right arm guarding face
            self.motion.setAngles(["LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw",
                                   "RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw"],
                                  [1.0, 0.3, 0.2, -1.5,   # Left arm - forward jab
                                   1.3, -0.5, -1.2, 1.0],  # Right arm - guarding
                                  0.2)
            self.motion.setAngles(["LHand", "RHand"], [1.0, 1.0], 0.2)
            print("Fight stance set!")
        except Exception as e:
            print("Error in fight stance: {}".format(e))

    def _neutral_pose(self):
        """Neutral - arms relaxed at sides"""
        try:
            self.motion.setAngles(["LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw",
                                   "RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw"],
                                  [1.5, 0.1, -0.1, 0.0,   # Left arm down
                                   1.5, -0.1, 0.1, 0.0],   # Right arm down
                                  0.2)
            self.motion.setAngles(["LHand", "RHand"], [0.0, 0.0], 0.2)
            print("Neutral pose set!")
        except Exception as e:
            print("Error in neutral pose: {}".format(e))

    def _arms_forward(self):
        """Both arms extended forward"""
        try:
            self.motion.setAngles(["LShoulderPitch", "RShoulderPitch"],
                                  [0.5, 0.5], 0.2)
            self.motion.setAngles(["LShoulderRoll", "RShoulderRoll"],
                                  [0.0, 0.0], 0.2)
            print("Arms forward!")
        except Exception as e:
            print("Error: {}".format(e))

    def _move_arm(self, joint, delta):
        """Move a single joint by delta"""
        try:
            current = self.motion.getAngles(joint, True)[0]
            new_angle = max(-1.5, min(1.5, current + delta))
            self.motion.setAngles(joint, new_angle, 0.1)
            print("{}: {} -> {}".format(joint, current, new_angle))
        except Exception as e:
            print("Error moving {}: {}".format(joint, e))

    def run(self):
        """Main loop"""
        print("\n=== Fast NAO Controller ===")
        print("Video: HTTP stream (300 FPS) - 640x480")
        print("\nMovement: W/S/A/D/Q/E | Head: I/J/K/L")
        print("\nCOMBAT CONTROLS:")
        print("  R/T: Right hand close/open")
        print("  F/G: Left hand close/open")
        print("  1: GUARD UP")
        print("  2: FIGHT STANCE")
        print("  3: ARMS FORWARD")
        print("  0: Neutral")
        print("\nDirect arm control:")
        print("  Z/X: Left arm up/down")
        print("  C/V: Right arm up/down")
        print("  B/N: Both arms up/down")
        print("  ESC: quit\n")

        cv2.namedWindow("NAO - Fast Stream", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("NAO - Fast Stream", 640, 480)

        while True:
            # Update movement (every frame - stops instantly when no keys held)
            self.update_movement()

            # Non-blocking key check
            key = cv2.waitKey(1) & 0xFF
            if key != 255:  # Key pressed
                if not self.handle_input(key):
                    break

            # Display frame with FPS
            if self.latest_frame is not None:
                # Calculate FPS
                self.frame_count += 1
                if self.frame_count >= 10:
                    self.current_fps = self.frame_count / (time.time() - self.last_time)
                    self.frame_count = 0
                    self.last_time = time.time()

                # Draw FPS
                frame = self.latest_frame.copy()
                fps_text = "FPS: {:.1f}".format(self.current_fps)
                cv2.putText(frame, fps_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(frame, fps_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)

                cv2.imshow("NAO - Fast Stream", frame)

        # Cleanup
        print("\nStopping...")
        self.motion.setWalkTargetVelocity(0, 0, 0, 0)
        self.cap.release()
        cv2.destroyAllWindows()


def main():
    FastNAOController().run()


if __name__ == "__main__":
    main()
