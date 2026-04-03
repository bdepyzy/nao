#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Simple fast MJPEG viewer - 50 FPS with FPS overlay"""

import cv2
import sys
import time

ROBOT_IP = "169.254.81.31"
VIDEO_URL = "http://{}:8080/stream".format(ROBOT_IP)

def main():
    print("Connecting to {}...".format(VIDEO_URL))

    cap = cv2.VideoCapture(VIDEO_URL)
    if not cap.isOpened():
        print("ERROR: Could not connect!")
        print("Make sure robot_streamer.py is running on the robot")
        sys.exit(1)

    print("Connected! Press ESC to quit\n")

    last_time = time.time()
    frame_count = 0
    current_fps = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Lost connection...")
            break

        # Fix color: JPEG is RGB, OpenCV wants BGR
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Calculate FPS
        frame_count += 1
        if frame_count >= 10:
            current_fps = frame_count / (time.time() - last_time)
            frame_count = 0
            last_time = time.time()

        # Draw FPS on frame (yellow text, black outline for readability)
        fps_text = "FPS: {:.1f}".format(current_fps)
        cv2.putText(frame, fps_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, fps_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow("NAO Fast Stream - Color + FPS", frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    print("\nBye!")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
