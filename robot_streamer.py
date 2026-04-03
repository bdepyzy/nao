#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Fast MJPEG streamer - runs ON THE ROBOT
# Streams compressed JPEG at max speed over HTTP

import qi
import time
import socket
import threading
import struct
import sys
import os

# Try importing OpenCV for JPEG encoding
try:
    import cv2
    HAS_CV2 = True
    print("Using OpenCV for JPEG encoding")
except ImportError:
    HAS_CV2 = False
    print("OpenCV not available, using fallback")

class FastStreamer:
    def __init__(self):
        # Connect to local NAOqi
        self.session = qi.Session()
        self.session.connect("tcp://127.0.0.1:9559")
        self.video = self.session.service("ALVideoDevice")

        # Subscribe to camera - resolution selectable via command line
        # Resolution options: 0=160x120, 1=320x240, 2=640x480, 3=1280x960
        self.res_map = {
            0: (160, 120),
            1: (320, 240),
            2: (640, 480),
            3: (1280, 960)
        }

        # Get resolution from command line (default to 640x480 for SLAM)
        res_level = int(sys.argv[1]) if len(sys.argv) > 1 else 2
        self.width, self.height = self.res_map.get(res_level, (640, 480))

        self.video.setActiveCamera(0)
        self.client = self.video.subscribe(
            "fast_stream",
            res_level,  # Resolution level
            11,  # RGB
            30
        )
        self.running = True

        # Precompute JPEG header for HTTP streaming
        self.jpeg_header = '--myboundary\r\nContent-Type: image/jpeg\r\n\r\n'

    def get_jpeg(self):
        """Get a JPEG encoded frame - fastest possible."""
        try:
            result = self.video.getImageRemote(self.client)
            if not result or len(result) < 7:
                return None

            # Parse result
            width = result[0]
            height = result[1]
            data = result[6]

            # Convert to numpy for encoding
            if HAS_CV2:
                import numpy as np
                img = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
                # Encode as JPEG with quality 60 (fast compression)
                _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 60])
                return jpeg.tostring()
            else:
                # Fallback - no encoding (just return raw, slower)
                return data

        except Exception as e:
            print("Error: {}".format(e))
            return None

    def run_http_stream(self):
        """Run HTTP MJPEG stream - much faster than NAOqi!"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', 8080))
        server.listen(1)

        print("HTTP streamer running on port 8080 at {}x{}".format(self.width, self.height))
        print("Access from your computer:")
        print("  ffplay http://169.254.81.31:8080")
        print("  Or use OpenCV: cap = cv2.VideoCapture('http://169.254.81.31:8080/stream')")

        server.settimeout(1.0)

        while self.running:
            try:
                client, addr = server.accept()
                client.settimeout(5.0)

                # Send MJPEG stream header
                client.send(b"HTTP/1.0 200 OK\r\n")
                client.send(b"Server: NAO Streamer\r\n")
                client.send(b"Cache-Control: no-cache\r\n")
                client.send(b"Content-Type: multipart/x-mixed-replace;boundary=myboundary\r\n\r\n")

                # Stream frames
                frame_count = 0
                while self.running:
                    jpeg = self.get_jpeg()

                    if jpeg:
                        # Send MJPEG frame
                        client.sendall(b"--myboundary\r\n")
                        client.sendall(b"Content-Type: image/jpeg\r\n")
                        client.sendall("Content-Length: {}\r\n\r\n".format(len(jpeg)))
                        client.sendall(jpeg)
                        client.sendall(b"\r\n")
                        frame_count += 1

                        # Log every 30 frames
                        if frame_count % 30 == 0:
                            print("Streamed {} frames".format(frame_count))

            except Exception as e:
                print("Client error: {}".format(e))
                try:
                    client.close()
                except:
                    pass

    def stop(self):
        self.running = False
        self.video.unsubscribe(self.client)
        print("Stopped")

if __name__ == "__main__":
    streamer = FastStreamer()

    # Run streamer in background
    import thread
    thread.start_new_thread(streamer.run_http_stream, ())

    print("Streamer started! Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
        streamer.stop()
