#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""MJPEG streamer for the NAO top camera. Run this on the robot."""

import socket
import sys
import time

import cv2
import numpy as np
import qi


ROBOT_IP = "169.254.81.31"
STREAM_PORT = 8080


class FastStreamer(object):
    def __init__(self):
        self.session = qi.Session()
        self.session.connect("tcp://127.0.0.1:9559")
        self.video = self.session.service("ALVideoDevice")

        self.resolution = int(sys.argv[1]) if len(sys.argv) > 1 else 2
        sizes = {
            0: (160, 120),
            1: (320, 240),
            2: (640, 480),
            3: (1280, 960),
        }
        self.width, self.height = sizes.get(self.resolution, sizes[2])

        self.video.setActiveCamera(0)
        self.client = self.video.subscribe("fast_stream", self.resolution, 11, 30)
        self.running = True

    def get_jpeg(self):
        result = self.video.getImageRemote(self.client)
        if not result or len(result) < 7:
            return None

        width = result[0]
        height = result[1]
        data = result[6]
        image = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
        ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 60])
        return jpeg.tostring() if ok else None

    def run_http_stream(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", STREAM_PORT))
        server.listen(1)
        server.settimeout(1.0)

        print("HTTP streamer running on port {} at {}x{}".format(
            STREAM_PORT, self.width, self.height
        ))
        print("ffplay http://{}:{}/stream".format(ROBOT_IP, STREAM_PORT))

        while self.running:
            try:
                client, _addr = server.accept()
            except socket.timeout:
                continue

            client.settimeout(5.0)
            client.send(b"HTTP/1.0 200 OK\r\n")
            client.send(b"Server: NAO Streamer\r\n")
            client.send(b"Cache-Control: no-cache\r\n")
            client.send(b"Content-Type: multipart/x-mixed-replace;boundary=myboundary\r\n\r\n")

            try:
                while self.running:
                    jpeg = self.get_jpeg()
                    if not jpeg:
                        continue
                    client.sendall(b"--myboundary\r\n")
                    client.sendall(b"Content-Type: image/jpeg\r\n")
                    client.sendall("Content-Length: {}\r\n\r\n".format(len(jpeg)))
                    client.sendall(jpeg)
                    client.sendall(b"\r\n")
            except socket.error:
                client.close()

    def stop(self):
        self.running = False
        self.video.unsubscribe(self.client)


if __name__ == "__main__":
    streamer = FastStreamer()

    import thread
    thread.start_new_thread(streamer.run_http_stream, ())

    print("Streamer started. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        streamer.stop()
