#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Python 2 bridge from local UDP commands to NAO ALMotion."""

import json
import socket
import time

import qi

from config import ROBOT_IP, ROBOT_PORT


LISTEN = ("127.0.0.1", 8765)
STALE_AFTER_S = 0.35


def clamp(value, low, high):
    return max(low, min(high, float(value)))


session = qi.Session()
session.connect("tcp://{}:{}".format(ROBOT_IP, ROBOT_PORT))
motion = session.service("ALMotion")
motion.wakeUp()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(LISTEN)
sock.settimeout(0.05)


def send(x, y, theta, frequency):
    motion.setWalkTargetVelocity(
        clamp(x, -1.0, 1.0),
        clamp(y, -1.0, 1.0),
        clamp(theta, -1.0, 1.0),
        clamp(frequency, 0.0, 1.0),
    )


print("Motion bridge listening on udp://{}:{}".format(*LISTEN))
last_command_at = 0.0

try:
    while True:
        try:
            data, _addr = sock.recvfrom(4096)
        except socket.timeout:
            if last_command_at and time.time() - last_command_at > STALE_AFTER_S:
                send(0.0, 0.0, 0.0, 0.0)
                last_command_at = 0.0
            continue

        command = json.loads(data)
        x = command.get("x", 0.0)
        y = command.get("y", 0.0)
        theta = command.get("theta", 0.0)
        frequency = command.get("frequency", 0.6)
        if not x and not y and not theta:
            frequency = 0.0
        send(x, y, theta, frequency)
        last_command_at = time.time()
except KeyboardInterrupt:
    pass
finally:
    send(0.0, 0.0, 0.0, 0.0)
