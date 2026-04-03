#!/usr/bin/env python
# -*- coding: utf-8 -*-
# IMU Streamer for NAO - Runs ON THE ROBOT
# Streams IMU data over TCP for SLAM

import qi
import time
import socket
import struct
import json
import sys

class ImuStreamer:
    def __init__(self):
        # Connect to local NAOqi
        self.session = qi.Session()
        self.session.connect("tcp://127.0.0.1:9559")

        # Get memory service for sensor values
        self.memory = self.session.service("ALMemory")

        # IMU keys in ALMemory
        # Accelerometer (in g's)
        self.acc_keys = [
            "Device/SubDeviceList/InertialSensor/Accelerometer/X/Sensor/Value",
            "Device/SubDeviceList/InertialSensor/Accelerometer/Y/Sensor/Value",
            "Device/SubDeviceList/InertialSensor/Accelerometer/Z/Sensor/Value",
        ]

        # Gyroscope (in deg/s)
        self.gyro_keys = [
            "Device/SubDeviceList/InertialSensor/Gyroscope/X/Sensor/Value",
            "Device/SubDeviceList/InertialSensor/Gyroscope/Y/Sensor/Value",
            "Device/SubDeviceList/InertialSensor/Gyroscope/Z/Sensor/Value",
        ]

        self.running = True
        self.start_time = time.time()

    def get_imu_data(self):
        """Read current IMU values from NAO."""
        try:
            acc = [self.memory.getData(key) for key in self.acc_keys]
            gyro = [self.memory.getData(key) for key in self.gyro_keys]

            # Calculate timestamp (seconds since start)
            timestamp = time.time() - self.start_time

            return {
                't': timestamp,
                'ax': acc[0],  # g
                'ay': acc[1],  # g
                'az': acc[2],  # g
                'gx': gyro[0],  # deg/s
                'gy': gyro[1],  # deg/s
                'gz': gyro[2],  # deg/s
            }
        except Exception as e:
            print("Error reading IMU: {}".format(e))
            return None

    def run_tcp_stream(self):
        """Run TCP stream for IMU data."""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', 8081))
        server.listen(1)

        print("IMU TCP server running on port 8081")
        print("Access from your computer:")
        print("  Connect to: tcp://169.254.81.31:8081")

        server.settimeout(1.0)

        while self.running:
            try:
                client, addr = server.accept()
                print("Client connected: {}".format(addr))
                client.settimeout(5.0)

                count = 0
                while self.running:
                    data = self.get_imu_data()
                    if data:
                        # Send as JSON line
                        line = json.dumps(data) + "\n"
                        try:
                            client.sendall(line.encode())
                            count += 1
                            if count % 100 == 0:
                                print("Sent {} IMU readings".format(count))
                        except:
                            break

                    # Aim for ~100Hz IMU rate
                    time.sleep(0.01)

            except Exception as e:
                print("Client error: {}".format(e))
                try:
                    client.close()
                except:
                    pass

    def stop(self):
        self.running = False
        print("Stopped")

if __name__ == "__main__":
    streamer = ImuStreamer()

    try:
        streamer.run_tcp_stream()
    except KeyboardInterrupt:
        print("\nStopping...")
        streamer.stop()
