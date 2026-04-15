#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Auto-launcher for NAO controller with SSH management.

Starts robot_streamer.py on the NAO via SSH, runs the controller locally,
and cleans up the remote process on exit.
"""

import subprocess
import time
import signal
import sys
import os

# Import config
from config import ROBOT_IP, SSH_KEY, SSH_USER, REMOTE_STREAMER_CMD


class SSHManager:
    def __init__(self):
        self.remote_pid = None

    def start_remote_streamer(self):
        """SSH into robot and start streamer in background."""
        print("[SSH] Connecting to {}@{}...".format(SSH_USER, ROBOT_IP))

        # Start streamer on robot - run it in background and capture PID
        ssh_cmd = [
            "ssh",
            "-i",
            SSH_KEY,
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "UserKnownHostsFile=/dev/null",
            "-o",
            "LogLevel=ERROR",
            "{}@{}".format(SSH_USER, ROBOT_IP),
            "cd ~ && nohup python robot_streamer.py > /dev/null 2>&1 & sleep 1 && pgrep -f 'python.*robot_streamer' | tail -1",
        ]

        try:
            result = subprocess.check_output(ssh_cmd, stderr=subprocess.STDOUT)
            self.remote_pid = result.strip()
            if self.remote_pid:
                print(
                    "[SSH] Streamer started on robot (PID: {})".format(self.remote_pid)
                )
            else:
                print("[SSH] Streamer started (PID unknown)")

            # Give streamer time to initialize
            print("[SSH] Waiting for streamer to initialize...")
            time.sleep(3)
            return True

        except subprocess.CalledProcessError as e:
            print("[SSH] Failed to start streamer: {}".format(e))
            return False

    def stop_remote_streamer(self):
        """Kill the remote streamer process."""
        print("[SSH] Stopping remote streamer...")

        # Kill by PID if we have it, or by pattern match
        kill_cmd = "kill {} 2>/dev/null; pkill -9 -f robot_streamer 2>/dev/null; echo done".format(
            self.remote_pid if self.remote_pid else "-1"
        )

        ssh_cmd = [
            "ssh",
            "-i",
            SSH_KEY,
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "UserKnownHostsFile=/dev/null",
            "-o",
            "LogLevel=ERROR",
            "{}@{}".format(SSH_USER, ROBOT_IP),
            kill_cmd,
        ]
        try:
            subprocess.call(ssh_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            print("[SSH] Remote streamer stopped.")
        except Exception as e:
            print("[SSH] Warning: couldn't stop streamer: {}".format(e))


def main():
    ssh_manager = SSHManager()

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\n[Launcher] Caught signal, shutting down...")
        ssh_manager.stop_remote_streamer()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Start remote streamer
    if not ssh_manager.start_remote_streamer():
        print("[Launcher] Failed to start remote streamer. Exiting.")
        sys.exit(1)

    # Import and run the actual controller
    print("[Launcher] Starting controller...")
    print("-" * 50)

    try:
        # Import slam_controller and run it
        import slam_controller

        controller = slam_controller.SLAMController()
        controller.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("[Launcher] Controller error: {}".format(e))
    finally:
        # Always stop remote streamer on exit
        print("-" * 50)
        ssh_manager.stop_remote_streamer()
        print("[Launcher] Done.")


if __name__ == "__main__":
    main()
