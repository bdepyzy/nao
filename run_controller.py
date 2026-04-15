#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Auto-launcher for NAO controller with automatic SSH password.

Uses sshpass for automatic password auth, starts robot_streamer.py on NAO,
runs the controller locally, cleans up on exit.
"""

import subprocess
import time
import signal
import sys
import os

# Import config
from config import ROBOT_IP, SSH_USER, SSH_PASS, REMOTE_STREAMER_CMD


class SSHManager:
    def __init__(self):
        self.remote_pid = None
        self.we_started_it = False

    def build_ssh_cmd(self, remote_cmd):
        """Build SSH command with automatic password."""
        if SSH_PASS:
            # Use sshpass for automatic password
            cmd = [
                "sshpass",
                "-p",
                SSH_PASS,
                "ssh",
                "-x",  # Disable X11 forwarding
                "-o",
                "StrictHostKeyChecking=no",
                "-o",
                "UserKnownHostsFile=/dev/null",
                "-o",
                "LogLevel=ERROR",
                "-o",
                "ConnectTimeout=5",
                "{}@{}".format(SSH_USER, ROBOT_IP),
                remote_cmd,
            ]
        else:
            # Use key-based auth
            cmd = [
                "ssh",
                "-x",  # Disable X11 forwarding
                "-i",
                "nao-ssh",
                "-o",
                "StrictHostKeyChecking=no",
                "-o",
                "UserKnownHostsFile=/dev/null",
                "-o",
                "LogLevel=ERROR",
                "-o",
                "ConnectTimeout=5",
                "{}@{}".format(SSH_USER, ROBOT_IP),
                remote_cmd,
            ]
        return cmd

    def start_remote_streamer(self):
        """SSH into robot and start streamer in background."""
        print("[SSH] Connecting to {}@{}...".format(SSH_USER, ROBOT_IP))

        # First check if streamer is already running
        check_cmd = self.build_ssh_cmd("pgrep -f 'python.*robot_streamer' | head -1")
        try:
            result = subprocess.check_output(check_cmd, stderr=subprocess.STDOUT)
            if result.strip():
                self.remote_pid = result.strip()
                print(
                    "[SSH] Streamer already running (PID: {})".format(self.remote_pid)
                )
                self.we_started_it = False
                time.sleep(1)
                return True
        except:
            pass

        # Start streamer on robot
        print("[SSH] Starting robot_streamer.py...")
        remote_cmd = "nohup python /home/nao/robot_streamer.py > /dev/null 2>&1 &"
        ssh_cmd = self.build_ssh_cmd(remote_cmd)

        try:
            subprocess.call(ssh_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            print("[SSH] Streamer started")
            self.we_started_it = True

            # Give streamer time to initialize
            print("[SSH] Waiting 3 seconds for streamer to initialize...")
            time.sleep(3)

            # Try to get PID
            pid_cmd = self.build_ssh_cmd("pgrep -f 'python.*robot_streamer' | head -1")
            try:
                result = subprocess.check_output(pid_cmd, stderr=subprocess.STDOUT)
                self.remote_pid = result.strip()
                print("[SSH] Streamer PID: {}".format(self.remote_pid))
            except:
                pass

            return True

        except subprocess.CalledProcessError as e:
            print("[SSH] Failed to start streamer: {}".format(e))
            print("[SSH] Make sure sshpass is installed: sudo apt-get install sshpass")
            return False
        except Exception as e:
            print("[SSH] Unexpected error: {}".format(e))
            return False
        except Exception as e:
            print("[SSH] Unexpected error: {}".format(e))
            return False
        except subprocess.CalledProcessError as e:
            print("[SSH] Failed to start streamer: {}".format(e))
            print("[SSH] Make sure sshpass is installed: sudo apt-get install sshpass")
            return False
        except Exception as e:
            print("[SSH] Unexpected error: {}".format(e))
            return False

    def stop_remote_streamer(self):
        """Kill the remote streamer process."""
        if not self.we_started_it:
            print("[SSH] Streamer was already running, leaving it alone.")
            return

        print("[SSH] Stopping remote streamer...")

        # Kill by PID if we have it, or by pattern match
        kill_cmd = "kill {} 2>/dev/null; pkill -9 -f robot_streamer 2>/dev/null; echo done".format(
            self.remote_pid if self.remote_pid else "-1"
        )
        ssh_cmd = self.build_ssh_cmd(kill_cmd)

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
