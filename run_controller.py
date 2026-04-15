#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Simple launcher for NAO controller.

Just runs slam_controller.py - assumes you already started robot_streamer.py on the NAO.
"""

import sys
import signal

# Import and run the controller
try:
    import slam_controller

    controller = slam_controller.SLAMController()
    controller.run()
except KeyboardInterrupt:
    pass
except Exception as e:
    print("Error: {}".format(e))
    sys.exit(1)
