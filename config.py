# -*- coding: future_fstrings -*-
# config.py - Centralized configuration for the NAO robot application

# Robot Connection
ROBOT_IP = "169.254.81.31"
ROBOT_PORT = 9559

# Video Settings
VIDEO_RESOLUTION = 2  # 320x240
VIDEO_COLOR_SPACE = 11  # RGB
VIDEO_FPS = 30

# Model Paths
YOLO_MODEL = "../models/yolov8n.pt"
TFLITE_MODEL = "../models/peekaboo_model.tflite"
COCO_NAMES = "../models/coco.names"

# Training Settings
SEQUENCE_LENGTH = 10
MODEL_SAVE_DIR = "../src/movement_models"

# GUI Settings
CENTER_BOX = 150
GUI_WIDTH = 600
GUI_HEIGHT = 600

# Server Settings
PREDICTION_SERVER_URL = "http://127.0.0.1:5000/predict"
ZMQ_SERVER_IP = "172.18.0.1"
ZMQ_PUSH_PORT = 5555
ZMQ_SUB_PORT = 5556

# Image Save Directories
COVERED_DIR = "../covered"
UNCOVERED_DIR = "../uncovered"
