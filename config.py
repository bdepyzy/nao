# NAO Robot Configuration

# Robot IP - change this to your NAO's IP
ROBOT_IP = "169.254.81.31"

# NAOqi port (default: 9559)
ROBOT_PORT = 9559

# Video stream URL (auto-generated from ROBOT_IP)
VIDEO_URL = "http://{}:8080/stream".format(ROBOT_IP)
