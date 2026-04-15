# NAO Robot Control

Control NAO robots with keyboard. Two controllers available:

- `slam_controller.py` - Movement only (WASD + QE). Use this one.
- `controller.py` - Movement + video stream. Broken half the time.

## Setup

Edit `config.py` and set your robot's IP:

```python
ROBOT_IP = "169.254.81.31"  # Your robot's IP
```

## Run

### 1. SSH into the robot and start the video streamer:

```bash
ssh -i nao-ssh nao@169.254.81.31
python robot_streamer.py
```

Keep this running in a terminal.

### 2. Run the controller:

```bash
python2 slam_controller.py
```

A window pops up. **Click on it to focus**, then use keys:

- **W/S** - Forward/Back
- **A/D** - Strafe left/right  
- **Q/E** - Turn left/right
- **SPACE** - Stop
- **ESC** - Quit

The robot only moves while you're holding the key down.

## Troubleshooting

**"Cannot connect"** - Make sure `robot_streamer.py` is running on the NAO.

**"qi module not found"** - Use `python2`, not `python3`. NAOqi only works with Python 2.7.

**Robot doesn't move** - Click the controller window first to focus it. The window must be active.

**Connection refused** - Wrong IP. Edit `config.py` and ping the robot to verify.

## Which Robot is Which

config.py has comments:
- `169.254.81.31` - Super load blue robot (SSH works)
- `169.254.66.118` - Blue robot with dead battery sticker (SSH broken, needs password)

Switch IPs in config.py to change robots.

## File Reference

### Python Files

- **`slam_controller.py`** - Main controller. WASD movement, connects to NAOqi.
- **`controller.py`** - Alternate controller with video preview. Kinda janky.
- **`robot_streamer.py`** - Runs ON the robot. HTTP video server at 50 FPS.
- **`imu_streamer.py`** - Runs ON the robot. Streams IMU data for inertial SLAM.
- **`view_fast.py`** - Just the video viewer. See what robot sees without controlling.
- **`config.py`** - Robot IP and other settings.

### C++ Files (SLAM)

- **`mono_nao.cc`** - Basic monocular SLAM. Camera only, outputs trajectory.
- **`mono_inertial_nao.cc`** - SLAM with IMU fusion. More accurate.
- **`mono_nao_control.cc`** - SLAM with built-in WASD control.

### Scripts

- **`run_slam.sh`** - Wrapper for C++ binaries:
  - `./run_slam.sh mono` - Basic SLAM
  - `./run_slam.sh inertial` - SLAM + IMU
  - `./run_slam.sh control` - SLAM + keyboard control

### Config

- **`NAO_640x480.yaml`** - Camera calibration for 640x480
- **`NAO_1280x960.yaml`** - Camera calibration for 1280x960

## Quick Start

**Just drive the robot:**
```bash
python2 slam_controller.py
```

**SLAM + drive:**
```bash
# Terminal 1
./run_slam.sh control

# Terminal 2 (optional - external control)
python2 slam_controller.py
```

**Just view camera:**
```bash
python2 view_fast.py
```
