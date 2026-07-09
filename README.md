# NAO Robot Controller

Control NAO robot with WASD keys + video feed.

## Files

- `config.py` - Robot IP and settings
- `robot_streamer.py` - MJPEG camera streamer; run this on the robot
- `slam_controller.py` - Python 2 camera viewer + direct NAO control
- `nao_motion_bridge.py` - Python 2 UDP bridge to `ALMotion`
- `cuvslam_nao_controller.py` - Python 3 cuVSLAM viewer + keyboard control
- `scripts/setup_nao_py2.sh` - Build/install `.venv2` with Python 2.7 + `qi`
- `scripts/setup_cuvslam_py3.sh` - Build/install `.venv3` with UV + cuVSLAM

## Setup

```bash
scripts/setup_nao_py2.sh
scripts/setup_cuvslam_py3.sh
```

## Robot Stream

Run this on the robot:

```bash
python robot_streamer.py
```

From the laptop, the stream should be available at:

```bash
http://169.254.81.31:8080/stream
```

## Direct Controller

```bash
source .venv2/bin/activate
python slam_controller.py
```

## cuVSLAM Controller

Terminal 1:
```bash
source .venv2/bin/activate
python nao_motion_bridge.py
```

Terminal 2:
```bash
source .venv3/bin/activate
python cuvslam_nao_controller.py
```

The cuVSLAM controller uses approximate NAO camera intrinsics. Replace the
constants at the top of `cuvslam_nao_controller.py` after calibration.

## Controls

- **W/S** - Forward/Back
- **A/D** - Strafe left/right  
- **Q/E** - Turn left/right
- **SPACE** - Stop
- **ESC** - Quit

**Click on the video window first** - keys only work when window is focused.

## Configuration

Edit `config.py` if the robot IP changes:

```python
ROBOT_IP = "169.254.81.31"
```

## Troubleshooting

**"Cannot connect to NAO"** - Robot's NAOqi is down. SSH in and run: `naoqi &`

**"No video"** - Streamer not running on robot. See step 1 above.

**Double-speaking robot** - You started multiple NAOqi processes. SSH in and run: `killall naoqi-bin`, then restart with `naoqi &`
