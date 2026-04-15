# NAO Robot Controller

Control NAO robot with WASD keys + video feed.

## Files

- `run_controller.py` - Launcher (just runs slam_controller.py)
- `slam_controller.py` - Controller with video window
- `config.py` - Robot IP and settings
- `robot_streamer.py` - **Upload this to `/home/nao/` on the robot**

## Quick Start

### 1. Start the streamer on the robot

SSH into the NAO and start the streamer:

```bash
ssh nao@169.254.81.31
python robot_streamer.py
```

Leave this running.

### 2. Run the controller locally

In another terminal:

```bash
python2 run_controller.py
```

Or directly:

```bash
python2 slam_controller.py
```

## Controls

- **W/S** - Forward/Back
- **A/D** - Strafe left/right  
- **Q/E** - Turn left/right
- **SPACE** - Stop
- **ESC** - Quit

**Click on the video window first** - keys only work when window is focused.

## Config

Edit `config.py` to set your robot's IP:

```python
ROBOT_IP = "169.254.81.31"
```

## Troubleshooting

**"Cannot connect to NAO"** - Robot's NAOqi is down. SSH in and run: `naoqi &`

**"No video"** - Streamer not running on robot. See step 1 above.

**Double-speaking robot** - You started multiple NAOqi processes. SSH in and run: `killall naoqi-bin`, then restart with `naoqi &`
