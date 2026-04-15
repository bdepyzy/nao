# NAO Robot Controller

Control NAO robot with WASD keys + video feed.

## Files

- `run_controller.py` - Auto-launches SSH, starts streamer on robot, runs controller locally
- `slam_controller.py` - Controller with video window
- `config.py` - Robot IP, SSH password, settings
- `robot_streamer.py` - Upload this to `/home/nao/` on the robot (if not already there)
- `nao-ssh` - SSH private key (or use password auth)

## Quick Start

1. Edit `config.py` - set your robot's IP:
```python
ROBOT_IP = "169.254.81.31"
```

2. Run:
```bash
python2 run_controller.py
```

This will:
- SSH into the robot (password: "nao" from config)
- Start `robot_streamer.py` on the NAO
- Launch the controller locally with video
- When you hit ESC, it stops everything

## Controls

- **W/S** - Forward/Back
- **A/D** - Strafe left/right  
- **Q/E** - Turn left/right
- **SPACE** - Stop
- **ESC** - Quit

**Click on the video window first** - keys only work when window is focused.

## Troubleshooting

**"Cannot connect"** - Robot's NAOqi is down. SSH in and run: `naoqi &`

**"No video"** - Streamer not running on robot. The launcher should start it automatically.

**Password prompts** - Make sure `SSH_PASS = "nao"` is set in config.py

## Manual Mode

If auto-launcher doesn't work:

```bash
# Terminal 1 - SSH to robot and start streamer
ssh nao@169.254.81.31
python robot_streamer.py

# Terminal 2 - Run controller
python2 slam_controller.py
```
