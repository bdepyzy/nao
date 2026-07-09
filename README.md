# NAO cuVSLAM Controller

Laptop Python 3 controller for NAO video, CUDA SLAM, and keyboard movement.

The laptop uses one virtual environment: `.venv`.

The robot does not use a virtual environment. The robot scripts run with NAO's
built-in Python and NAOqi.

## Files

- `config.py` - robot IP and stream URL used by laptop code
- `cuvslam_nao_controller.py` - laptop Python 3 video, cuVSLAM, and keyboard controller
- `robot_streamer.py` - robot-side camera stream server
- `nao_motion_bridge.py` - robot-side UDP movement bridge to `ALMotion`
- `scripts/setup_cuvslam_py3.sh` - creates laptop `.venv` with Python 3.12 and cuVSLAM

## 1. Connect Ethernet

Run this on the laptop, not on the robot.

```bash
nmcli connection modify 'Wired connection 1' \
  connection.interface-name enp83s0 \
  ipv4.method manual \
  ipv4.addresses 169.254.81.30/16 \
  ipv4.never-default yes \
  ipv6.method disabled

nmcli connection up 'Wired connection 1'
```

Check that the laptop routes to the robot through Ethernet:

```bash
ip route get 169.254.81.31
```

Expected:

```text
169.254.81.31 dev enp83s0 src 169.254.81.30
```

Then check the robot is reachable:

```bash
ping -c 3 169.254.81.31
ssh nao@169.254.81.31
```

## 2. Set Up The Laptop venv

Run this on the laptop.

```bash
scripts/setup_cuvslam_py3.sh
source .venv/bin/activate
```

This installs Python 3.12, cuVSLAM, OpenCV, NumPy, and PyYAML into `.venv`.

Do not install NAOqi or Python 2 into this laptop venv. NAOqi runs on the robot.

## 3. Copy Robot Scripts

Run this on the laptop.

```bash
scp robot_streamer.py nao_motion_bridge.py nao@169.254.81.31:/home/nao/
```

## 4. Start Robot Services

Open one SSH terminal to the robot:

```bash
ssh nao@169.254.81.31
python robot_streamer.py
```

Leave that running.

Open a second SSH terminal to the robot:

```bash
ssh nao@169.254.81.31
python nao_motion_bridge.py
```

Leave that running too.

At this point:

- `robot_streamer.py` serves camera video from the robot at `http://169.254.81.31:8080/stream`
- `nao_motion_bridge.py` listens for movement commands from the laptop on UDP port `8765`

## 5. Run The Laptop Controller

Run this on the laptop.

```bash
source .venv/bin/activate
python cuvslam_nao_controller.py
```

Two windows should open:

- `NAO cuVSLAM` - camera video, pose text, and keyboard focus
- `NAO SLAM Map` - top-down trajectory, current robot position, origin, and landmarks

Click the video window before using the keyboard.

## Controls

- `W/S` - forward/back
- `A/D` - strafe left/right
- `Q/E` - turn left/right
- `SPACE` - stop
- `ESC` - quit

## Configuration

If the robot IP changes, update `config.py`:

```python
ROBOT_IP = "169.254.81.31"
```

If your Ethernet interface is not `enp83s0`, find the correct name:

```bash
nmcli device status
```

Then use that interface name in the `nmcli connection modify` command.

## Troubleshooting

If `ping 169.254.81.31` fails, check the route:

```bash
ip route get 169.254.81.31
```

If it shows `wlan0`, the laptop is trying Wi-Fi instead of the Ethernet cable.
Bring the wired connection up again:

```bash
nmcli connection up 'Wired connection 1'
```

If SSH works but there is no video, `robot_streamer.py` is not running on the
robot.

If movement does not work, `nao_motion_bridge.py` is not running on the robot.

If a robot script cannot connect to NAOqi, restart NAOqi on the robot:

```bash
naoqi &
```
