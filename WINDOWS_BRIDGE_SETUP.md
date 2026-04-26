# Windows Vision Bridge Setup

This setup keeps ROS 2 on the Raspberry Pi and runs YOLO on a Windows PC as a
plain Python server.

Use this path if:

- the Pi camera works locally
- WSL2 or native Windows ROS networking is unreliable
- you still want the Windows machine to do the heavy vision work

The bridge works like this:

- the Pi publishes `/camera/image/compressed`
- `vision_bridge_client_node` on the Pi sends the latest JPEG frame to Windows
- the Windows server runs the same YOLO + tracking logic used by the ROS vision node
- the Windows server sends back target data
- the Pi publishes `/vision/ready` and `/target_track` for the rest of the robot

## 1. Pi Setup

Build the Pi workspace with `rover_vision` included:

```bash
cd ~/Ruff-Ruff-Rover
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rover_interfaces rover_base rover_behavior rover_vision rover_bringup
source install/setup.bash
```

Set the Windows PC IP in
[src/rover_bringup/config/rover_params.yaml](./src/rover_bringup/config/rover_params.yaml)
under `vision_bridge_client_node.ros__parameters.server_host`.

Example:

```yaml
vision_bridge_client_node:
  ros__parameters:
    server_host: 192.168.1.50
    server_port: 8765
```

You can find the Windows PC IPv4 address with:

```powershell
ipconfig
```

## 2. Windows Setup

Open a normal terminal in the repo root:

```powershell
cd C:\Users\mmamd\OneDrive\Desktop\Git\Ruff-Ruff-Rover
```

Create and activate a virtual environment:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
```

If PowerShell blocks activation, run this once in a PowerShell window:

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

Then activate again:

```powershell
.\.venv\Scripts\Activate.ps1
```

Install Python dependencies into the virtual environment:

```powershell
python -m pip install --upgrade pip
python -m pip install -r tools\windows_vision_bridge_requirements.txt
```

Put these model files where the config expects them:

- `yolo11n.pt` in the repo root
- `models\face_detection_yunet_2023mar.onnx`
- `models\face_recognition_sface_2021dec.onnx`

The bridge server resolves model paths relative to the repo root. `yolo11n.pt`
is left in the repo root because that is what the existing
`vision_target_node` config already expects. You can move it into `models\` if
you prefer, but then also update `vision_target_node.ros__parameters.yolo_model`
in [src/rover_bringup/config/rover_params.yaml](./src/rover_bringup/config/rover_params.yaml)
to match, for example:

```yaml
vision_target_node:
  ros__parameters:
    yolo_model: models/yolo11n.pt
```

## 3. Start The Windows Vision Server

From the repo root on Windows:

```powershell
.\.venv\Scripts\Activate.ps1
python tools\vision_bridge_server.py --params-file src\rover_bringup\config\rover_params.yaml --host 0.0.0.0 --port 8765
```

If you want to see the incoming camera stream on the Windows PC with YOLO boxes
drawn on top, start the server in preview mode:

```powershell
.\.venv\Scripts\Activate.ps1
python tools\vision_bridge_server.py --params-file src\rover_bringup\config\rover_params.yaml --host 0.0.0.0 --port 8765 --show-preview
```

That preview window is the easiest way to tune exposure and confirm detections.
By default it draws all current YOLO person detections, even before a whistle
has happened. If you only want to see the currently selected tracked target, use:

```powershell
python tools\vision_bridge_server.py --params-file src\rover_bringup\config\rover_params.yaml --host 0.0.0.0 --port 8765 --show-preview --preview-selected-only
```

If Windows Defender asks about firewall access, allow it on your private
network.

You should see:

```text
Vision bridge server listening on http://0.0.0.0:8765
Waiting for Pi requests...
```

## 4. Start The Pi Robot

For the full whistle-follow behavior with the bridge:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Ruff-Ruff-Rover/install/setup.bash
ros2 launch rover_bringup bridge_full_bringup.launch.py micro_ros_device:="$PICO_DEV"
```

For testing only the camera + bridge client without behavior:

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Ruff-Ruff-Rover/install/setup.bash
ros2 launch rover_bringup hardware_bringup.launch.py micro_ros_device:="$PICO_DEV"
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Ruff-Ruff-Rover/install/setup.bash
ros2 launch rover_bringup vision_bridge_client.launch.py
```

## 5. Verify The Bridge

On the Pi:

```bash
ros2 topic echo /camera/ready --once
```

```bash
ros2 topic echo /vision/ready --once
```

```bash
ros2 topic echo /target_track
```

Expected:

- `/camera/ready` becomes `true`
- `/vision/ready` becomes `true` once the Windows server is reachable and the models load
- `/target_track` stays invisible until a whistle happens or a previous tracked target remains active
- if `--show-preview` is enabled, the Windows PC opens a live camera window with
  YOLO boxes overlaid

## 6. Full Robot Flow

1. Start the Windows vision bridge server.
2. Start `bridge_full_bringup.launch.py` on the Pi.
3. Wait for self-test to pass.
4. Whistle.
5. The Pi whistle node publishes `/whistle/event`.
6. The bridge client sends frames plus the whistle direction to Windows.
7. Windows picks the best person and sends back a track.
8. The Pi follow behavior consumes `/target_track` and drives the robot.

## 7. Troubleshooting

`/vision/ready` never goes true:

- Confirm the Windows server is running.
- Confirm `server_host` in `rover_params.yaml` points to the Windows machine.
- Confirm the Windows firewall allowed Python on the private network.
- Confirm the model files exist at the paths in the params file.

`Vision bridge request failed` on the Pi:

- Check that the Windows server terminal is still running.
- Check the IP address in `server_host`.
- Check that the Windows machine and Pi are on the same local network.

## 8. Network Requirement

Yes, both computers need to be on the same local network for the bridge as it
is currently written.

The Pi-side bridge client sends HTTP requests directly to the Windows machine
using its LAN IP address from `server_host`. That means:

- the Pi must be able to reach the Windows PC by IP
- the Windows firewall must allow the Python server on your private network
- both devices should normally be on the same Wi-Fi or same Ethernet/Wi-Fi LAN

They do not need ROS 2 discovery between them anymore, but they still need
ordinary IP connectivity between the Pi and the Windows machine.

Windows server says YOLO or OpenCV import failed:

- Re-run:

```powershell
py -m pip install -r tools\windows_vision_bridge_requirements.txt
```

No detections even though the server is running:

- Confirm the camera is facing forward and `/camera/image/compressed` is active.
- Confirm `yolo11n.pt` exists in the repo root on Windows.
- Try increasing light and testing with a person centered in frame.

Self-test times out waiting for vision:

- Start the Windows server first.
- Then start the Pi launch.
- Check `/vision/ready` on the Pi to confirm the bridge came up.
