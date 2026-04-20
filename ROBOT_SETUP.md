# Robot Setup Guide

This robot uses:

- A Raspberry Pi 4 for ROS 2, behavior, microphone input, and camera streaming
- A Raspberry Pi Pico for motors, encoders, and ToF sensors
- A separate laptop for YOLO and target tracking over Wi-Fi

The Pi and laptop must be on the same Wi-Fi network.

## 1. Hardware Layout

Current Pico firmware expects these pins:

- Left motor PWM: `GP15`
- Left motor direction: `GP14`
- Right motor PWM: `GP16`
- Right motor direction: `GP17`
- Left encoder A/B: `GP4`, `GP13`
- Right encoder A/B: `GP2`, `GP3`
- I2C SDA/SCL: `GP6`, `GP7`
- Left ToF `XSHUT`: `GP8`
- Right ToF `XSHUT`: `GP9`

If your wiring is different, update [firmware/pico_micro_ros/src/main.c](./firmware/pico_micro_ros/src/main.c) before flashing the Pico.

Mounting:

- USB camera faces straight ahead
- ReSpeaker mic array has a fixed front direction
- Left and right ToF sensors are mounted at the front, angled slightly outward
- Keep Pi, Pico, motor driver, and sensors on a common ground

## 2. Flash The Pico

Build the Pico firmware from [firmware/pico_micro_ros](./firmware/pico_micro_ros).

Example:

```bash
cd firmware/pico_micro_ros
mkdir -p build
cd build
cmake ..
make -j4
```

Put the Pico in `BOOTSEL` mode and copy the generated `.uf2` file to it.

Then connect the Pico to the Pi over USB.

## 3. Set Up The Pi

Install Ubuntu `24.04` and ROS 2 `Jazzy`.

Install needed tools and packages:

```bash
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions ros-jazzy-desktop ros-jazzy-micro-ros-agent
```

Clone and build the workspace:

```bash
cd ~
git clone <your-repo-url> Ruff-Ruff-Rover
cd Ruff-Ruff-Rover
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
python3 -m pip install numpy opencv-python pyaudio pyusb
```

Plug into the Pi:

- Pico by USB
- USB camera
- ReSpeaker USB mic array

## 4. Set Up The Laptop

Install Ubuntu `24.04` and ROS 2 `Jazzy`.

Clone and build the same repo:

```bash
cd ~
git clone <your-repo-url> Ruff-Ruff-Rover
cd Ruff-Ruff-Rover
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
python3 -m pip install numpy opencv-python ultralytics
```

Download and place these model files where the config expects them:

- `yolo11n.pt` in the repo root: [download](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt)
- `models/face_detection_yunet_2023mar.onnx` in the `models` folder: [download](https://huggingface.co/opencv/face_detection_yunet/resolve/main/face_detection_yunet_2023mar.onnx)
- `models/face_recognition_sface_2021dec.onnx` in the `models` folder: [download](https://huggingface.co/opencv/face_recognition_sface/resolve/main/face_recognition_sface_2021dec.onnx)

Main robot settings live in [src/rover_bringup/config/rover_params.yaml](./src/rover_bringup/config/rover_params.yaml).

## 5. Set ROS 2 Networking

On both the Pi and the laptop:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Ruff-Ruff-Rover/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

Both machines must use the same `ROS_DOMAIN_ID`.

## 6. Bring Up The Robot

Start laptop vision first:

```bash
ros2 launch rover_bringup laptop_vision.launch.py
```

Then start the robot on the Pi:

```bash
ros2 launch rover_bringup full_bringup.launch.py
```

If the Pico shows up on a different serial device, set it manually:

```bash
ros2 launch rover_bringup full_bringup.launch.py micro_ros_device:=/dev/ttyACM1
```

For hardware-only checks without behavior, use:

```bash
ros2 launch rover_bringup self_test.launch.py
```

## 7. What Should Happen

- The Pico connects through `micro_ros_agent`
- The Pi publishes camera frames over Wi-Fi
- The laptop runs YOLO and publishes `/target_track`
- The Pi waits for self-test to pass
- The robot stays still until a whistle is detected
- After a whistle, it turns, finds the matching person, and moves toward them while avoiding obstacles

## 8. Before Driving On The Floor

Check these first:

- Wheels spin in the correct direction
- Encoder counts change when each wheel moves
- Both ToF sensors publish valid ranges
- The whistle node detects direction correctly
- The laptop sees the camera stream and publishes `/target_track`
- The robot remains stopped when there is no whistle

Do the first test with the wheels off the ground.

## 9. Tuning

If behavior needs adjustment, edit:

- [src/rover_bringup/config/rover_params.yaml](./src/rover_bringup/config/rover_params.yaml)

Most useful parameters:

- `turn_tolerance_deg`
- `arrival_range_m`
- `arrival_bbox_width_px`
- `soft_avoid_distance_m`
- `hard_stop_distance_m`
- `camera_hfov_deg`
- `doa_offset_deg`

## 10. Common Problems

No Pico connection:

- Check the USB cable
- Check the `micro_ros_device` path
- Reflash the Pico

Laptop vision does not see the robot:

- Make sure both machines are on the same Wi-Fi
- Make sure both use the same `ROS_DOMAIN_ID`
- Make sure `ROS_LOCALHOST_ONLY=0`

Robot turns the wrong way:

- Fix mic array mounting
- Adjust `doa_offset_deg`
- Confirm left/right motor wiring

Robot stops too early or too late:

- Tune `arrival_range_m`
- Tune `arrival_bbox_width_px`
