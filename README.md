<p align="center">
  <h1 align="center">🚁 Autonomous Drone Computer Vision Projects</h1>
  <p align="center">
    <strong>PX4 AI Tracker &nbsp;·&nbsp; Aerial YOLOv8 Object Detection</strong>
  </p>
  <p align="center">
    <em>PX4 Autopilot · ROS 2 Humble · Gazebo · OpenCV · YOLOv8 · MAVSDK · Docker</em>
  </p>
  <p align="center">
    <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" />
    <img src="https://img.shields.io/badge/PX4-SITL-orange" />
    <img src="https://img.shields.io/badge/YOLOv8-Ultralytics-purple" />
    <img src="https://img.shields.io/badge/Docker-GPU-informational?logo=docker" />
    <img src="https://img.shields.io/badge/Python-3.10+-yellow?logo=python" />
  </p>
</p>

---

This repository contains **two complementary computer-vision drone projects** developed for the Master SIPNOM (Intelligent Systems) programme at USTHB. Both projects are built on the PX4 / ROS 2 / Gazebo ecosystem.

| | Project 1 — PX4 AI Tracker | Project 2 — YOLOv8 Aerial Detection |
|---|---|---|
| **Simulator** | Gazebo Classic | Gazebo Garden |
| **Drone** | Iris (`iris_cam`) | x500_depth |
| **Camera** | Fixed downward | 2-axis gimbal (pitch + yaw) |
| **CV approach** | OpenCV classical | YOLOv8 deep learning |
| **Flight control** | ROS 2 offboard | MAVSDK keyboard |
| **Orchestration** | Shell scripts | tmuxinator |
| **Deployment** | Bare-metal | Docker + GPU passthrough |
| **Targets** | Roads · humans · maze | Moving car |

---

## 🏙️ Project 1 — PX4 AI Tracker

Fully autonomous drone system combining classical computer vision with PID flight control across three missions and a general-purpose GUI.

### Missions

| Mission | Description |
|---------|-------------|
| 🛣️ **City Road Follower** | Follows lane markings over a CitySim city using HSV colour segmentation, contour fitting and PID yaw control |
| 🌙 **Nighttime Thermal Tracker** | Detects and tracks human heat signatures with a simulated thermal camera and dual-axis PID visual servoing |
| 🧩 **Maze Solver** | Ascends to scan a 10×10 grid maze, detects walls via OpenCV thresholding (YOLO-style overlays), solves the path with A\*, then navigates waypoint by waypoint |
| 🎯 **Demo Mission** | Autonomous circle orbit → figure-8 → spiral descent with no user input |

### Architecture

```
┌──────────────┐     ┌───────────────┐     ┌──────────────────┐
│ PX4 SITL     │     │ Micro         │     │ Perception       │
│ (Autopilot)  │◄───►│ XRCE-DDS      │◄───►│ Nodes (OpenCV)   │
└──────┬───────┘     └───────────────┘     └────────┬─────────┘
       │                                            │
┌──────┴───────┐                          ┌─────────┴────────┐
│ Gazebo       │                          │ Mission          │
│ Classic      │                          │ Controller / GUI │
└──────────────┘                          └──────────────────┘
```

### Project Structure

```
px4_ai_tracker/
├── road_detection_node.py        # OpenCV road detection (HSV + contours)
├── road_follower_node.py         # 14-state FSM road-following controller
├── thermal_perception_node.py    # Thermal camera human detection (Inferno colormap)
├── offboard_tracker_node.py      # PID visual servoing tracker
├── maze_solver_mission.py        # Maze solver GUI + A* pathfinding
├── demo_mission.py               # Autonomous demo flight patterns
├── drone_gui.py                  # Tkinter GUI — live feed, telemetry, 9 missions
├── launch_city_tracker.sh        # 🏙️  One-click city road-following launcher
├── launch_thermal_tracker.sh     # 🌙  One-click thermal tracking launcher
├── launch_maze_solver.sh         # 🧩  One-click maze solver launcher
├── fly_now.sh                    # 🚁  One-click demo launcher
├── fastdds_udp_only.xml          # FastDDS UDP-only transport profile
├── citysim/                      # CitySim city simulation assets
└── PX4-Autopilot/                # PX4 firmware (SITL build)
```

### Installation

**Prerequisites:** Ubuntu 22.04 · ROS 2 Humble · Gazebo Classic 11 · Python 3.10+

```bash
# 1. Install ROS 2 Humble
sudo apt install ros-humble-desktop ros-humble-gazebo-ros ros-humble-gazebo-plugins
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc

# 2. Clone this repository
git clone https://github.com/dhiaeddine-ayh/archi-drone-project.git
cd archi-drone-project

# 3. Build PX4 Autopilot (SITL)
cd PX4-Autopilot && bash Tools/setup/ubuntu.sh
make px4_sitl gazebo-classic && cd ..

# 4. Install Micro XRCE-DDS Agent
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig && cd ../..

# 5. Build ROS 2 workspace with px4_msgs
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ~/ros2_ws && colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc && source ~/.bashrc

# 6. Build CitySim
cd citysim && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../../citysim_install
make -j$(nproc) && make install && cd ../..

# 7. Python dependencies
python3 -m venv venv && source venv/bin/activate
pip install opencv-python numpy Pillow
```

### Usage

```bash
# City Road Tracker (GUI auto-opens)
bash launch_city_tracker.sh

# Nighttime Thermal Tracker
bash launch_thermal_tracker.sh

# Maze Solver
bash launch_maze_solver.sh

# Demo flight patterns (circle → figure-8 → spiral)
bash fly_now.sh
```

### GUI Controls

| Key / Button | Action |
|---|---|
| **ARM + TAKEOFF** | Arm motors and take off to 15 m |
| **LAND** | Auto-land at current position |
| **E-STOP** | Emergency motor kill |
| Arrow keys | Move forward / back / left / right |
| `W` / `X` | Altitude up / down |
| `A` / `D` | Yaw left / right |
| `S` | Start road-following |
| `E` | Land |
| Escape | Emergency stop |

**Available mission buttons:** Follow Roads · Follow Cars · Road Patrol · Circle Orbit · Square Patrol · Hover · Figure-8 · Grid Scan · Return Home

### Key ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Downward camera feed |
| `/road_tracker/road_offset` | `geometry_msgs/Point` | Lateral offset + heading |
| `/road_tracker/annotated_image` | `sensor_msgs/Image` | Annotated road debug view |
| `/thermal_tracker/target_centroid` | `geometry_msgs/Point` | Normalised target position |
| `/thermal_tracker/annotated_image` | `sensor_msgs/Image` | Thermal view (Inferno) |
| `/mission/command` | `std_msgs/String` | Mission command input |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | Position/velocity commands |

### Troubleshooting

| Issue | Fix |
|-------|-----|
| Gazebo city models missing | Verify `GAZEBO_MODEL_PATH` includes `citysim_install/share/citysim-0/models` |
| PX4 ↔ ROS 2 no communication | Check DDS Agent: `MicroXRCEAgent udp4 -p 8888` |
| Shared-memory DDS errors | Set `FASTRTPS_DEFAULT_PROFILES_FILE=./fastdds_udp_only.xml` |
| Camera not publishing | Verify drone SDF includes `gazebo_ros_camera` plugin |
| Offboard mode rejected | Heartbeats must be sent ≥ 2 s before arming (launch scripts handle this) |

---

## 🛸 Project 2 — PX4-ROS2-Gazebo-YOLOv8

Real-time aerial object detection using **YOLOv8** on a **2-axis gimbal camera** with full keyboard flight via **MAVSDK**, packaged in a single Docker container.

> Based on: [monemati/PX4-ROS2-Gazebo-YOLOv8](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8)

### Features

- ✅ YOLOv8 real-time detection on live drone camera feed
- ✅ 2-axis gimbal (pitch + yaw) adjustable in flight
- ✅ Keyboard-controlled flight via MAVSDK (WASD + arrows)
- ✅ Moving hatchback car target driving in circles
- ✅ Docker with GPU passthrough + X11 forwarding
- ✅ Full stack in one tmuxinator 6-pane window

### Architecture

```
┌────────────┐   ┌───────────────┐   ┌─────────────────────┐
│ PX4 SITL   │   │ Micro         │   │ ros_gz_bridge        │
│ x500_depth │◄──│ XRCE-DDS      │   │ (Gazebo → ROS 2 cam) │
└────────────┘   └───────────────┘   └──────────┬──────────┘
      │                                          │
┌─────┴──────┐                        ┌──────────▼──────────┐
│ Gazebo     │                        │ YOLOv8 Detection    │
│ Garden     │                        │ uav_camera_det.py   │
└────────────┘                        └─────────────────────┘
                                      ┌─────────────────────┐
                                      │ MAVSDK Keyboard     │
                                      │ keyboard-mavsdk.py  │
                                      └─────────────────────┘
```

### tmuxinator Pane Layout

| Pane | Service |
|------|---------|
| 1 | Micro XRCE-DDS Agent |
| 2 | PX4 SITL (`x500_depth`) |
| 3 | ROS-Gazebo camera bridge |
| 4 | YOLOv8 detection display |
| 5 | Moving car (`move_car.py`) |
| 6 | Keyboard drone controller |

### Gimbal Camera System

`setup_gimbal.py` rewrites the `x500_depth` SDF at build time to add:

- **`gimbal_yaw_joint`** — revolute around Z-axis, range ±90°
- **`gimbal_pitch_joint`** — revolute around Y-axis, range −90° to +30° (default −45°)

Each joint is driven by a Gazebo `JointPositionController` plugin on topics `/gimbal/cmd_pitch` and `/gimbal/cmd_yaw`.

### Keyboard Controls

| Key | Action |
|-----|--------|
| `r` | Arm |
| `l` | Land |
| `w` / `s` | Throttle up / down |
| `a` / `d` | Yaw left / right |
| Arrow keys | Roll / Pitch |
| `j` / `k` | Gimbal pitch down / up |
| `n` / `m` | Gimbal yaw left / right |
| Ctrl+C | Quit |

### Quick Start (Docker)

```bash
# Allow X11 access
xhost +local:docker

# Pull pre-built image
docker pull monemati/px4_ros2_gz_yolov8_image

# Launch (tmuxinator starts all 6 services automatically)
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run --privileged -it --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e MESA_GL_VERSION_OVERRIDE=3.3 \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --env="XAUTHORITY=$XAUTH" \
  --volume="$XAUTH:$XAUTH" \
  --network=host --ipc=host --shm-size=2gb \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --rm --name px4_ros2_gz_yolov8_container \
  px4_ros2_gz_yolov8_image
```

Switch to the keyboard controller pane (**Ctrl+b → arrow key**), then press **`r`** to arm.

### Manual Installation (no Docker)

```bash
# Python virtual environment
python -m venv ~/px4-venv && source ~/px4-venv/bin/activate

# Clone
git clone https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8.git

# Install PX4 (Gazebo Garden / SITL)
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot && make px4_sitl

# Install ROS 2 Humble + Gazebo Garden bridge
sudo apt install ros-humble-desktop ros-humble-ros-gzgarden

# Install Python packages
pip install mavsdk aioconsole numpy opencv-python ultralytics

# Set up gimbal (edit MODEL_PATH in script first)
python setup_gimbal.py

# Copy models and world
cp -r models/* ~/.gz/models/
cp worlds/default.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```

### Running (without Docker)

Open 6 terminals:

```bash
# T1 — DDS Agent
MicroXRCEAgent udp4 -p 8888

# T2 — PX4 SITL
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0,0,-0.7" \
  PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4

# T3 — Camera bridge
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image \
  --ros-args -r /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image:=/camera

# T4 — YOLOv8 detection
source ~/px4-venv/bin/activate && python uav_camera_det.py

# T5 — Moving car
source ~/px4-venv/bin/activate && python move_car.py

# T6 — Keyboard controller (focus this terminal)
source ~/px4-venv/bin/activate && python keyboard-mavsdk-test.py
```

---

## 👥 Authors

| Name | Role |
|------|------|
| **Ayachi Dhia Eddine** | Project lead, computer vision |
| **Zemmouli Ahmed** | Flight control, ROS 2 integration |
| **Touhari Mounir** | Simulation, system integration |

Faculty of Electrical Engineering — USTHB  
Master SIPNOM — Intelligent Systems

---

## 📄 License

Educational and research use.  
PX4 Autopilot — BSD-3-Clause · CitySim — Apache 2.0 · Ultralytics YOLOv8 — AGPL-3.0

---

## 🙏 Acknowledgements

- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
- [monemati/PX4-ROS2-Gazebo-YOLOv8](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8)
- [ROS 2](https://www.ros.org/) · [Gazebo](https://gazebosim.org) · [MAVSDK](https://mavsdk.mavlink.io)
