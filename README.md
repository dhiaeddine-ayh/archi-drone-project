<p align="center">
  <h1 align="center">🚁 PX4 AI Tracker</h1>
  <p align="center">
    <strong>Autonomous Drone System with Computer Vision &amp; Thermal Tracking</strong>
  </p>
  <p align="center">
    <em>PX4 Autopilot · ROS 2 Humble · Gazebo Classic · OpenCV · PID Visual Servoing</em>
  </p>
</p>

---

## 📋 Overview

**PX4 AI Tracker** is a fully autonomous drone system built in simulation that combines real-time computer vision with flight control. The project features two core missions:

| Mission | Description |
|---------|-------------|
| 🏙️ **City Road Tracker** | Autonomous road-following over a realistic city using a downward-facing camera, HSV colour segmentation and contour-based lane detection |
| 🌙 **Nighttime Thermal Tracker** | Autonomous human tracking in a dark environment using a simulated thermal camera and PID visual servoing |

A **Drone Control GUI** provides live video, telemetry and one-click access to **9 flight missions** (road follow, car follow, patrol, circle orbit, square patrol, figure-8, grid scan, hover, return-to-home).

---

## 🏗️ Architecture

```
┌──────────────┐     ┌───────────────┐     ┌──────────────────┐
│ PX4 SITL     │     │ Micro         │     │ Perception       │
│ (Autopilot)  │◄───►│ XRCE-DDS      │◄───►│ Nodes (OpenCV)   │
└──────┬───────┘     └───────────────┘     └────────┬─────────┘
       │                                            │
┌──────┴───────┐                            ┌───────┴──────────┐
│ Gazebo       │                            │ Mission          │
│ Classic      │                            │ Controller / GUI │
└──────────────┘                            └──────────────────┘
```

**Key Components:**
- **PX4 Autopilot (SITL)** — flight controller in Software-In-The-Loop mode
- **Gazebo Classic** — 3D physics simulator with custom worlds & drone models
- **Micro XRCE-DDS** — bridge between PX4 uORB and ROS 2 DDS
- **ROS 2 Humble** — middleware for inter-process communication
- **OpenCV** — real-time image processing for road detection & thermal perception

---

## 📁 Project Structure

```
px4_ai_tracker/
├── road_detection_node.py        # OpenCV road surface detection (HSV + contours)
├── road_follower_node.py         # Autonomous road-following controller (14-state FSM)
├── thermal_perception_node.py    # Thermal camera human detection (Inferno colormap)
├── offboard_tracker_node.py      # PID visual servoing for thermal tracking
├── demo_mission.py               # Demo flight patterns (circle, figure-8, spiral)
├── drone_gui.py                  # Tkinter GUI with live camera, telemetry & missions
├── launch_city_tracker.sh        # 🏙️  One-click launcher for city road-following
├── launch_thermal_tracker.sh     # 🌙  One-click launcher for thermal tracking
├── fly_now.sh                    # 🚁  One-click launcher for demo mission
├── fastdds_udp_only.xml          # FastDDS UDP-only transport config
├── PX4-Autopilot/                # PX4 firmware (SITL build)
├── citysim/                      # CitySim city model source
├── citysim_install/              # CitySim installed assets
└── venv/                         # Python virtual environment
```

---

## 🛠️ Installation

### Prerequisites

- **Ubuntu 22.04 LTS**
- **ROS 2 Humble**
- **Gazebo Classic 11**
- **Python 3.10+**

### Step 1 — Install ROS 2 Humble

```bash
# Follow the official docs: https://docs.ros.org/en/humble/Installation.html
sudo apt install ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2 — Install Gazebo Classic

```bash
sudo apt install gazebo ros-humble-gazebo-ros ros-humble-gazebo-plugins
```

### Step 3 — Clone this repository

```bash
git clone https://github.com/<your-username>/px4_ai_tracker.git
cd px4_ai_tracker
```

### Step 4 — Build PX4 Autopilot (SITL)

```bash
cd PX4-Autopilot
bash Tools/setup/ubuntu.sh
make px4_sitl gazebo-classic
cd ..
```

### Step 5 — Install Micro XRCE-DDS Agent

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install && sudo ldconfig
cd ../..
```

### Step 6 — Build ROS 2 workspace with px4_msgs

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 7 — Build CitySim

```bash
cd ~/px4_ai_tracker/citysim
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../../citysim_install
make -j$(nproc) && make install
cd ../..
```

### Step 8 — Install Python dependencies

```bash
python3 -m venv venv
source venv/bin/activate
pip install opencv-python numpy Pillow
```

---

## 🚀 Usage

### 🏙️ City Road Tracker

Launches PX4 + Gazebo city world + road detection + road follower + GUI:

```bash
bash launch_city_tracker.sh
```

Once all windows are open:
1. The GUI window appears with a live camera feed
2. Click **🔓 ARM + TAKEOFF** (or press `S` to start road-following)
3. The drone takes off and autonomously follows road lines
4. Click **⏹ LAND** (or press `E`) to land

### 🌙 Nighttime Thermal Tracker

Launches PX4 + Gazebo dark world + thermal perception + PID tracker:

```bash
bash launch_thermal_tracker.sh
```

The drone will automatically:
1. Take off to cruise altitude
2. Search for heat signatures (rotating scan)
3. Lock onto the target and track it with PID visual servoing
4. Land after the tracking timeout

View the thermal feed:
```bash
ros2 run rqt_image_view rqt_image_view /thermal_tracker/annotated_image
```

### 🚁 Demo Mission (Circle + Figure-8 + Spiral)

```bash
bash fly_now.sh
```

The drone performs circle orbit → figure-8 → spiral descent → auto-land, fully autonomously.

---

## 🎮 GUI Controls

| Control | Action |
|---------|--------|
| **🔓 ARM + TAKEOFF** | Arm motors and take off to 15m |
| **⏹ LAND** | Auto-land at current position |
| **🛑 E-STOP** | Emergency motor kill |
| **Arrow keys** | Move forward/back/left/right |
| **W / X** | Increase / decrease altitude |
| **A / D** | Rotate left / right (yaw) |
| **S** | Start road-following mission |
| **E** | Land |
| **Escape** | Emergency stop |

### Available Missions

| Button | Command | Description |
|--------|---------|-------------|
| 🛣️ Follow Roads | `START` | Autonomous road following |
| 🚗 Follow Cars | `FOLLOWCAR` | Track and follow a vehicle |
| 🚓 Road Patrol | `PATROL` | Patrol along road waypoints |
| 🔄 Circle Orbit | `CIRCLE` | Circular flight pattern |
| ⬜ Square Patrol | `SQUARE` | Square waypoint pattern |
| ✈️ Hover | `HOVER` | Hold position |
| ∞ Figure-8 | `FIGURE8` | Lemniscate flight pattern |
| 📐 Grid Scan | `GRID` | Systematic area coverage |
| 🏠 Return Home | `RTH` | Return to launch position |

---

## 🧩 Node Details

### `road_detection_node.py`
- Subscribes to `/camera/image_raw`
- HSV thresholding for asphalt detection (low saturation, medium value)
- Morphological clean-up → contour detection → centre-of-mass + fitted heading
- Publishes road offset, detection flag and annotated debug image

### `road_follower_node.py`
- 14-state finite state machine (WAITING → PREFLIGHT → TAKEOFF → mission → LAND → DONE)
- Position-based offboard control via PX4
- Yaw correction using road offset + heading angle with tunable gains
- Subscribes to `/mission/command` for mission switching

### `thermal_perception_node.py`
- Binary threshold on grayscale thermal image → contour detection
- Inferno colourmap annotated output with bounding boxes and HUD
- Publishes normalised centroid (−1 to +1) for the PID tracker

### `offboard_tracker_node.py`
- 7-state FSM: PREFLIGHT → TAKEOFF → SEARCH → TRACKING → LOST_TARGET → LAND → DONE
- Dual PID controllers (X/Y axes) with anti-windup for velocity-based visual servoing
- Automatic search pattern (yaw rotation) when target is lost

### `drone_gui.py`
- Tkinter GUI with dark theme
- Live camera feed (annotated or raw), telemetry bar, 9 mission buttons
- ROS 2 spin in background thread, GUI updates at ~12 Hz

---

## ⚙️ Configuration

### Road Detection HSV Tuning

Adjust via ROS 2 parameters:
```bash
python3 road_detection_node.py --ros-args \
  -p road_h_low:=0 -p road_h_high:=180 \
  -p road_s_low:=0 -p road_s_high:=80 \
  -p road_v_low:=20 -p road_v_high:=160 \
  -p min_road_area:=500
```

### Thermal Detection Tuning

```bash
python3 thermal_perception_node.py --ros-args \
  -p thermal_threshold:=180 \
  -p min_contour_area:=200 \
  -p max_contour_area:=50000
```

### FastDDS Transport

The `fastdds_udp_only.xml` forces UDP transport to avoid shared-memory issues. It is loaded automatically by all launch scripts via:
```
export FASTRTPS_DEFAULT_PROFILES_FILE=./fastdds_udp_only.xml
```

---

## 📡 ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Downward camera feed |
| `/road_tracker/road_offset` | `geometry_msgs/Point` | Road lateral offset + heading |
| `/road_tracker/road_detected` | `std_msgs/Bool` | Road detection flag |
| `/road_tracker/annotated_image` | `sensor_msgs/Image` | Annotated road debug view |
| `/thermal_camera/image_raw` | `sensor_msgs/Image` | Thermal camera feed |
| `/thermal_tracker/target_centroid` | `geometry_msgs/Point` | Normalised target position |
| `/thermal_tracker/target_detected` | `std_msgs/Bool` | Target detection flag |
| `/thermal_tracker/annotated_image` | `sensor_msgs/Image` | Thermal debug view (Inferno) |
| `/mission/command` | `std_msgs/String` | Mission command input |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | Position/velocity commands |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | Offboard heartbeat |
| `/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | PX4 vehicle state |

---

## 🧪 Troubleshooting

| Issue | Solution |
|-------|----------|
| Gazebo fails to load city models | Verify `GAZEBO_MODEL_PATH` includes `citysim_install/share/citysim-0/models` |
| PX4 and ROS 2 don't communicate | Check DDS Agent is running: `MicroXRCEAgent udp4 -p 8888` |
| Shared memory DDS errors | Ensure `FASTRTPS_DEFAULT_PROFILES_FILE` points to `fastdds_udp_only.xml` |
| Camera topic not publishing | Verify the drone model includes the `gazebo_ros_camera` plugin |
| Offboard mode rejected | Send heartbeats for ≥2 seconds before arming (handled by launch scripts) |

---

## 👥 Authors

- **Ayachi Dhia Eddine**

---

## 📄 License

This project is for educational and research purposes.  
PX4 Autopilot is licensed under BSD-3. CitySim is licensed under Apache 2.0.
