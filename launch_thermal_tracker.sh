#!/bin/bash
# ═══════════════════════════════════════════════════════════════
#   🌙 NIGHTTIME THERMAL DRONE TRACKER — LAUNCHER
#   Starts all components for autonomous thermal tracking mission
# ═══════════════════════════════════════════════════════════════
#
# Architecture:
#   ┌──────────────┐    ┌───────────────┐    ┌──────────────────┐
#   │ PX4 SITL +   │    │ Thermal       │    │ Offboard Tracker │
#   │ Gazebo Night │◄──►│ Perception    │──► │ (PID Servoing)   │
#   │ World        │    │ Node          │    │                  │
#   └──────┬───────┘    └───────────────┘    └──────────────────┘
#          │
#   ┌──────┴───────┐
#   │ XRCE-DDS     │
#   │ Agent        │
#   └──────────────┘

set -e

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "  🌙  NIGHTTIME THERMAL DRONE TRACKER — LAUNCHER"
echo "═══════════════════════════════════════════════════════════════"
echo "  Autonomous thermal-guided human tracking mission"
echo "  Dark world | Thermal camera | PID visual servoing"
echo "═══════════════════════════════════════════════════════════════"
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR="${SCRIPT_DIR}/PX4-Autopilot"

# Validate PX4 directory
if [ ! -d "$PX4_DIR" ]; then
    echo "❌ ERROR: PX4-Autopilot directory not found at $PX4_DIR"
    exit 1
fi

# ─────────────────────────────────────────────────────────────
# 1. Start Micro XRCE-DDS Agent
# ─────────────────────────────────────────────────────────────
echo "▶ [1/4] Starting Micro XRCE-DDS Agent on UDP:8888..."
gnome-terminal --tab --title="🔗 DDS Agent" -- bash -c "\
    echo '═══ Micro XRCE-DDS Agent ═══'; \
    MicroXRCEAgent udp4 -p 8888; \
    exec bash"
sleep 2

# ─────────────────────────────────────────────────────────────
# 2. Start PX4 SITL + Gazebo Nighttime World
# ─────────────────────────────────────────────────────────────
echo "▶ [2/4] Starting PX4 SITL + Gazebo (nighttime_thermal world)..."
echo "         Model: iris_thermal_cam (downward thermal camera)"
echo "         This takes ~15-20 seconds..."
gnome-terminal --tab --title="🚁 PX4 Gazebo Night" -- bash -c "\
    cd ${PX4_DIR} && \
    PX4_SITL_WORLD=nighttime_thermal make px4_sitl gazebo-classic_iris_thermal_cam; \
    exec bash"

# Wait for Gazebo + PX4 to fully launch
echo "   ⏳ Waiting 30 seconds for Gazebo to initialize..."
sleep 30
echo "   ✅ Gazebo should be ready."

# ─────────────────────────────────────────────────────────────
# 3. Start Thermal Perception Node
# ─────────────────────────────────────────────────────────────
echo "▶ [3/4] Starting Thermal Perception Node..."
gnome-terminal --tab --title="🌡️ Thermal Perception" -- bash -c "\
    source /opt/ros/humble/setup.bash && \
    source ~/ros2_ws/install/setup.bash && \
    export FASTRTPS_DEFAULT_PROFILES_FILE=${SCRIPT_DIR}/fastdds_udp_only.xml && \
    echo '═══ Thermal Perception Node ═══' && \
    python3 ${SCRIPT_DIR}/thermal_perception_node.py; \
    exec bash"
sleep 3

# ─────────────────────────────────────────────────────────────
# 4. Start Offboard Tracker Node (Mission Controller)
# ─────────────────────────────────────────────────────────────
echo "▶ [4/4] Starting Offboard Tracker (Mission Controller)..."
gnome-terminal --tab --title="🎯 Tracker Mission" -- bash -c "\
    source /opt/ros/humble/setup.bash && \
    source ~/ros2_ws/install/setup.bash && \
    export FASTRTPS_DEFAULT_PROFILES_FILE=${SCRIPT_DIR}/fastdds_udp_only.xml && \
    echo '═══ Offboard Tracker Mission ═══' && \
    python3 ${SCRIPT_DIR}/offboard_tracker_node.py; \
    exec bash"

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "  ✅ ALL SYSTEMS LAUNCHED!"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "  🔗 DDS Agent       — Micro XRCE-DDS bridge (UDP:8888)"
echo "  🚁 PX4 Gazebo      — Nighttime world with thermal targets"
echo "  🌡️  Perception     — Thermal camera processing"
echo "  🎯 Tracker         — PID visual servoing controller"
echo ""
echo "  Mission flow: PREFLIGHT → TAKEOFF → SEARCH → TRACKING → LAND"
echo ""
echo "  📹 View thermal feed:  (in a new terminal)"
echo "     source /opt/ros/humble/setup.bash"
echo "     ros2 run rqt_image_view rqt_image_view /thermal_tracker/annotated_image"
echo ""
echo "  📊 Monitor topics:  (in a new terminal)"
echo "     source /opt/ros/humble/setup.bash"
echo "     ros2 topic echo /thermal_tracker/target_detected"
echo "     ros2 topic echo /thermal_tracker/target_centroid"
echo ""
