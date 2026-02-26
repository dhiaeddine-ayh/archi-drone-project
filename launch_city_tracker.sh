#!/bin/bash
# ═══════════════════════════════════════════════════════════════
#   🏙️ CITYSIM ROAD-TRACKING DRONE — LAUNCHER
#   Drone follows road lines over CitySim city using OpenCV
# ═══════════════════════════════════════════════════════════════

# Note: not using set -e because pkill returns non-zero when no processes found

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "  🏙️  CITYSIM ROAD-TRACKING DRONE — LAUNCHER"
echo "═══════════════════════════════════════════════════════════════"
echo "  Autonomous road-following over CitySim city"
echo "  Downward camera | OpenCV lane detection | PID servoing"
echo "═══════════════════════════════════════════════════════════════"
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PX4_DIR="${SCRIPT_DIR}/PX4-Autopilot"
CITYSIM_INSTALL="${SCRIPT_DIR}/citysim_install"

# Validate PX4 directory
if [ ! -d "$PX4_DIR" ]; then
    echo "❌ ERROR: PX4-Autopilot directory not found at $PX4_DIR"
    exit 1
fi

# Kill leftover processes from previous runs
echo "▶ Cleaning up old processes..."
pkill -9 MicroXRCEAgent 2>/dev/null
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
pkill -9 gazebo 2>/dev/null
sleep 1
echo "   ✅ Cleanup done."

# ─────────────────────────────────────────────────────────────
# 0. Set up CitySim paths so Gazebo can find models/media
# ─────────────────────────────────────────────────────────────
echo "▶ [0/4] Setting up CitySim Gazebo paths..."
export GAZEBO_MODEL_PATH="${CITYSIM_INSTALL}/share/citysim-0/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="${CITYSIM_INSTALL}/share/citysim-0:${GAZEBO_RESOURCE_PATH}"
export GAZEBO_PLUGIN_PATH="${CITYSIM_INSTALL}/lib/citysim-0/plugins:${GAZEBO_PLUGIN_PATH}"
echo "   GAZEBO_MODEL_PATH includes: ${CITYSIM_INSTALL}/share/citysim-0/models"
echo "   ✅ CitySim paths configured."

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
# 2. Start PX4 SITL + Gazebo City World
# ─────────────────────────────────────────────────────────────
echo "▶ [2/5] Starting PX4 SITL + Gazebo (city_drone world)..."
echo "         Model: iris_cam (quad + downward camera)"
echo "         This takes ~20-30 seconds..."
gnome-terminal --tab --title="🚁 PX4 City" -- bash -c "\
    source /opt/ros/humble/setup.bash && \
    source /usr/share/gazebo/setup.bash && \
    export FASTRTPS_DEFAULT_PROFILES_FILE=${SCRIPT_DIR}/fastdds_udp_only.xml && \
    export GAZEBO_MODEL_PATH=\"${CITYSIM_INSTALL}/share/citysim-0/models:\${GAZEBO_MODEL_PATH}\" && \
    export GAZEBO_RESOURCE_PATH=\"${CITYSIM_INSTALL}/share/citysim-0:\${GAZEBO_RESOURCE_PATH}\" && \
    export GAZEBO_PLUGIN_PATH=\"/opt/ros/humble/lib:${CITYSIM_INSTALL}/lib/citysim-0/plugins:\${GAZEBO_PLUGIN_PATH}\" && \
    cd ${PX4_DIR} && \
    PX4_SITL_WORLD=road_cars make px4_sitl gazebo-classic_iris_cam; \
    exec bash"

# Wait for Gazebo + PX4 to fully launch (city is large)
echo "   ⏳ Waiting 15 seconds for Gazebo to initialize..."
sleep 15
echo "   ✅ Gazebo should be ready."

# ─────────────────────────────────────────────────────────────
# 3. Start Road Detection Node (+ camera window)
# ─────────────────────────────────────────────────────────────
echo "▶ [3/5] Starting Road Detection Node (camera window)..."
gnome-terminal --tab --title="🛣️ Road Detection" -- bash -c "\
    source /opt/ros/humble/setup.bash && \
    source ~/ros2_ws/install/setup.bash && \
    export FASTRTPS_DEFAULT_PROFILES_FILE=${SCRIPT_DIR}/fastdds_udp_only.xml && \
    echo '═══ Road Detection Node ═══' && \
    python3 ${SCRIPT_DIR}/road_detection_node.py; \
    exec bash"
sleep 3

# ─────────────────────────────────────────────────────────────
# 4. Start Road Follower Node
# ─────────────────────────────────────────────────────────────
echo "▶ [4/5] Starting Road Follower..."
gnome-terminal --tab --title="🎯 Road Follower" -- bash -c "\
    source /opt/ros/humble/setup.bash && \
    source ~/ros2_ws/install/setup.bash && \
    export FASTRTPS_DEFAULT_PROFILES_FILE=${SCRIPT_DIR}/fastdds_udp_only.xml && \
    echo '═══ Road Follower Mission ═══' && \
    python3 ${SCRIPT_DIR}/road_follower_node.py; \
    exec bash"
sleep 2

# ─────────────────────────────────────────────────────────────
# 5. Start Drone GUI (Camera + Controls)
# ─────────────────────────────────────────────────────────────
echo "▶ [5/5] Starting Drone Control Interface..."
gnome-terminal --tab --title="🎮 Drone GUI" -- bash -c "\
    source /opt/ros/humble/setup.bash && \
    source ~/ros2_ws/install/setup.bash && \
    export FASTRTPS_DEFAULT_PROFILES_FILE=${SCRIPT_DIR}/fastdds_udp_only.xml && \
    python3 ${SCRIPT_DIR}/drone_gui.py; \
    exec bash"

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "  ✅ ALL SYSTEMS LAUNCHED!"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "  A control window will open with:"
echo "    📷  Live drone camera feed"
echo "    📊  Altitude, speed, road status"
echo "    ▶   START button (or press S)"
echo "    ⏹   END button   (or press E)"
echo ""
