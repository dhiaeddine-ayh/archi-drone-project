#!/bin/bash
# ═══════════════════════════════════════════════
#   🚁 PX4 DRONE DEMO MISSION — LAUNCHER
#   Starts DDS Agent, PX4 Gazebo, and the Mission
# ═══════════════════════════════════════════════

echo ""
echo "═══════════════════════════════════════════════"
echo "  🚁  PX4 Drone Demo Mission Launcher"
echo "═══════════════════════════════════════════════"
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 1. Start the Micro-XRCE-DDS Agent
echo "▶ [1/3] Starting DDS Agent..."
gnome-terminal --tab --title="DDS Agent" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"
sleep 2

# 2. Start PX4 SITL + Gazebo
echo "▶ [2/3] Starting PX4 SITL + Gazebo (this takes ~15-20 seconds)..."
gnome-terminal --tab --title="PX4 Gazebo" -- bash -c "cd ${SCRIPT_DIR}/PX4-Autopilot && PX4_SITL_WORLD=empty make px4_sitl gazebo; exec bash"

# Wait 30 seconds for Gazebo + PX4 to fully launch
echo "   Waiting 30 seconds for Gazebo to be ready..."
sleep 30

# 3. Start the Demo Mission
# The mission will automatically wait for PX4 connection before flying!
echo "▶ [3/3] Starting Demo Mission (will auto-detect PX4 readiness)..."
gnome-terminal --tab --title="Demo Mission" -- bash -c "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && export FASTRTPS_DEFAULT_PROFILES_FILE=${SCRIPT_DIR}/fastdds_udp_only.xml && python3 ${SCRIPT_DIR}/demo_mission.py; exec bash"

echo ""
echo "✅ All systems launched!"
echo "   The mission will start flying once Gazebo is ready."
echo "   Watch the 'Demo Mission' terminal for progress."
echo ""
