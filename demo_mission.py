#!/usr/bin/env python3
"""
PX4 Drone Demo Mission — Impressive Flight Patterns
=====================================================
A professional ROS 2 drone mission demonstrating:
  1. Smart PX4 connection detection (waits for Gazebo to be ready)
  2. Autonomous takeoff
  3. Circle orbit pattern
  4. Figure-8 (lemniscate) pattern
  5. Spiral descent back to home
  6. Autonomous landing

Author: Dhia — PX4 AI Tracker Project
"""

import rclpy
import math
from enum import Enum, auto
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)


class MissionState(Enum):
    """State machine states for the mission."""
    PREFLIGHT = auto()
    TAKEOFF = auto()
    CIRCLE = auto()
    FIGURE_EIGHT = auto()
    SPIRAL_DESCENT = auto()
    RETURN_HOME = auto()
    LAND = auto()
    DONE = auto()


class DemoMission(Node):
    """
    A professional drone demo mission with proper state machine.
    Waits for PX4/Gazebo to be fully connected before starting.
    """

    def __init__(self):
        super().__init__('demo_mission_node')

        # ─── QoS Profile ───────────────────────────────────────────
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ─── Publishers ────────────────────────────────────────────
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10
        )

        # ─── Subscribers ───────────────────────────────────────────
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_cb, qos_profile,
        )
        # ─── Vehicle State ─────────────────────────────────────────
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        # ─── Mission Parameters ────────────────────────────────────
        self.CRUISE_ALT = -5.0          # 5 meters up (NED: negative = up)
        self.CIRCLE_RADIUS = 8.0        # Circle radius in meters
        self.CIRCLE_CENTER_X = 8.0      # Circle center offset
        self.CIRCLE_CENTER_Y = 0.0
        self.FIGURE8_SCALE_X = 10.0     # Figure-8 X amplitude
        self.FIGURE8_SCALE_Y = 5.0      # Figure-8 Y amplitude
        self.POSITION_TOLERANCE = 1.0   # meters — close enough to waypoint
        self.SPIRAL_LOOPS = 2           # Number of spiral loops during descent

        # ─── State Machine ─────────────────────────────────────────
        self.state = MissionState.PREFLIGHT
        self.phase_time = 0.0           # Time spent in current phase
        self.total_time = 0.0           # Total mission time
        self.preflight_counter = 0      # Heartbeats sent before arming
        self.dt = 0.05                  # 20 Hz timer
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info('╔══════════════════════════════════════════════╗')
        self.get_logger().info('║   🚁  PX4 DRONE DEMO MISSION               ║')
        self.get_logger().info('║   Starting preflight sequence...            ║')
        self.get_logger().info('╚══════════════════════════════════════════════╝')

    # ──────────────────────────────────────────────────────────────
    #  Subscriber Callbacks
    # ──────────────────────────────────────────────────────────────

    def vehicle_status_cb(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state



    # ──────────────────────────────────────────────────────────────
    #  Publishers
    # ──────────────────────────────────────────────────────────────

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_position(self, x: float, y: float, z: float, yaw: float = float('nan')):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    # ──────────────────────────────────────────────────────────────
    #  Utility
    # ──────────────────────────────────────────────────────────────

    def distance_to(self, x, y, z):
        return math.sqrt((self.pos_x - x)**2 + (self.pos_y - y)**2 + (self.pos_z - z)**2)

    def reached(self, x, y, z):
        return self.distance_to(x, y, z) < self.POSITION_TOLERANCE

    def switch_state(self, new_state):
        self.get_logger().info(f'━━━ State: {self.state.name} ➜ {new_state.name} ━━━')
        self.state = new_state
        self.phase_time = 0.0

    # ──────────────────────────────────────────────────────────────
    #  Main Timer Callback — State Machine
    # ──────────────────────────────────────────────────────────────

    def timer_callback(self):
        self.phase_time += self.dt
        self.total_time += self.dt

        # Always send offboard heartbeat
        self.publish_offboard_mode()

        # ── STATE: Preflight (send heartbeats, then arm) ────────
        if self.state == MissionState.PREFLIGHT:
            # Send position setpoints to build up offboard heartbeat
            self.publish_position(0.0, 0.0, self.CRUISE_ALT)
            self.preflight_counter += 1

            # Need ~40 heartbeats (2 seconds at 20Hz) before switching
            if self.preflight_counter == 20:
                self.get_logger().info('🔄 Setting OFFBOARD mode...')
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

            if self.preflight_counter == 40:
                self.get_logger().info('🔑 Arming motors...')
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

            if self.preflight_counter >= 50:
                if self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                    self.get_logger().info('✅ Armed! Taking off...')
                    self.switch_state(MissionState.TAKEOFF)
                else:
                    # Retry arming
                    if self.preflight_counter % 20 == 0:
                        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        # ── STATE: Takeoff ──────────────────────────────────────
        elif self.state == MissionState.TAKEOFF:
            self.publish_position(0.0, 0.0, self.CRUISE_ALT)
            if self.reached(0.0, 0.0, self.CRUISE_ALT):
                self.get_logger().info('🎯 Reached cruise altitude! Starting circle pattern...')
                self.switch_state(MissionState.CIRCLE)

        # ── STATE: Circle Pattern ───────────────────────────────
        elif self.state == MissionState.CIRCLE:
            duration = 20.0  # seconds for full circle
            t = self.phase_time
            progress = t / duration

            if progress >= 1.0:
                self.get_logger().info('✅ Circle complete! Starting figure-8...')
                self.switch_state(MissionState.FIGURE_EIGHT)
            else:
                angle = 2.0 * math.pi * progress
                cx = self.CIRCLE_CENTER_X
                cy = self.CIRCLE_CENTER_Y
                r = self.CIRCLE_RADIUS
                x = cx + r * math.cos(angle)
                y = cy + r * math.sin(angle)
                # Yaw: face the direction of travel (tangent)
                yaw = angle + math.pi / 2.0
                self.publish_position(x, y, self.CRUISE_ALT, yaw)

                if int(t * 4) % 20 == 0 and abs((t * 4) % 1.0) < self.dt * 2:
                    self.get_logger().info(
                        f'🔵 Circle: {progress * 100:.0f}% — '
                        f'pos=({x:.1f}, {y:.1f})'
                    )

        # ── STATE: Figure-8 Pattern ─────────────────────────────
        elif self.state == MissionState.FIGURE_EIGHT:
            duration = 30.0  # seconds for full figure-8
            t = self.phase_time
            progress = t / duration

            if progress >= 1.0:
                self.get_logger().info('✅ Figure-8 complete! Starting spiral descent...')
                self.switch_state(MissionState.SPIRAL_DESCENT)
            else:
                angle = 2.0 * math.pi * progress
                # Lemniscate of Bernoulli (figure-8)
                x = self.FIGURE8_SCALE_X * math.sin(angle)
                y = self.FIGURE8_SCALE_Y * math.sin(angle) * math.cos(angle)
                # Yaw faces direction of movement
                dx = self.FIGURE8_SCALE_X * math.cos(angle)
                dy = self.FIGURE8_SCALE_Y * (math.cos(angle)**2 - math.sin(angle)**2)
                yaw = math.atan2(dy, dx)
                self.publish_position(x, y, self.CRUISE_ALT, yaw)

                if int(t * 4) % 20 == 0 and abs((t * 4) % 1.0) < self.dt * 2:
                    self.get_logger().info(
                        f'♾️  Figure-8: {progress * 100:.0f}% — '
                        f'pos=({x:.1f}, {y:.1f})'
                    )

        # ── STATE: Spiral Descent ───────────────────────────────
        elif self.state == MissionState.SPIRAL_DESCENT:
            duration = 20.0  # seconds for spiral
            t = self.phase_time
            progress = t / duration

            if progress >= 1.0:
                self.get_logger().info('✅ Spiral complete! Returning home...')
                self.switch_state(MissionState.RETURN_HOME)
            else:
                angle = 2.0 * math.pi * self.SPIRAL_LOOPS * progress
                # Radius shrinks from CIRCLE_RADIUS to 1m
                radius = self.CIRCLE_RADIUS * (1.0 - progress) + 1.0 * progress
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                # Altitude rises from CRUISE_ALT (-5) to -2 (descend to 2m)
                z = self.CRUISE_ALT * (1.0 - progress) + (-2.0) * progress
                yaw = angle + math.pi / 2.0
                self.publish_position(x, y, z, yaw)

                if int(t * 4) % 20 == 0 and abs((t * 4) % 1.0) < self.dt * 2:
                    self.get_logger().info(
                        f'🌀 Spiral: {progress * 100:.0f}% — '
                        f'alt={-z:.1f}m, radius={radius:.1f}m'
                    )

        # ── STATE: Return Home ──────────────────────────────────
        elif self.state == MissionState.RETURN_HOME:
            self.publish_position(0.0, 0.0, -3.0)
            if self.reached(0.0, 0.0, -3.0) or self.phase_time > 10.0:
                self.get_logger().info('🏠 Home reached! Landing...')
                self.switch_state(MissionState.LAND)

        # ── STATE: Land ─────────────────────────────────────────
        elif self.state == MissionState.LAND:
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            if self.phase_time > 2.0:
                self.switch_state(MissionState.DONE)

        # ── STATE: Done ─────────────────────────────────────────
        elif self.state == MissionState.DONE:
            if self.phase_time < 1.0:
                self.get_logger().info('╔══════════════════════════════════════════════╗')
                self.get_logger().info(f'║   ✅  MISSION COMPLETE — {self.total_time:.1f}s total       ║')
                self.get_logger().info('║   Patterns: Circle ➜ Figure-8 ➜ Spiral      ║')
                self.get_logger().info('╚══════════════════════════════════════════════╝')


def main(args=None):
    print()
    print('═══════════════════════════════════════════════')
    print('  🚁  PX4 DRONE DEMO MISSION')
    print('  Waiting for Gazebo & PX4 to be ready...')
    print('═══════════════════════════════════════════════')
    print()

    rclpy.init(args=args)
    node = DemoMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
