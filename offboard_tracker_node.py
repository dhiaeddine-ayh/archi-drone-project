#!/usr/bin/env python3
"""
ROS 2 Offboard Tracker Node — Visual Servoing with PID Control
================================================================
Subscribes to thermal perception centroid data and controls the drone
to track and follow detected human heat signatures using PX4 offboard mode.

Mission State Machine:
  PREFLIGHT → TAKEOFF → SEARCH → TRACKING → LOST_TARGET → LAND → DONE
                                     ↕
                                LOST_TARGET

PID Controller:
  Converts 2D pixel error (normalized centroid offset from frame center)
  into 3D velocity setpoints for smooth visual servoing.

Topics Subscribed:
  /thermal_tracker/target_centroid  (geometry_msgs/Point)
  /thermal_tracker/target_detected  (std_msgs/Bool)
  /fmu/out/vehicle_status           (px4_msgs/VehicleStatus)

Topics Published:
  /fmu/in/offboard_control_mode     (px4_msgs/OffboardControlMode)
  /fmu/in/trajectory_setpoint       (px4_msgs/TrajectorySetpoint)
  /fmu/in/vehicle_command           (px4_msgs/VehicleCommand)

Author: Dhia — PX4 AI Tracker Project
"""

import rclpy
import math
from enum import Enum, auto
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import Bool

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)


# ═════════════════════════════════════════════════════════════════
#  PID Controller
# ═════════════════════════════════════════════════════════════════

class PIDController:
    """Simple PID controller with anti-windup and output clamping."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -2.0, output_max: float = 2.0,
                 integral_max: float = 5.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max

        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def compute(self, error: float, dt: float) -> float:
        """Compute PID output from error signal."""
        if dt <= 0:
            return 0.0

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup
        self.integral += error * dt
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))
        i_term = self.ki * self.integral

        # Derivative (skip on first call)
        if self.initialized:
            d_term = self.kd * (error - self.prev_error) / dt
        else:
            d_term = 0.0
            self.initialized = True

        self.prev_error = error

        # Sum and clamp
        output = p_term + i_term + d_term
        return max(self.output_min, min(self.output_max, output))

    def reset(self):
        """Reset all PID state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False


# ═════════════════════════════════════════════════════════════════
#  Mission State Machine
# ═════════════════════════════════════════════════════════════════

class MissionState(Enum):
    """Mission state machine states."""
    PREFLIGHT = auto()       # Send heartbeats, arm, switch to offboard
    TAKEOFF = auto()         # Climb to cruise altitude
    SEARCH = auto()          # Sweep/yaw to find thermal targets
    TRACKING = auto()        # Active visual servoing on detected target
    LOST_TARGET = auto()     # Target lost — hover and wait for re-acquisition
    LAND = auto()            # Send land command
    DONE = auto()            # Mission complete


# ═════════════════════════════════════════════════════════════════
#  Offboard Tracker Node
# ═════════════════════════════════════════════════════════════════

class OffboardTrackerNode(Node):
    """
    Full mission controller: takes off, searches for thermal targets,
    tracks them with PID visual servoing, handles target loss, and lands.
    """

    def __init__(self):
        super().__init__('offboard_tracker_node')

        # ─── Parameters ───────────────────────────────────────────
        self.declare_parameter('cruise_altitude', -6.0)      # NED: negative = up
        self.declare_parameter('max_velocity', 2.0)           # m/s horizontal cap
        self.declare_parameter('search_yaw_rate', 0.3)        # rad/s during search
        self.declare_parameter('search_forward_speed', 0.5)   # m/s forward during search
        self.declare_parameter('search_timeout', 120.0)       # seconds before giving up
        self.declare_parameter('lost_target_timeout', 10.0)   # seconds before returning to search
        self.declare_parameter('tracking_timeout', 300.0)     # max tracking time (5 min)
        self.declare_parameter('pid_kp', 1.5)
        self.declare_parameter('pid_ki', 0.02)
        self.declare_parameter('pid_kd', 0.3)

        self.CRUISE_ALT = self.get_parameter('cruise_altitude').value
        self.MAX_VEL = self.get_parameter('max_velocity').value
        self.SEARCH_YAW_RATE = self.get_parameter('search_yaw_rate').value
        self.SEARCH_FWD_SPEED = self.get_parameter('search_forward_speed').value
        self.SEARCH_TIMEOUT = self.get_parameter('search_timeout').value
        self.LOST_TIMEOUT = self.get_parameter('lost_target_timeout').value
        self.TRACK_TIMEOUT = self.get_parameter('tracking_timeout').value

        kp = self.get_parameter('pid_kp').value
        ki = self.get_parameter('pid_ki').value
        kd = self.get_parameter('pid_kd').value

        # ─── PID Controllers ──────────────────────────────────────
        # X-axis PID: centroid_x error → vy (lateral velocity)
        # Y-axis PID: centroid_y error → vx (forward velocity)
        # Note: camera X-error maps to drone Y-velocity, camera Y-error maps to drone X-velocity
        self.pid_vx = PIDController(kp, ki, kd, -self.MAX_VEL, self.MAX_VEL)
        self.pid_vy = PIDController(kp, ki, kd, -self.MAX_VEL, self.MAX_VEL)

        # ─── QoS Profile ──────────────────────────────────────────
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_default = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ─── PX4 Publishers ───────────────────────────────────────
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # ─── PX4 Subscribers ──────────────────────────────────────
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_cb, qos_px4)

        # ─── Thermal Tracker Subscribers ───────────────────────────
        self.create_subscription(
            Point, '/thermal_tracker/target_centroid',
            self.centroid_cb, qos_default)
        self.create_subscription(
            Bool, '/thermal_tracker/target_detected',
            self.detected_cb, qos_default)

        # ─── Vehicle State ─────────────────────────────────────────
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        # ─── Perception State ──────────────────────────────────────
        self.target_detected = False
        self.centroid_x = 0.0  # Normalized [-1, 1]
        self.centroid_y = 0.0
        self.last_detection_time = 0.0

        # ─── Mission State ─────────────────────────────────────────
        self.state = MissionState.PREFLIGHT
        self.phase_time = 0.0
        self.total_time = 0.0
        self.preflight_counter = 0
        self.search_yaw = 0.0
        self.tracking_time = 0.0

        # ─── Timer ─────────────────────────────────────────────────
        self.dt = 0.05  # 20 Hz
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info('╔══════════════════════════════════════════════╗')
        self.get_logger().info('║  🚁 OFFBOARD THERMAL TRACKER STARTED        ║')
        self.get_logger().info(f'║  Cruise alt: {-self.CRUISE_ALT:.0f}m  |  Max vel: {self.MAX_VEL:.1f} m/s    ║')
        self.get_logger().info(f'║  PID: Kp={kp}  Ki={ki}  Kd={kd}          ║')
        self.get_logger().info('╚══════════════════════════════════════════════╝')

    # ──────────────────────────────────────────────────────────────
    #  Subscriber Callbacks
    # ──────────────────────────────────────────────────────────────

    def vehicle_status_cb(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def centroid_cb(self, msg: Point):
        self.centroid_x = msg.x
        self.centroid_y = msg.y

    def detected_cb(self, msg: Bool):
        self.target_detected = msg.data
        if msg.data:
            self.last_detection_time = self.total_time

    # ──────────────────────────────────────────────────────────────
    #  PX4 Command Publishers
    # ──────────────────────────────────────────────────────────────

    def publish_offboard_mode(self, position=False, velocity=False):
        """Publish offboard control mode heartbeat."""
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_position(self, x: float, y: float, z: float, yaw: float = float('nan')):
        """Publish position setpoint (NED frame)."""
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def publish_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """Publish velocity setpoint (NED frame)."""
        msg = TrajectorySetpoint()
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.yawspeed = float(yaw_rate)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0):
        """Send a vehicle command to PX4."""
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
    #  State Machine Utilities
    # ──────────────────────────────────────────────────────────────

    def switch_state(self, new_state: MissionState):
        """Transition to a new mission state."""
        self.get_logger().info(
            f'━━━ State: {self.state.name} ➜ {new_state.name} '
            f'(T={self.total_time:.1f}s) ━━━'
        )
        self.state = new_state
        self.phase_time = 0.0

        # Reset PIDs on state change
        if new_state in (MissionState.SEARCH, MissionState.LOST_TARGET):
            self.pid_vx.reset()
            self.pid_vy.reset()

    # ──────────────────────────────────────────────────────────────
    #  Main Timer Callback — State Machine
    # ──────────────────────────────────────────────────────────────

    def timer_callback(self):
        self.phase_time += self.dt
        self.total_time += self.dt

        # ── STATE: PREFLIGHT ─────────────────────────────────────
        if self.state == MissionState.PREFLIGHT:
            self.publish_offboard_mode(position=True)
            self.publish_position(0.0, 0.0, self.CRUISE_ALT)
            self.preflight_counter += 1

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
                elif self.preflight_counter % 20 == 0:
                    self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        # ── STATE: TAKEOFF ───────────────────────────────────────
        elif self.state == MissionState.TAKEOFF:
            self.publish_offboard_mode(position=True)
            self.publish_position(0.0, 0.0, self.CRUISE_ALT)

            # Check if at altitude (within 1m)
            if self.phase_time > 5.0:
                self.get_logger().info(
                    f'🎯 At cruise altitude ({-self.CRUISE_ALT:.0f}m)! '
                    f'Entering search mode...'
                )
                self.switch_state(MissionState.SEARCH)

        # ── STATE: SEARCH ────────────────────────────────────────
        elif self.state == MissionState.SEARCH:
            self.publish_offboard_mode(velocity=True)

            # Slow yaw rotation + forward movement to scan area
            self.search_yaw += self.SEARCH_YAW_RATE * self.dt
            vx = self.SEARCH_FWD_SPEED * math.cos(self.search_yaw)
            vy = self.SEARCH_FWD_SPEED * math.sin(self.search_yaw)

            self.publish_velocity(vx, vy, 0.0, self.SEARCH_YAW_RATE)

            # Check for target detection
            if self.target_detected:
                self.get_logger().info(
                    '🔥 THERMAL TARGET ACQUIRED! '
                    'Switching to active tracking...'
                )
                self.tracking_time = 0.0
                self.switch_state(MissionState.TRACKING)

            # Search timeout
            elif self.phase_time > self.SEARCH_TIMEOUT:
                self.get_logger().warn(
                    f'⏰ Search timeout ({self.SEARCH_TIMEOUT:.0f}s). Landing...'
                )
                self.switch_state(MissionState.LAND)

            # Periodic status
            elif int(self.phase_time) % 10 == 0 and abs(self.phase_time % 1.0) < self.dt * 1.5:
                self.get_logger().info(
                    f'🔍 Searching... {self.phase_time:.0f}s / {self.SEARCH_TIMEOUT:.0f}s'
                )

        # ── STATE: TRACKING (Visual Servoing) ────────────────────
        elif self.state == MissionState.TRACKING:
            self.publish_offboard_mode(velocity=True)
            self.tracking_time += self.dt

            if self.target_detected:
                self.last_detection_time = self.total_time

                # ─── PID Visual Servoing ───────────────────────────
                # centroid_x > 0 means target is right of center → move right (+vy in NED)
                # centroid_y > 0 means target is below center → move forward (+vx in NED)
                # Note: for downward cam, "below center" = "ahead of drone" in body frame
                error_x = self.centroid_x  # Lateral error
                error_y = self.centroid_y  # Longitudinal error

                vy = self.pid_vy.compute(error_x, self.dt)   # Lateral velocity
                vx = self.pid_vx.compute(error_y, self.dt)   # Forward velocity
                vz = 0.0  # Hold altitude

                # Yaw towards movement direction for smoother tracking
                yaw_rate = error_x * 0.5

                self.publish_velocity(vx, vy, vz, yaw_rate)

                # Periodic tracking status
                if int(self.tracking_time) % 5 == 0 and abs(self.tracking_time % 1.0) < self.dt * 1.5:
                    self.get_logger().info(
                        f'🎯 Tracking | err=({error_x:+.2f},{error_y:+.2f}) '
                        f'| vel=({vx:+.2f},{vy:+.2f}) '
                        f'| t={self.tracking_time:.0f}s'
                    )

            else:
                # Target lost — check timeout
                time_since_detection = self.total_time - self.last_detection_time
                if time_since_detection > 3.0:
                    self.get_logger().warn(
                        f'⚠️ Target lost for {time_since_detection:.1f}s! '
                        f'Entering failsafe hover...'
                    )
                    self.switch_state(MissionState.LOST_TARGET)
                else:
                    # Brief loss — hold last velocity at reduced speed
                    self.publish_velocity(0.0, 0.0, 0.0, 0.0)

            # Tracking timeout
            if self.tracking_time > self.TRACK_TIMEOUT:
                self.get_logger().info(
                    f'⏰ Tracking timeout ({self.TRACK_TIMEOUT:.0f}s). '
                    f'Mission complete. Landing...'
                )
                self.switch_state(MissionState.LAND)

        # ── STATE: LOST TARGET (Failsafe Hover) ─────────────────
        elif self.state == MissionState.LOST_TARGET:
            self.publish_offboard_mode(velocity=True)
            # Hover in place, slow yaw to scan
            self.publish_velocity(0.0, 0.0, 0.0, self.SEARCH_YAW_RATE * 0.5)

            # Check for re-acquisition
            if self.target_detected:
                self.get_logger().info(
                    '🔥 Target re-acquired! Resuming tracking...'
                )
                self.tracking_time = 0.0
                self.switch_state(MissionState.TRACKING)

            # Timeout — return to full search
            elif self.phase_time > self.LOST_TIMEOUT:
                self.get_logger().warn(
                    f'⏰ Lost target timeout ({self.LOST_TIMEOUT:.0f}s). '
                    f'Returning to search pattern...'
                )
                self.switch_state(MissionState.SEARCH)

            # Status
            elif int(self.phase_time) % 3 == 0 and abs(self.phase_time % 1.0) < self.dt * 1.5:
                self.get_logger().info(
                    f'🔍 Hovering, scanning for target... '
                    f'{self.phase_time:.0f}s / {self.LOST_TIMEOUT:.0f}s'
                )

        # ── STATE: LAND ──────────────────────────────────────────
        elif self.state == MissionState.LAND:
            self.publish_offboard_mode(position=True)
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

            if self.phase_time > 2.0:
                self.switch_state(MissionState.DONE)

        # ── STATE: DONE ──────────────────────────────────────────
        elif self.state == MissionState.DONE:
            if self.phase_time < 1.0:
                self.get_logger().info('╔══════════════════════════════════════════════╗')
                self.get_logger().info(f'║  ✅ THERMAL TRACKING MISSION COMPLETE        ║')
                self.get_logger().info(f'║  Total time: {self.total_time:.1f}s                         ║')
                self.get_logger().info(f'║  States: SEARCH ➜ TRACK ➜ LAND              ║')
                self.get_logger().info('╚══════════════════════════════════════════════╝')


def main(args=None):
    print()
    print('═══════════════════════════════════════════════')
    print('  🚁  OFFBOARD THERMAL TRACKER')
    print('  PID Visual Servoing Mission Controller')
    print('  Waiting for PX4 connection...')
    print('═══════════════════════════════════════════════')
    print()

    rclpy.init(args=args)
    node = OffboardTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Tracker node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
