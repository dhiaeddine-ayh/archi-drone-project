#!/usr/bin/env python3
"""
road_follower_node.py — ROS 2 Road-Following Offboard Controller (VTOL)

State machine: PREFLIGHT → TAKEOFF → FOLLOW_ROAD → HOVER → LAND → DONE

Uses POSITION-based offboard control for reliable flight.
Subscribes to road detection and adjusts position setpoints to follow roads.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_msgs.msg import String
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)
import math
from enum import Enum


class MissionState(Enum):
    WAITING = 0
    PREFLIGHT = 1
    TAKEOFF = 2
    FOLLOW_ROAD = 3
    CIRCLE_AREA = 4
    SQUARE_PATROL = 5
    FIGURE8 = 6
    GRID_SCAN = 7
    RTH = 8
    FOLLOW_CAR = 9
    PATROL = 10
    HOVER = 11
    LAND = 12
    DONE = 13


class RoadFollowerNode(Node):
    def __init__(self):
        super().__init__('road_follower_node')

        # ─── Parameters ───
        self.declare_parameter('cruise_altitude', 15.0)     # meters AGL (positive = up)
        self.declare_parameter('forward_speed', 3.0)         # m/s along road
        self.declare_parameter('lateral_gain', 3.0)          # lateral correction gain
        self.declare_parameter('heading_gain', 0.8)          # yaw correction gain
        self.declare_parameter('lost_timeout', 10.0)          # seconds before HOVER
        self.declare_parameter('hover_timeout', 15.0)        # seconds before LAND
        self.declare_parameter('mission_timeout', 300.0)     # 5 minutes

        self.cruise_alt = self.get_parameter('cruise_altitude').value
        self.fwd_speed = self.get_parameter('forward_speed').value
        self.lat_gain = self.get_parameter('lateral_gain').value
        self.heading_gain = self.get_parameter('heading_gain').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.hover_timeout = self.get_parameter('hover_timeout').value
        self.mission_timeout = self.get_parameter('mission_timeout').value

        # ─── QoS for PX4 ───
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # ─── Subscribers ───
        self.road_offset_sub = self.create_subscription(
            Point, '/road_tracker/road_offset', self.road_offset_cb, 10)
        self.road_detected_sub = self.create_subscription(
            Bool, '/road_tracker/road_detected', self.road_detected_cb, 10)
        self.mission_cmd_sub = self.create_subscription(
            String, '/mission/command', self.mission_cmd_cb, 10)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_cb, qos)
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_pos_cb, qos)

        # ─── Publishers ───
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # ─── State ───
        self.state = MissionState.WAITING
        self.road_offset_x = 0.0      # lateral offset [-1, 1]
        self.road_angle = 0.0          # heading angle (radians)
        self.road_detected = False
        self.nav_state = 0
        self.arming_state = 0

        # Position tracking
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.heading = 0.0

        # Target position (updated continuously)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0

        self.offboard_setpoint_count = 0
        self.mission_start_time = None
        self.lost_start_time = None
        self.hover_start_time = None
        self.takeoff_start_time = None

        # Mission-specific state
        self.requested_mission = 'FOLLOW_ROAD'  # default after takeoff
        self.circle_angle = 0.0
        self.circle_radius = 20.0
        self.circle_center_x = 0.0
        self.circle_center_y = 0.0
        self.square_waypoints = []
        self.square_wp_index = 0
        self.square_laps = 0
        self.fig8_angle = 0.0
        self.fig8_radius = 15.0
        self.grid_waypoints = []
        self.grid_wp_index = 0
        self.home_x = 0.0
        self.home_y = 0.0
        self.car_waypoints = []
        self.car_wp_index = 0
        self.patrol_waypoints = []
        self.patrol_wp_index = 0
        self.patrol_laps = 0

        # Timer at 20Hz
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info(' Road Follower Node (VTOL) — Started')
        self.get_logger().info(f'  Altitude: {self.cruise_alt:.0f}m')
        self.get_logger().info(f'  Speed: {self.fwd_speed:.1f} m/s')
        self.get_logger().info('')
        self.get_logger().info(' ⏳ Waiting for START command...')
        self.get_logger().info('    (Press S in the Mission Control window)')
        self.get_logger().info('═══════════════════════════════════════')

    # ─── Callbacks ───

    def road_offset_cb(self, msg):
        self.road_offset_x = msg.x
        self.road_angle = msg.y

    def road_detected_cb(self, msg):
        self.road_detected = msg.data

    def vehicle_status_cb(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def mission_cmd_cb(self, msg):
        cmd = msg.data.upper().strip()
        mission_map = {
            'START': 'FOLLOW_ROAD', 'CIRCLE': 'CIRCLE_AREA',
            'SQUARE': 'SQUARE_PATROL', 'HOVER': 'HOVER',
            'FIGURE8': 'FIGURE8', 'GRID': 'GRID_SCAN', 'RTH': 'RTH',
            'FOLLOWCAR': 'FOLLOW_CAR', 'PATROL': 'PATROL',
        }
        if cmd in mission_map:
            self.requested_mission = mission_map[cmd]
            active = (MissionState.FOLLOW_ROAD, MissionState.CIRCLE_AREA,
                      MissionState.SQUARE_PATROL, MissionState.FIGURE8,
                      MissionState.GRID_SCAN, MissionState.RTH,
                      MissionState.FOLLOW_CAR, MissionState.PATROL,
                      MissionState.HOVER)

            if self.state == MissionState.WAITING:
                self.offboard_setpoint_count = 0
                self.home_x = self.pos_x
                self.home_y = self.pos_y
                self.state = MissionState.PREFLIGHT
                self.get_logger().info(f'✅ {cmd} → {self.requested_mission}')
            elif self.state in active:
                self._enter_mission()
                self.get_logger().info(f'🔄 Switch: {self.requested_mission}')

        elif cmd == 'END' and self.state not in (MissionState.WAITING, MissionState.DONE):
            self.get_logger().info('⏹ END → landing!')
            self.state = MissionState.LAND

    def local_pos_cb(self, msg):
        self.pos_x = msg.x
        self.pos_y = msg.y
        self.pos_z = msg.z
        self.heading = msg.heading

    # ─── Commands ───

    def publish_offboard_heartbeat(self):
        """Must be called at >2Hz for offboard mode to stay active."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_position_setpoint(self, x, y, z, yaw=float('nan')):
        """Publish a NED position setpoint. z is negative = up."""
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param7 = float(param7)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def arm(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('✈  ARM command sent')

    def set_offboard_mode(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info('✈  OFFBOARD mode command sent')

    def land_command(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('✈  LAND command sent')

    # ─── Main Loop ───

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # ═══════════════════════════════════════
        # WAITING: Do NOTHING — let the GUI control the drone
        # ═══════════════════════════════════════
        if self.state == MissionState.WAITING:
            return

        # Only publish offboard heartbeat when this node is active
        self.publish_offboard_heartbeat()

        # ═══════════════════════════════════════
        # PREFLIGHT: Send setpoints then arm
        # ═══════════════════════════════════════
        if self.state == MissionState.PREFLIGHT:
            # PX4 requires setpoints before switching to offboard.
            # Send the current position as setpoint (hold position).
            self.publish_position_setpoint(
                self.pos_x, self.pos_y, -self.cruise_alt, self.heading)

            self.offboard_setpoint_count += 1

            if self.offboard_setpoint_count == 10:
                self.get_logger().info('Sending OFFBOARD mode...')
                self.set_offboard_mode()

            if self.offboard_setpoint_count == 20:
                self.get_logger().info('Sending ARM...')
                self.arm()

            if self.offboard_setpoint_count >= 30:
                # Set takeoff target
                self.target_x = self.pos_x
                self.target_y = self.pos_y
                self.target_z = -self.cruise_alt  # NED: negative = up
                self.target_yaw = 0.0

                self.state = MissionState.TAKEOFF
                self.takeoff_start_time = now
                self.mission_start_time = now
                self.get_logger().info(
                    f'→ TAKEOFF (target alt: {self.cruise_alt:.0f}m)')

        # ═══════════════════════════════════════
        # TAKEOFF: Climb to cruise altitude
        # ═══════════════════════════════════════
        elif self.state == MissionState.TAKEOFF:
            self.publish_position_setpoint(
                self.target_x, self.target_y, self.target_z, self.target_yaw)

            current_alt = -self.pos_z
            at_altitude = current_alt > (self.cruise_alt - 2.0)
            time_elapsed = now - self.takeoff_start_time > 10.0

            if at_altitude or time_elapsed:
                self._enter_mission()
                self.get_logger().info(
                    f'→ {self.state.name} (alt: {current_alt:.1f}m)')

        # ═══════════════════════════════════════
        # FOLLOW_ROAD
        # ═══════════════════════════════════════
        elif self.state == MissionState.FOLLOW_ROAD:
            if self.road_detected:
                self.lost_start_time = None
                self.target_x += self.fwd_speed * self.dt * math.cos(self.target_yaw)
                self.target_y += self.fwd_speed * self.dt * math.sin(self.target_yaw)
                lateral = self.road_offset_x * self.lat_gain * self.dt
                self.target_x += lateral * math.cos(self.target_yaw + math.pi/2)
                self.target_y += lateral * math.sin(self.target_yaw + math.pi/2)
                self.target_yaw += self.road_angle * self.heading_gain * self.dt
                self.target_z = -self.cruise_alt
            else:
                if self.lost_start_time is None:
                    self.lost_start_time = now
                    self.get_logger().warn('Road lost...')
                self.target_x += 1.0 * self.dt * math.cos(self.target_yaw)
                self.target_y += 1.0 * self.dt * math.sin(self.target_yaw)
                if now - self.lost_start_time > self.lost_timeout:
                    self.state = MissionState.HOVER
                    self.hover_start_time = now
                    self.get_logger().info('→ HOVER (road lost)')

            self.publish_position_setpoint(
                self.target_x, self.target_y, self.target_z, self.target_yaw)

        # ═══════════════════════════════════════
        # CIRCLE_AREA: Orbit a point
        # ═══════════════════════════════════════
        elif self.state == MissionState.CIRCLE_AREA:
            self.circle_angle += 0.3 * self.dt  # ~0.3 rad/s
            self.target_x = self.circle_center_x + self.circle_radius * math.cos(self.circle_angle)
            self.target_y = self.circle_center_y + self.circle_radius * math.sin(self.circle_angle)
            self.target_z = -self.cruise_alt
            # Face center of circle
            self.target_yaw = self.circle_angle + math.pi

            self.publish_position_setpoint(
                self.target_x, self.target_y, self.target_z, self.target_yaw)

        # ═══════════════════════════════════════
        # SQUARE_PATROL: Fly a square pattern
        # ═══════════════════════════════════════
        elif self.state == MissionState.SQUARE_PATROL:
            if len(self.square_waypoints) > 0:
                wp = self.square_waypoints[self.square_wp_index]
                self.target_x = wp[0]
                self.target_y = wp[1]
                self.target_z = -self.cruise_alt

                # Face direction of travel
                dx = wp[0] - self.pos_x
                dy = wp[1] - self.pos_y
                if abs(dx) > 0.5 or abs(dy) > 0.5:
                    self.target_yaw = math.atan2(dy, dx)

                self.publish_position_setpoint(
                    self.target_x, self.target_y, self.target_z, self.target_yaw)

                # Check if reached waypoint
                dist = math.sqrt((self.pos_x - wp[0])**2 +
                                 (self.pos_y - wp[1])**2)
                if dist < 3.0:
                    self.square_wp_index = (self.square_wp_index + 1) % len(self.square_waypoints)
                    if self.square_wp_index == 0:
                        self.square_laps += 1
                        self.get_logger().info(
                            f'Square lap {self.square_laps} complete')

        # ═══════════════════════════════════════
        # FIGURE8
        # ═══════════════════════════════════════
        elif self.state == MissionState.FIGURE8:
            self.fig8_angle += 0.25 * self.dt
            r = self.fig8_radius
            # Lemniscate (figure-8)
            self.target_x = self.circle_center_x + r * math.sin(self.fig8_angle)
            self.target_y = self.circle_center_y + r * math.sin(2 * self.fig8_angle) / 2
            self.target_z = -self.cruise_alt
            self.target_yaw = self.fig8_angle
            self.publish_position_setpoint(
                self.target_x, self.target_y, self.target_z, self.target_yaw)

        # ═══════════════════════════════════════
        # GRID_SCAN: Lawnmower pattern
        # ═══════════════════════════════════════
        elif self.state == MissionState.GRID_SCAN:
            if len(self.grid_waypoints) > 0:
                wp = self.grid_waypoints[self.grid_wp_index]
                self.target_x, self.target_y = wp
                self.target_z = -self.cruise_alt
                dx = wp[0] - self.pos_x
                dy = wp[1] - self.pos_y
                if abs(dx) > 0.5 or abs(dy) > 0.5:
                    self.target_yaw = math.atan2(dy, dx)
                self.publish_position_setpoint(
                    self.target_x, self.target_y, self.target_z, self.target_yaw)
                dist = math.sqrt((self.pos_x - wp[0])**2 + (self.pos_y - wp[1])**2)
                if dist < 3.0:
                    self.grid_wp_index += 1
                    if self.grid_wp_index >= len(self.grid_waypoints):
                        self.grid_wp_index = 0
                        self.get_logger().info('Grid scan complete, restarting')

        # ═══════════════════════════════════════
        # RTH: Return to home
        # ═══════════════════════════════════════
        elif self.state == MissionState.RTH:
            self.target_x = self.home_x
            self.target_y = self.home_y
            self.target_z = -self.cruise_alt
            self.target_yaw = math.atan2(self.home_y - self.pos_y, self.home_x - self.pos_x)
            self.publish_position_setpoint(
                self.target_x, self.target_y, self.target_z, self.target_yaw)
            dist = math.sqrt((self.pos_x - self.home_x)**2 + (self.pos_y - self.home_y)**2)
            if dist < 3.0:
                self.get_logger().info('Home reached → LAND')
                self.state = MissionState.LAND

        # ═══════════════════════════════════════
        # FOLLOW_CAR: Visit each car position
        # ═══════════════════════════════════════
        elif self.state == MissionState.FOLLOW_CAR:
            if len(self.car_waypoints) > 0:
                wp = self.car_waypoints[self.car_wp_index]
                self.target_x = wp[0]
                self.target_y = wp[1]
                self.target_z = -10.0  # lower altitude for car inspection
                dx = wp[0] - self.pos_x
                dy = wp[1] - self.pos_y
                if abs(dx) > 0.5 or abs(dy) > 0.5:
                    self.target_yaw = math.atan2(dy, dx)
                self.publish_position_setpoint(
                    self.target_x, self.target_y, self.target_z, self.target_yaw)
                dist = math.sqrt((self.pos_x - wp[0])**2 + (self.pos_y - wp[1])**2)
                if dist < 4.0:
                    self.car_wp_index += 1
                    if self.car_wp_index >= len(self.car_waypoints):
                        self.car_wp_index = 0
                        self.get_logger().info('All cars visited, restarting')
                    else:
                        self.get_logger().info(
                            f'Car {self.car_wp_index}/{len(self.car_waypoints)} reached')

        # ═══════════════════════════════════════
        # PATROL: Follow main road back and forth
        # ═══════════════════════════════════════
        elif self.state == MissionState.PATROL:
            if len(self.patrol_waypoints) > 0:
                wp = self.patrol_waypoints[self.patrol_wp_index]
                self.target_x = wp[0]
                self.target_y = wp[1]
                self.target_z = -self.cruise_alt
                dx = wp[0] - self.pos_x
                dy = wp[1] - self.pos_y
                if abs(dx) > 0.5 or abs(dy) > 0.5:
                    self.target_yaw = math.atan2(dy, dx)
                self.publish_position_setpoint(
                    self.target_x, self.target_y, self.target_z, self.target_yaw)
                dist = math.sqrt((self.pos_x - wp[0])**2 + (self.pos_y - wp[1])**2)
                if dist < 5.0:
                    self.patrol_wp_index += 1
                    if self.patrol_wp_index >= len(self.patrol_waypoints):
                        self.patrol_wp_index = 0
                        self.patrol_laps += 1
                        self.get_logger().info(f'Patrol lap {self.patrol_laps} complete')

        # ═══════════════════════════════════════
        # HOVER: Hold position
        # ═══════════════════════════════════════
        elif self.state == MissionState.HOVER:
            self.publish_position_setpoint(
                self.target_x, self.target_y, self.target_z, self.target_yaw)

        # ═══════════════════════════════════════
        # LAND
        # ═══════════════════════════════════════
        elif self.state == MissionState.LAND:
            self.land_command()
            self.state = MissionState.DONE
            self.get_logger().info('→ DONE — Landing')

        elif self.state == MissionState.DONE:
            pass

    def _enter_mission(self):
        """Transition from TAKEOFF to the requested mission."""
        m = self.requested_mission
        if m == 'FOLLOW_ROAD':
            self.state = MissionState.FOLLOW_ROAD
        elif m == 'CIRCLE_AREA':
            self.circle_center_x = self.pos_x
            self.circle_center_y = self.pos_y
            self.circle_angle = 0.0
            self.state = MissionState.CIRCLE_AREA
            self.get_logger().info(
                f'Circle: center=({self.circle_center_x:.0f},{self.circle_center_y:.0f}) r={self.circle_radius}m')
        elif m == 'SQUARE_PATROL':
            s = 20.0  # half-side length
            cx, cy = self.pos_x, self.pos_y
            self.square_waypoints = [
                (cx + s, cy + s),
                (cx - s, cy + s),
                (cx - s, cy - s),
                (cx + s, cy - s),
            ]
            self.square_wp_index = 0
            self.square_laps = 0
            self.state = MissionState.SQUARE_PATROL
            self.get_logger().info(f'Square: 40m x 40m around ({cx:.0f},{cy:.0f})')
        elif m == 'FIGURE8':
            self.circle_center_x = self.pos_x
            self.circle_center_y = self.pos_y
            self.fig8_angle = 0.0
            self.state = MissionState.FIGURE8
            self.get_logger().info(f'Figure-8: center=({self.pos_x:.0f},{self.pos_y:.0f}) r={self.fig8_radius}m')
        elif m == 'GRID_SCAN':
            s = 15.0
            cx, cy = self.pos_x, self.pos_y
            self.grid_waypoints = []
            for i in range(5):
                y_off = -2*s + i * s
                if i % 2 == 0:
                    self.grid_waypoints.append((cx - s, cy + y_off))
                    self.grid_waypoints.append((cx + s, cy + y_off))
                else:
                    self.grid_waypoints.append((cx + s, cy + y_off))
                    self.grid_waypoints.append((cx - s, cy + y_off))
            self.grid_wp_index = 0
            self.state = MissionState.GRID_SCAN
            self.get_logger().info(f'Grid scan: {len(self.grid_waypoints)} waypoints')
        elif m == 'RTH':
            self.state = MissionState.RTH
            self.get_logger().info(f'RTH: heading to ({self.home_x:.0f},{self.home_y:.0f})')
        elif m == 'HOVER':
            self.state = MissionState.HOVER
            self.get_logger().info('Hovering at altitude')
        elif m == 'FOLLOW_CAR':
            # Car positions from road_cars.world
            self.car_waypoints = [
                (-30, -2),    # car_red_1 on main road
                (-10, 2),     # car_blue_1
                (25, -2),     # suv_1
                (45, 2),      # car_white_1
                (80, -2),     # car_red_3
                (-2, -30),    # car_red_2 on cross road
                (2, 25),      # suv_2
                (-2, 50),     # car_blue_2
                (58, 20),     # car_cross2_1 on 2nd cross
                (62, -25),    # suv_cross2
                (-65, -20),   # truck_1 on diagonal
                (-85, -50),   # car_diag_1
            ]
            self.car_wp_index = 0
            self.state = MissionState.FOLLOW_CAR
            self.get_logger().info(f'Follow Car: {len(self.car_waypoints)} cars to visit')
        elif m == 'PATROL':
            # Patrol along the main E-W road
            self.patrol_waypoints = [
                (-90, 0), (-50, 0), (0, 0), (50, 0), (90, 0),
                (50, 0), (0, 0), (-50, 0),
            ]
            self.patrol_wp_index = 0
            self.patrol_laps = 0
            self.state = MissionState.PATROL
            self.get_logger().info('Road Patrol: E-W main road')


def main(args=None):
    rclpy.init(args=args)
    node = RoadFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
