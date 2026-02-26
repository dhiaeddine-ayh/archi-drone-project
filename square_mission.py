#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus
import math

class SquareMission(Node):
    def __init__(self):
        super().__init__('square_mission_node')

        # QoS Setup
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Subscribers
        self.vehicle_status_subscriber_ = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Variables
        self.vehicle_nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.timer_period = 0.1  # 10Hz (0.1 seconds)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Mission Logic Variables
        self.time_elapsed = 0.0
        self.takeoff_height = -5.0

    def vehicle_status_callback(self, msg):
        self.vehicle_nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_position(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

    def timer_callback(self):
        self.publish_offboard_control_mode()
        
        # Arming & Offboard Logic (First 5 seconds)
        if self.time_elapsed < 5.0:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.publish_position(0, 0, self.takeoff_height)

        # --- بداية الحركة (Waypoints) ---
        
        # 1. للأمام (X=5)
        elif 5.0 <= self.time_elapsed < 10.0:
            self.publish_position(20, 0, self.takeoff_height)
            self.get_logger().info("Moving Forward (North)...")

        # 2. لليمين (X=5, Y=5)
        elif 10.0 <= self.time_elapsed < 15.0:
            self.publish_position(5, 12, self.takeoff_height)
            self.get_logger().info("Moving Right (East)...")

        # 3. للوراء (X=0, Y=5)
        elif 15.0 <= self.time_elapsed < 20.0:
            self.publish_position(0, 5, self.takeoff_height)
            self.get_logger().info("Moving Backward...")

        # 4. غلق المربع (X=0, Y=0)
        elif 20.0 <= self.time_elapsed < 25.0:
            self.publish_position(0, 0, self.takeoff_height)
            self.get_logger().info("Going Home...")
        
        # 5. الهبوط (Land)
        elif self.time_elapsed >= 25.0:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info("Landing...")
            # نتوقف عن إرسال Offboard Mode باش نخلوها تهبط براحتها

        self.time_elapsed += self.timer_period

def main(args=None):
    rclpy.init(args=args)
    node = SquareMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
