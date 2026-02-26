#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus
import time

class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control_node')

        # إعدادات الجودة (ضرورية باش يفهمونا)
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

        self.vehicle_nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.counter = 0

        # Timer (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

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

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        # Z = -5.0 (يعني 5 أمتار للأعلى، لأن Z مقلوب)
        msg.position = [0.0, 0.0, -5.0]
        msg.yaw = 0.0 
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
        # 1. ديما نبعثو "راني هنا" (Heartbeat)
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # 2. منطق الإقلاع (بعد ثانية وحدة)
        if self.counter == 10:
            # التحويل لوضع Offboard
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info("SETTING OFFBOARD MODE...")

        # 3. تشغيل المحركات (بعد ثانيتين)
        if self.counter == 20 and self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info("ARMING DRONE...")

        self.counter += 1

def main(args=None):
    print("Starting Drone Mission...")
    rclpy.init(args=args)
    drone_control = DroneControl()
    rclpy.spin(drone_control)
    drone_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
