#!/usr/bin/env python3
"""
mission_control.py — Keyboard Mission Control Panel

Press S to START the mission (drone takes off + follows roads)
Press E to END the mission (drone lands)
Press Q to QUIT this control panel
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty


def get_key():
    """Read a single keypress without requiring Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control')
        self.cmd_pub = self.create_publisher(String, '/mission/command', 10)
        self.get_logger().info('Mission control ready')

    def send_command(self, cmd):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = MissionControlNode()

    print("")
    print("═══════════════════════════════════════════════════")
    print("  🎮  MISSION CONTROL PANEL")
    print("═══════════════════════════════════════════════════")
    print("")
    print("  [ S ]  →  START mission (takeoff + follow roads)")
    print("  [ E ]  →  END mission   (land the drone)")
    print("  [ Q ]  →  QUIT this panel")
    print("")
    print("  Waiting for command...")
    print("")

    try:
        while True:
            key = get_key().lower()

            if key == 's':
                print("  ▶ START command sent — drone will take off!")
                node.send_command('START')

            elif key == 'e':
                print("  ⏹ END command sent — drone will land!")
                node.send_command('END')

            elif key == 'q' or key == '\x03':  # q or Ctrl+C
                print("  Quitting control panel...")
                break

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
