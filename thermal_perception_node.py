#!/usr/bin/env python3
"""
ROS 2 Thermal Perception Node
===============================
Subscribes to a simulated thermal camera feed and detects human heat signatures.

Processing Pipeline:
  1. Receive RGB frame from Gazebo camera (emissive targets on dark background)
  2. Convert to grayscale
  3. Gaussian blur for noise reduction
  4. Binary threshold to isolate bright "hot" pixels
  5. Morphological operations to clean noise
  6. Contour detection to find heat blobs
  7. Filter by contour area (reject small noise)
  8. Compute bounding box + centroid of largest contour
  9. Publish centroid (normalized), detection status, and annotated debug image

Topics Published:
  /thermal_tracker/target_centroid  (geometry_msgs/Point)   — normalized pixel coords [-1, 1]
  /thermal_tracker/target_detected  (std_msgs/Bool)         — is a target detected?
  /thermal_tracker/annotated_image  (sensor_msgs/Image)     — debug overlay with bounding box

Author: Dhia — PX4 AI Tracker Project
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

from cv_bridge import CvBridge
import cv2
import numpy as np


class ThermalPerceptionNode(Node):
    """
    Processes simulated thermal camera images to detect and localize
    human heat signatures using OpenCV contour detection.
    """

    def __init__(self):
        super().__init__('thermal_perception_node')

        # ─── Parameters (configurable via ROS 2 params) ────────────
        self.declare_parameter('thermal_threshold', 180)
        self.declare_parameter('min_contour_area', 200)
        self.declare_parameter('max_contour_area', 50000)
        self.declare_parameter('blur_kernel_size', 5)
        self.declare_parameter('morph_kernel_size', 5)
        self.declare_parameter('camera_topic', '/thermal_camera/image_raw')

        self.threshold = self.get_parameter('thermal_threshold').value
        self.min_area = self.get_parameter('min_contour_area').value
        self.max_area = self.get_parameter('max_contour_area').value
        self.blur_ksize = self.get_parameter('blur_kernel_size').value
        self.morph_ksize = self.get_parameter('morph_kernel_size').value
        camera_topic = self.get_parameter('camera_topic').value

        # ─── CV Bridge ─────────────────────────────────────────────
        self.bridge = CvBridge()

        # ─── Subscribers ───────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, qos,
        )

        # ─── Publishers ────────────────────────────────────────────
        self.centroid_pub = self.create_publisher(Point, '/thermal_tracker/target_centroid', 10)
        self.detected_pub = self.create_publisher(Bool, '/thermal_tracker/target_detected', 10)
        self.annotated_pub = self.create_publisher(Image, '/thermal_tracker/annotated_image', 10)

        # ─── Internal State ────────────────────────────────────────
        self.frame_count = 0
        self.detection_count = 0

        self.get_logger().info('╔══════════════════════════════════════════════╗')
        self.get_logger().info('║  🌡️  THERMAL PERCEPTION NODE STARTED        ║')
        self.get_logger().info(f'║  Subscribing to: {camera_topic:<24} ║')
        self.get_logger().info(f'║  Threshold: {self.threshold}  |  Min area: {self.min_area:<10} ║')
        self.get_logger().info('╚══════════════════════════════════════════════╝')

    def image_callback(self, msg: Image):
        """Process each incoming thermal camera frame."""
        self.frame_count += 1

        try:
            # Convert ROS Image to OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge conversion failed: {e}')
            return

        h, w = cv_image.shape[:2]

        # ─── Step 1: Convert to grayscale ──────────────────────────
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # ─── Step 2: Gaussian blur for noise suppression ───────────
        blurred = cv2.GaussianBlur(gray, (self.blur_ksize, self.blur_ksize), 0)

        # ─── Step 3: Binary threshold (isolate hot pixels) ─────────
        _, binary = cv2.threshold(blurred, self.threshold, 255, cv2.THRESH_BINARY)

        # ─── Step 4: Morphological open + close to clean noise ─────
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (self.morph_ksize, self.morph_ksize)
        )
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # ─── Step 5: Find contours ─────────────────────────────────
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # ─── Step 6: Filter by area and find largest ───────────────
        valid_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if self.min_area <= area <= self.max_area:
                valid_contours.append((area, c))

        # ─── Prepare annotated debug image ─────────────────────────
        # Apply thermal colormap for visual feedback
        thermal_colored = cv2.applyColorMap(blurred, cv2.COLORMAP_INFERNO)
        annotated = thermal_colored.copy()

        # Draw all valid contours in cyan
        for _, c in valid_contours:
            cv2.drawContours(annotated, [c], -1, (255, 255, 0), 1)

        detected = False
        centroid_x_norm = 0.0
        centroid_y_norm = 0.0

        if valid_contours:
            detected = True
            self.detection_count += 1

            # Sort by area, take the largest
            valid_contours.sort(key=lambda x: x[0], reverse=True)
            largest_area, largest_contour = valid_contours[0]

            # Bounding rectangle
            x, y, bw, bh = cv2.boundingRect(largest_contour)

            # Centroid via moments
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
            else:
                cx = x + bw // 2
                cy = y + bh // 2

            # Normalize centroid to [-1, 1] range (0,0 = center of frame)
            centroid_x_norm = (cx - w / 2.0) / (w / 2.0)
            centroid_y_norm = (cy - h / 2.0) / (h / 2.0)

            # ─── Draw on annotated image ───────────────────────────
            # Bounding box (green)
            cv2.rectangle(annotated, (x, y), (x + bw, y + bh), (0, 255, 0), 2)

            # Centroid crosshair (red)
            cv2.circle(annotated, (cx, cy), 6, (0, 0, 255), -1)
            cv2.line(annotated, (cx - 15, cy), (cx + 15, cy), (0, 0, 255), 2)
            cv2.line(annotated, (cx, cy - 15), (cx, cy + 15), (0, 0, 255), 2)

            # Label
            label = f'HEAT SIG  area={largest_area:.0f}'
            cv2.putText(annotated, label, (x, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Normalized coords display
            coords_text = f'({centroid_x_norm:+.2f}, {centroid_y_norm:+.2f})'
            cv2.putText(annotated, coords_text, (x, y + bh + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

            # Log periodically
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'🎯 Target locked | px=({cx},{cy}) | '
                    f'norm=({centroid_x_norm:+.2f},{centroid_y_norm:+.2f}) | '
                    f'area={largest_area:.0f} | targets={len(valid_contours)}'
                )
        else:
            if self.frame_count % 60 == 0:
                self.get_logger().info('🔍 No heat signature detected')

        # ─── HUD overlay ───────────────────────────────────────────
        # Frame counter
        cv2.putText(annotated, f'Frame: {self.frame_count}', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        # Status indicator
        status_color = (0, 255, 0) if detected else (0, 0, 255)
        status_text = 'TARGET LOCKED' if detected else 'SCANNING...'
        cv2.putText(annotated, status_text, (w - 160, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2)
        # Crosshair at frame center
        cv2.line(annotated, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (100, 100, 100), 1)
        cv2.line(annotated, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (100, 100, 100), 1)

        # ─── Publish results ───────────────────────────────────────
        # Centroid
        centroid_msg = Point()
        centroid_msg.x = centroid_x_norm
        centroid_msg.y = centroid_y_norm
        centroid_msg.z = 0.0  # unused, reserved for depth/distance
        self.centroid_pub.publish(centroid_msg)

        # Detection flag
        detected_msg = Bool()
        detected_msg.data = detected
        self.detected_pub.publish(detected_msg)

        # Annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated image: {e}')


def main(args=None):
    print()
    print('═══════════════════════════════════════════════')
    print('  🌡️  THERMAL PERCEPTION NODE')
    print('  Processing thermal camera feed...')
    print('═══════════════════════════════════════════════')
    print()

    rclpy.init(args=args)
    node = ThermalPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Thermal perception node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
