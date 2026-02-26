#!/usr/bin/env python3
"""
road_detection_node.py — ROS 2 Road Line Detection for CitySim

Subscribes to a downward-facing camera on the drone and detects road lines
using OpenCV. Publishes the road centerline offset and heading for the
road-following controller.

Pipeline:
  1. Receive RGB frame from /thermal_camera/image_raw
  2. Convert to HSV
  3. Threshold for dark gray road surface (low saturation, mid value)
  4. Morphological clean-up
  5. Find contours → largest = road
  6. Compute road center offset from frame center
  7. Compute road heading via fitLine or minAreaRect
  8. Publish offset, heading, detection flag, annotated image
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class RoadDetectionNode(Node):
    def __init__(self):
        super().__init__('road_detection_node')

        # ─── Parameters ───
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('road_h_low', 0)
        self.declare_parameter('road_h_high', 180)
        self.declare_parameter('road_s_low', 0)
        self.declare_parameter('road_s_high', 80)
        self.declare_parameter('road_v_low', 20)
        self.declare_parameter('road_v_high', 160)
        self.declare_parameter('min_road_area', 500)
        self.declare_parameter('blur_size', 5)
        self.declare_parameter('morph_size', 7)

        # Read params
        camera_topic = self.get_parameter('camera_topic').value
        self.h_low = self.get_parameter('road_h_low').value
        self.h_high = self.get_parameter('road_h_high').value
        self.s_low = self.get_parameter('road_s_low').value
        self.s_high = self.get_parameter('road_s_high').value
        self.v_low = self.get_parameter('road_v_low').value
        self.v_high = self.get_parameter('road_v_high').value
        self.min_area = self.get_parameter('min_road_area').value
        self.blur_k = self.get_parameter('blur_size').value
        self.morph_k = self.get_parameter('morph_size').value

        # ─── ROS interfaces ───
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)

        self.offset_pub = self.create_publisher(Point, '/road_tracker/road_offset', 10)
        self.detected_pub = self.create_publisher(Bool, '/road_tracker/road_detected', 10)
        self.annotated_pub = self.create_publisher(Image, '/road_tracker/annotated_image', 10)

        self.get_logger().info(f'Road detection node started — listening on {camera_topic}')
        self.get_logger().info(f'HSV range: H[{self.h_low}-{self.h_high}] '
                               f'S[{self.s_low}-{self.s_high}] V[{self.v_low}-{self.v_high}]')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        h, w = cv_image.shape[:2]

        # ─── Step 1: Preprocessing ───
        blurred = cv2.GaussianBlur(cv_image, (self.blur_k, self.blur_k), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # ─── Step 2: Road mask — dark gray/asphalt (low saturation, medium value) ───
        lower = np.array([self.h_low, self.s_low, self.v_low])
        upper = np.array([self.h_high, self.s_high, self.v_high])
        road_mask = cv2.inRange(hsv, lower, upper)

        # ─── Step 3: Morphological operations ───
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,
                                           (self.morph_k, self.morph_k))
        road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_CLOSE, kernel)
        road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_OPEN, kernel)

        # ─── Step 4: Find road contours ───
        contours, _ = cv2.findContours(road_mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        # Filter by area
        valid_contours = [c for c in contours if cv2.contourArea(c) > self.min_area]

        # ─── Step 5: Detection logic ───
        detected = False
        offset_x = 0.0
        offset_y = 0.0
        road_angle = 0.0

        # Debug image
        annotated = cv_image.copy()

        if valid_contours:
            detected = True

            # Merge all road contours into one big mask
            all_road = np.zeros((h, w), dtype=np.uint8)
            cv2.drawContours(all_road, valid_contours, -1, 255, -1)

            # Compute moments for center of road mass
            M = cv2.moments(all_road)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
            else:
                cx, cy = w // 2, h // 2

            # Normalized offset from frame center [-1, 1]
            offset_x = (cx - w / 2.0) / (w / 2.0)
            offset_y = (cy - h / 2.0) / (h / 2.0)

            # Compute road heading using the largest contour's fitted line
            largest = max(valid_contours, key=cv2.contourArea)

            # Fit a line to the road edge points
            if len(largest) >= 5:
                [vx, vy, x0, y0] = cv2.fitLine(largest, cv2.DIST_L2, 0, 0.01, 0.01)
                road_angle = math.atan2(float(vy), float(vx))

                # Draw the fitted line
                line_length = max(w, h)
                pt1 = (int(x0 - vx * line_length), int(y0 - vy * line_length))
                pt2 = (int(x0 + vx * line_length), int(y0 + vy * line_length))
                cv2.line(annotated, pt1, pt2, (0, 255, 255), 2)

            # ─── Draw on annotated image ───
            # Road overlay in green
            road_overlay = np.zeros_like(annotated)
            cv2.drawContours(road_overlay, valid_contours, -1, (0, 200, 0), -1)
            annotated = cv2.addWeighted(annotated, 0.7, road_overlay, 0.3, 0)

            # Center of road mass
            cv2.circle(annotated, (cx, cy), 8, (0, 0, 255), -1)

            # Frame center crosshair
            cv2.drawMarker(annotated, (w // 2, h // 2), (255, 255, 0),
                          cv2.MARKER_CROSS, 20, 2)

            # Arrow from frame center to road center
            cv2.arrowedLine(annotated, (w // 2, h // 2), (cx, cy),
                           (0, 255, 255), 2, tipLength=0.3)

            # Info text
            cv2.putText(annotated, f'Offset: ({offset_x:.2f}, {offset_y:.2f})',
                       (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(annotated, f'Angle: {math.degrees(road_angle):.1f} deg',
                       (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(annotated, f'Road area: {sum(cv2.contourArea(c) for c in valid_contours):.0f}px',
                       (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            # No road detected
            cv2.putText(annotated, 'NO ROAD DETECTED', (w // 2 - 120, h // 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.drawMarker(annotated, (w // 2, h // 2), (255, 255, 0),
                          cv2.MARKER_CROSS, 20, 2)

        # ─── Publish ───
        offset_msg = Point()
        offset_msg.x = offset_x      # lateral offset
        offset_msg.y = road_angle     # road heading angle (radians)
        offset_msg.z = offset_y       # forward/backward offset
        self.offset_pub.publish(offset_msg)

        detected_msg = Bool()
        detected_msg.data = detected
        self.detected_pub.publish(detected_msg)

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RoadDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
