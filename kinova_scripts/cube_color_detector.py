#!/usr/bin/env python3
"""
cube_color_detector.py

Stage 2 of the pick-and-place vision pipeline.

Subscribes to synced RGB, depth, and camera_info topics from the static
overhead camera. For each of red/green/blue:
  - HSV-thresholds the RGB frame
  - finds the largest matching contour's centroid (pixel coords)
  - looks up depth at that pixel
  - deprojects (pixel + depth) -> 3D point in the camera's optical frame
  - publishes PointStamped on /detected_cube/<color>

Deliberately does NOT transform into base_link -- that's Stage 3, once the
camera's static mount pose relative to the robot base is known. Points here
are published directly in the camera_info frame_id.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import message_filters
from cv_bridge import CvBridge
import cv2
import numpy as np


class CubeColorDetector(Node):
    def __init__(self):
        super().__init__('cube_color_detector')
        self.bridge = CvBridge()

        # Fallback intrinsics (overwritten live from camera_info each callback,
        # in case they ever change -- e.g. if the camera model is swapped).
        self.fx = 554.3827128226441
        self.fy = 554.3827128226441
        self.cx = 320.0
        self.cy = 240.0

        # HSV bounds, derived empirically from rendered pixel samples
        # (red=H0/S190/V226, green=H60/S194/V229, blue=H119/S195/V228),
        # widened on S/V to tolerate shading across each cube's face.
        # Red wraps hue 0/179 so it needs two ranges OR'd together.
        self.color_ranges = {
            'red':   [((0, 100, 100), (10, 255, 255)),
                      ((170, 100, 100), (179, 255, 255))],
            'green': [((45, 100, 100), (75, 255, 255))],
            'blue':  [((105, 100, 100), (135, 255, 255))],
        }

        self.color_publishers = {
            color: self.create_publisher(PointStamped, f'/detected_cube/{color}', 10)
            for color in self.color_ranges
        }

        # Matches the camera_info frame_id confirmed via `ros2 topic echo`.
        self.frame_id = 'overhead_camera/camera_link/overhead_rgbd'

        # Minimum contour area (pixels) to accept as a real detection,
        # filtering out stray noise pixels that happen to pass the HSV mask.
        self.min_contour_area = 20.0

        rgb_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/camera_info')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

        self.get_logger().info('cube_color_detector started, waiting for synced frames...')

    def callback(self, rgb_msg, depth_msg, info_msg):
        # Refresh intrinsics from the live camera_info message.
        self.fx = info_msg.k[0]
        self.fy = info_msg.k[4]
        self.cx = info_msg.k[2]
        self.cy = info_msg.k[5]

        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

        for color, ranges in self.color_ranges.items():
            mask = None
            for lower, upper in ranges:
                m = cv2.inRange(hsv, np.array(lower), np.array(upper))
                mask = m if mask is None else cv2.bitwise_or(mask, m)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) < self.min_contour_area:
                continue

            moments = cv2.moments(largest)
            if moments['m00'] == 0:
                continue

            u = int(moments['m10'] / moments['m00'])
            v = int(moments['m01'] / moments['m00'])

            if v >= depth.shape[0] or u >= depth.shape[1] or u < 0 or v < 0:
                self.get_logger().warn(f'{color}: centroid ({u},{v}) outside depth frame bounds')
                continue

            z = float(depth[v, u])
            if not np.isfinite(z) or z <= 0.0:
                self.get_logger().warn(f'{color}: invalid depth at pixel ({u},{v}) -> {z}')
                continue

            # Pinhole deprojection: pixel + depth -> 3D point in camera optical frame.
            x = (u - self.cx) * z / self.fx
            y = (v - self.cy) * z / self.fy

            point_msg = PointStamped()
            point_msg.header.stamp = rgb_msg.header.stamp
            point_msg.header.frame_id = self.frame_id
            point_msg.point.x = x
            point_msg.point.y = y
            point_msg.point.z = z

            self.color_publishers[color].publish(point_msg)
            self.get_logger().info(
                f'{color}: pixel=({u},{v}) depth={z:.3f}m -> '
                f'point=({x:.3f}, {y:.3f}, {z:.3f}) in {self.frame_id}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CubeColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()