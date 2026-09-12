#!/usr/bin/env python3
"""
cube_color_detector.py

Stage 3 of the pick-and-place vision pipeline.

Subscribes to synced RGB, depth, and camera_info topics from the static
overhead camera. For each of red/green/blue:
  - HSV-thresholds the RGB frame
  - finds the largest matching contour's centroid (pixel coords)
  - looks up depth at that pixel
  - deprojects (pixel + depth) -> 3D point in the camera's optical frame
  - transforms that point into base_link via tf2, using the static
    transform published separately (see README) from the validated
    camera extrinsics:
        translation: (0.4, 0.0, 1.5)
        quaternion:  (0.7071, -0.7071, 0, 0)
  - publishes PoseStamped (orientation identity) on /detected_cube/<color>,
    now in the base_link frame

Requires a static_transform_publisher (or equivalent) already broadcasting
base_link -> overhead_camera/camera_link/overhead_rgbd. See README for the
exact command.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
import message_filters
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform support)


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
            color: self.create_publisher(PoseStamped, f'/detected_cube/{color}', 10)
            for color in self.color_ranges
        }

        # Matches the camera_info frame_id confirmed via `ros2 topic echo`.
        self.frame_id = 'overhead_camera/camera_link/overhead_rgbd'
        self.target_frame = 'base_link'

        # tf2 buffer/listener to transform camera-frame detections into
        # base_link, using the static transform published separately.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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

            # Transform camera-frame point into base_link via the static TF.
            try:
                transformed = self.tf_buffer.transform(
                    point_msg, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.2))
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException) as e:
                self.get_logger().warn(
                    f'{color}: transform to {self.target_frame} failed: {e}')
                continue

            pose_msg = PoseStamped()
            pose_msg.header.stamp = rgb_msg.header.stamp
            pose_msg.header.frame_id = self.target_frame
            pose_msg.pose.position.x = transformed.point.x
            pose_msg.pose.position.y = transformed.point.y
            pose_msg.pose.position.z = transformed.point.z
            pose_msg.pose.orientation.w = 1.0  # identity orientation

            self.color_publishers[color].publish(pose_msg)
            self.get_logger().info(
                f'{color}: pixel=({u},{v}) depth={z:.3f}m -> '
                f'base_link=({transformed.point.x:.3f}, {transformed.point.y:.3f}, '
                f'{transformed.point.z:.3f})'
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
        import os
        os._exit(0)


if __name__ == '__main__':
    main()