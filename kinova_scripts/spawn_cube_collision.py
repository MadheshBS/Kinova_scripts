#!/usr/bin/env python3
"""
spawn_cube_collision.py

Publishes a CollisionObject into the MoveIt planning scene matching the
real cube spawned in Gazebo at (0.4, 0.0, 0.02), 4cm side -- confirmed via
the Gazebo GUI entity pose.

This makes the cube visible in RViz's Motion Planning panel, so you can
visually drag the interactive marker down to it and confirm grasp height
and orientation together, rather than guessing coordinates blind.

Run this once, after RViz/move_group are up. It publishes a single latched
message and exits.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time


class CubeSpawner(Node):
    def __init__(self):
        super().__init__("spawn_cube_collision")
        self.publisher = self.create_publisher(CollisionObject, "collision_object", 10)

    def spawn(self):
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = "grasp_target_cube"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.04, 0.04, 0.04]  # 4cm cube

        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = 0.0
        pose.position.z = 0.02  # confirmed real cube center height from Gazebo GUI
        pose.orientation.w = 1.0

        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD

        # Give the publisher time to register with the planning scene monitor
        time.sleep(1.0)
        self.publisher.publish(obj)
        self.get_logger().info("Published collision object 'grasp_target_cube' at (0.4, 0.0, 0.02)")


def main():
    rclpy.init()
    node = CubeSpawner()
    node.spawn()
    time.sleep(0.5)  # let the message actually go out before shutdown
    rclpy.shutdown()


if __name__ == "__main__":
    main()