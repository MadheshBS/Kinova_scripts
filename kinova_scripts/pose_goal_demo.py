import math
import rclpy
import os
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped

CONTINUOUS_JOINTS = ["joint_1", "joint_3", "joint_5", "joint_7"]
PLANNING_GROUP = "manipulator"

def wrap_to_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def main():
    rclpy.init()

    kinova = MoveItPy(node_name="kinova_pose_goal_demo")
    arm = kinova.get_planning_component(PLANNING_GROUP)

    joint_group = kinova.get_robot_model().get_joint_model_group(PLANNING_GROUP)
    group_joint_names = list(joint_group.active_joint_model_names)

    def get_safe_start_state():
        with kinova.get_planning_scene_monitor().read_write() as scene:
            robot_state = scene.current_state
            robot_state.update()

            positions = robot_state.get_joint_group_positions(PLANNING_GROUP)
            changed = False

            for joint_name in CONTINUOUS_JOINTS:
                if joint_name in group_joint_names:
                    idx = group_joint_names.index(joint_name)
                    current_val = positions[idx]
                    wrapped_val = wrap_to_pi(current_val)
                    if abs(wrapped_val - current_val) > 1e-6:
                        positions[idx] = wrapped_val
                        changed = True

            if changed:
                robot_state.set_joint_group_positions(PLANNING_GROUP, positions)

            robot_state.update()
            return robot_state

        if changed:
            robot_state.set_joint_group_positions(PLANNING_GROUP, positions)

        robot_state.update()
        return robot_state

    def move_to_home():
        safe_state = get_safe_start_state()
        arm.set_start_state(robot_state=safe_state)
        arm.set_goal_state(configuration_name="Home")
        plan_result = arm.plan()
        if plan_result:
            print("Plan succeeded, moving to Home...")
            kinova.execute(plan_result.trajectory, controllers=[])
        else:
            print("Home plan failed.")

    def move_to_pose(x, y, z):
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.orientation.w = 1.0
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z

        safe_state = get_safe_start_state()
        arm.set_start_state(robot_state=safe_state)
        arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="end_effector_link")
        plan_result = arm.plan()
        if plan_result:
            print(f"Plan succeeded, moving to ({x}, {y}, {z})...")
            kinova.execute(plan_result.trajectory, controllers=[])
        else:
            print("Pose plan failed.")

    move_to_home()
    move_to_pose(0.4, 0.0, 0.5)
    move_to_home()
    print("Done.")
    os._exit(0)
    rclpy.shutdown()

if __name__ == "__main__":
    main()