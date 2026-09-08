#!/usr/bin/env python3
import rclpy
from moveit.planning import MoveItPy

def main():
    rclpy.init()
    kinova = MoveItPy(node_name="gripper_test")
    gripper = kinova.get_planning_component("gripper")

    def move_gripper(state_name):
        gripper.set_start_state_to_current_state()
        gripper.set_goal_state(configuration_name=state_name)
        plan_result = gripper.plan()
        if plan_result:
            kinova.execute(plan_result.trajectory, controllers=[])
            print(f"Gripper moved to '{state_name}'")
        else:
            print(f"Planning FAILED for '{state_name}'")

    move_gripper("Open")
    move_gripper("Close")

    rclpy.shutdown()

if __name__ == "__main__":
    main()