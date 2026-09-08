#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.logging import get_logger

from moveit.planning import MoveItPy, PlanRequestParameters
from geometry_msgs.msg import PoseStamped


def make_pose(x, y, z, qx=1.0, qy=0.0, qz=0.0, qw=0.0, frame="base_link"):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def make_plan_params(robot, planner_id="RRTConnectkConfigDefault"):
    params = PlanRequestParameters(robot, "ompl")
    params.planning_pipeline = "ompl"      # force it - don't rely on yaml lookup
    params.planner_id = planner_id
    params.planning_time = 5.0
    params.planning_attempts = 5
    params.max_velocity_scaling_factor = 0.3
    params.max_acceleration_scaling_factor = 0.3
    return params


def plan_and_execute(planning_component, robot, logger, label, plan_params):
    logger.info(f"Planning: {label}")
    plan_result = planning_component.plan(single_plan_parameters=plan_params)

    if not plan_result:
        logger.error(f"Planning FAILED: {label}")
        return False

    robot.execute(plan_result.trajectory, controllers=[])
    logger.info(f"Executed: {label}")
    return True


def stepped_cartesian_descent(manipulator, robot, logger, plan_params, x, y, z_start, z_end, n_steps=6):
    zs = [z_start + (z_end - z_start) * i / n_steps for i in range(1, n_steps + 1)]
    for i, z in enumerate(zs, start=1):
        manipulator.set_start_state_to_current_state()
        goal_pose = make_pose(x, y, z)
        manipulator.set_goal_state(pose_stamped_msg=goal_pose, pose_link="end_effector_link")
        ok = plan_and_execute(manipulator, robot, logger, f"descent step {i}/{n_steps} (z={z:.3f})", plan_params)
        if not ok:
            logger.error(f"Descent aborted at step {i} (z={z:.3f})")
            return False
        time.sleep(0.2)
    return True


def main():
    rclpy.init()
    logger = get_logger("grasp_isolation_test")

    robot = MoveItPy(node_name="grasp_isolation_test")
    time.sleep(2.0)
    manipulator = robot.get_planning_component("manipulator")
    gripper = robot.get_planning_component("gripper")

    plan_params = make_plan_params(robot)

    X, Y = 0.4, 0.0
    PREGRASP_Z = 0.38
    GRASP_Z = 0.28
    LIFT_Z = 0.38

    manipulator.set_start_state_to_current_state()
    manipulator.set_goal_state(configuration_name="Home")
    plan_and_execute(manipulator, robot, logger, "move to Home", plan_params)

    gripper.set_start_state_to_current_state()
    gripper.set_goal_state(configuration_name="Open")
    plan_and_execute(gripper, robot, logger, "open gripper", plan_params)

    manipulator.set_start_state_to_current_state()
    manipulator.set_goal_state(pose_stamped_msg=make_pose(X, Y, PREGRASP_Z), pose_link="end_effector_link")
    plan_and_execute(manipulator, robot, logger, "move to pre-grasp", plan_params)

    stepped_cartesian_descent(manipulator, robot, logger, plan_params, X, Y, PREGRASP_Z, GRASP_Z, n_steps=6)

    gripper.set_start_state_to_current_state()
    gripper.set_goal_state(configuration_name="Close")
    plan_and_execute(gripper, robot, logger, "close gripper", plan_params)

    stepped_cartesian_descent(manipulator, robot, logger, plan_params, X, Y, GRASP_Z, LIFT_Z, n_steps=6)

    manipulator.set_start_state_to_current_state()
    manipulator.set_goal_state(configuration_name="Home")
    plan_and_execute(manipulator, robot, logger, "return to Home", plan_params)

    logger.info("Sequence complete.")
    # os._exit skips normal interpreter/destructor teardown, which avoids a
    # known moveit_py crash where the MoveItCpp C++ destructor segfaults
    # (exit code -11) during its internal spin-thread cleanup. Everything
    # above has already completed by this point, so this is safe.
    os._exit(0)


if __name__ == "__main__":
    main()