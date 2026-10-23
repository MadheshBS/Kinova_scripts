#!/usr/bin/env python3
"""
Single-cube pick-and-place demo.

Use with full_sim_random_cube.launch.py, which spawns ONE cube of a random
colour at a random spot on the table. This node:

  1. waits for a detection on /detected_cube/{red,green,blue}
     (ignoring blobs that sit on a place marker, so the coloured slot
     markers on the table can't be mistaken for the cube),
  2. picks that cube using perception (x, y) + calibrated Z constants,
  3. places it on the slot assigned to ITS colour, and returns home.

Motion primitives (seeded-IK stepped Cartesian moves, safe start-state
snapshots, retry logic) are identical to pick_and_place_demo.py.
"""
import os
import time
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.executors import MultiThreadedExecutor

from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import Pose, PoseStamped

PLANNING_GROUP = "manipulator"
GRIPPER_GROUP = "gripper"
EE_LINK = "end_effector_link"
CONTINUOUS_JOINTS = ["joint_1", "joint_3", "joint_5", "joint_7"]

JOINT_LIMITS = {
    "joint_6": (-2.09, 2.09),
    "joint_3": (-math.pi, math.pi),
}
JOINT_LIMIT_MARGIN = 0.001

PREGRASP_Z = 0.38
GRASP_Z = 0.28
GRASP_QUAT = dict(qx=1.0, qy=0.0, qz=0.0, qw=0.0)

MAX_STEP = 0.015
TRANSFER_STEP = 0.03
IK_TIMEOUT = 0.2
SETTLE_TIME = 0.5
STEP_ATTEMPTS = 2

COLORS = ["red", "green", "blue"]

PLACE_POSES = {
    "red":   (0.55, -0.10, GRASP_Z),
    "green": (0.55, 0.00, GRASP_Z),
    "blue":  (0.55, 0.10, GRASP_Z),
}

# A detection closer than this (m, in XY) to any place marker is treated as
# the marker itself, not the cube.
MARKER_REJECT_RADIUS = 0.04
# How long (s) to wait for a valid cube detection, and how long to keep
# listening after the first one so the pose has settled.
DETECTION_TIMEOUT = 30.0
DETECTION_SETTLE = 1.0


def wrap_to_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def make_plan_params(robot, planner_id="RRTConnectkConfigDefault"):
    params = PlanRequestParameters(robot, "ompl")
    params.planning_pipeline = "ompl"
    params.planner_id = planner_id
    params.planning_time = 5.0
    params.planning_attempts = 1
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


def snapshot_state(robot_model, live_state):
    snap = RobotState(robot_model)
    try:
        snap.joint_positions = dict(live_state.joint_positions)
    except Exception:
        for group in (PLANNING_GROUP, GRIPPER_GROUP):
            snap.set_joint_group_positions(group, live_state.get_joint_group_positions(group))
    snap.update()
    return snap


def dump_joints(logger, tag, joint_dict):
    if not joint_dict:
        logger.error(f"{tag}: no raw joint dump available")
        return
    logger.error(f"{tag}: live joints: " +
                 ", ".join(f"{k}={v:.9f}" for k, v in sorted(joint_dict.items())))


def make_safe_state_getter(robot, group_name, logger, continuous_joints=None, joint_limits=None):
    continuous_joints = continuous_joints or []
    joint_limits = joint_limits or {}
    robot_model = robot.get_robot_model()
    group_model = robot_model.get_joint_model_group(group_name)
    names = list(group_model.active_joint_model_names)

    def get_safe_state():
        with robot.get_planning_scene_monitor().read_write() as scene:
            live = scene.current_state
            live.update()
            try:
                get_safe_state.last_raw = dict(live.joint_positions)
            except Exception:
                get_safe_state.last_raw = None
            state = snapshot_state(robot_model, live)

        positions = state.get_joint_group_positions(group_name)

        for joint_name in continuous_joints:
            if joint_name in names:
                idx = names.index(joint_name)
                val = positions[idx]
                wrapped = wrap_to_pi(val)
                if abs(wrapped - val) > 1e-6:
                    positions[idx] = wrapped

        for joint_name, (lo, hi) in joint_limits.items():
            if joint_name in names:
                idx = names.index(joint_name)
                val = positions[idx]
                clamped = min(max(val, lo + JOINT_LIMIT_MARGIN), hi - JOINT_LIMIT_MARGIN)
                if clamped != val:
                    logger.warning(f"{joint_name} at {val:.6f}, clamping to {clamped:.6f} "
                                   f"(limit [{lo}, {hi}])")
                    positions[idx] = clamped

        state.set_joint_group_positions(group_name, positions)
        state.update()
        return state

    get_safe_state.last_raw = None
    return get_safe_state


def joints_close(state_a, state_b, group_name=PLANNING_GROUP, tol=0.05):
    a = state_a.get_joint_group_positions(group_name)
    b = state_b.get_joint_group_positions(group_name)
    return all(abs(wrap_to_pi(x - y)) <= tol for x, y in zip(a, b))


def make_gripper_state_getter(robot, logger, open_snap=0.01):
    robot_model = robot.get_robot_model()

    def get_gripper_state():
        with robot.get_planning_scene_monitor().read_write() as scene:
            live = scene.current_state
            live.update()
            state = snapshot_state(robot_model, live)
        positions = state.get_joint_group_positions(GRIPPER_GROUP)
        if max(abs(v) for v in positions) < open_snap:
            positions[:] = 0.0
            state.set_joint_group_positions(GRIPPER_GROUP, positions)
        state.update()
        return state

    return get_gripper_state


def ee_position_error(state, x, y, z):
    try:
        pose = state.get_pose(EE_LINK)
        return math.sqrt((pose.position.x - x) ** 2 +
                         (pose.position.y - y) ** 2 +
                         (pose.position.z - z) ** 2)
    except Exception:
        return None


def solve_seeded_ik(robot_model, seed_state, x, y, z, qx, qy, qz, qw,
                    group_name=PLANNING_GROUP, link_name=EE_LINK,
                    timeout=IK_TIMEOUT):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw

    candidate = RobotState(robot_model)
    candidate.set_joint_group_positions(group_name, seed_state.get_joint_group_positions(group_name))
    candidate.update()

    if not candidate.set_from_ik(group_name, pose, link_name, timeout):
        return None
    candidate.update()
    return candidate


def stepped_cartesian_move(manipulator, robot, robot_model, logger, plan_params,
                           x_start, y_start, z_start,
                           x_end, y_end, z_end,
                           max_step=MAX_STEP, get_start_state=None,
                           label_prefix="move", tolerate_blocked_end=False):
    distance = math.sqrt((x_end - x_start) ** 2 + (y_end - y_start) ** 2 + (z_end - z_start) ** 2)
    n_steps = max(3, math.ceil(distance / max_step))

    for i in range(1, n_steps + 1):
        t = i / n_steps
        x = x_start + (x_end - x_start) * t
        y = y_start + (y_end - y_start) * t
        z = z_start + (z_end - z_start) * t

        reached = False
        achieved = None
        target_state = None
        detail = "no attempt made"

        for attempt in range(STEP_ATTEMPTS):
            current_state = get_start_state() if get_start_state is not None else None
            if current_state is None:
                logger.error(f"{label_prefix} step {i}/{n_steps}: no start state available")
                return False

            target_state = solve_seeded_ik(robot_model, current_state, x, y, z, **GRASP_QUAT)
            if target_state is None:
                logger.error(f"{label_prefix} step {i}/{n_steps}: seeded IK failed for "
                             f"({x:.3f},{y:.3f},{z:.3f})")
                return False

            manipulator.set_start_state(robot_state=current_state)
            manipulator.set_goal_state(robot_state=target_state)
            plan_and_execute(manipulator, robot, logger,
                             f"{label_prefix} step {i}/{n_steps} try {attempt + 1} "
                             f"({x:.3f},{y:.3f},{z:.3f})",
                             plan_params)
            time.sleep(SETTLE_TIME)

            achieved = get_start_state()
            err = ee_position_error(achieved, x, y, z)
            if err is None:
                reached = joints_close(achieved, target_state)
                detail = "joint check"
            else:
                is_last = (i == n_steps)
                limit = 0.03 if (tolerate_blocked_end and is_last) else 0.02
                reached = err <= limit
                detail = f"EE error {err * 1000:.1f} mm (limit {limit * 1000:.0f} mm)"
                if reached and err > 0.01:
                    logger.warning(f"{label_prefix} step {i}/{n_steps}: stopped {err * 1000:.1f} mm "
                                   f"short of target (likely contact) -- continuing.")
            if reached:
                break
            logger.warning(f"{label_prefix} step {i}/{n_steps}: attempt {attempt + 1} "
                           f"missed [{detail}]"
                           + (", retrying." if attempt + 1 < STEP_ATTEMPTS else "."))

        if not reached:
            logger.error(f"{label_prefix} step {i}/{n_steps}: arm did NOT reach the target "
                         f"[{detail}] -- aborting.")
            a = achieved.get_joint_group_positions(PLANNING_GROUP)
            tg = target_state.get_joint_group_positions(PLANNING_GROUP)
            logger.error(f"{label_prefix}: per-joint error achieved-target (rad): " +
                         ", ".join(f"j{k + 1}={wrap_to_pi(av - tv):+.4f}"
                                   for k, (av, tv) in enumerate(zip(a, tg))))
            dump_joints(logger, label_prefix, getattr(get_start_state, "last_raw", None))
            return False

    return True


class CubePoseSubscriber(Node):
    """Holds the latest PoseStamped per colour (spun on a background thread)."""

    def __init__(self):
        super().__init__("pick_and_place_single_cube_sub")
        self._lock = threading.Lock()
        self._latest = {c: None for c in COLORS}
        for color in COLORS:
            self.create_subscription(
                PoseStamped, f"/detected_cube/{color}",
                self._make_callback(color), 10)

    def _make_callback(self, color):
        def cb(msg):
            with self._lock:
                self._latest[color] = msg
        return cb

    def get_latest(self, color):
        with self._lock:
            return self._latest[color]

    @staticmethod
    def _on_marker(msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        return any(math.hypot(x - mx, y - my) < MARKER_REJECT_RADIUS
                   for mx, my, _ in PLACE_POSES.values())

    def find_cube(self, logger, timeout_sec=DETECTION_TIMEOUT, settle_sec=DETECTION_SETTLE,
                  poll_period=0.1):
        """
        Wait for a detection that is NOT sitting on a place marker. Once one
        shows up, keep listening for settle_sec so the pose is current, then
        return (color, PoseStamped). Returns (None, None) on timeout.
        """
        def candidates():
            return {c: self.get_latest(c) for c in COLORS
                    if self.get_latest(c) is not None
                    and not self._on_marker(self.get_latest(c))}

        elapsed = 0.0
        while elapsed < timeout_sec:
            if candidates():
                time.sleep(settle_sec)
                found = candidates()
                if not found:
                    elapsed += settle_sec
                    continue
                if len(found) > 1:
                    logger.warning(f"Multiple colours detected off-marker: {list(found)}; "
                                   f"using '{next(iter(found))}'.")
                color = next(iter(found))
                return color, found[color]
            time.sleep(poll_period)
            elapsed += poll_period
        return None, None


def main():
    rclpy.init()
    logger = get_logger("pick_and_place_single")

    cube_sub = CubePoseSubscriber()
    executor = MultiThreadedExecutor()
    executor.add_node(cube_sub)
    threading.Thread(target=executor.spin, daemon=True).start()

    robot = MoveItPy(node_name="pick_and_place_single")
    time.sleep(2.0)
    manipulator = robot.get_planning_component(PLANNING_GROUP)
    gripper = robot.get_planning_component(GRIPPER_GROUP)
    robot_model = robot.get_robot_model()
    plan_params = make_plan_params(robot)

    get_safe_start_state = make_safe_state_getter(
        robot, PLANNING_GROUP, logger,
        continuous_joints=[], joint_limits=JOINT_LIMITS)
    get_safe_gripper_state = make_gripper_state_getter(robot, logger)

    def move_home():
        for attempt in range(2):
            time.sleep(SETTLE_TIME)
            manipulator.set_start_state(robot_state=get_safe_start_state())
            manipulator.set_goal_state(configuration_name="Home")
            if plan_and_execute(manipulator, robot, logger,
                                f"move to Home (try {attempt + 1})", plan_params):
                return True
            dump_joints(logger, "move to Home", getattr(get_safe_start_state, "last_raw", None))
        return False

    def set_gripper(state_name):
        gripper.set_start_state(robot_state=get_safe_gripper_state())
        gripper.set_goal_state(configuration_name=state_name)
        return plan_and_execute(gripper, robot, logger, f"gripper {state_name}", plan_params)

    def move_to_pose(x, y, z, label):
        seed = get_safe_start_state()
        target = solve_seeded_ik(robot_model, seed, x, y, z, **GRASP_QUAT)
        if target is None:
            logger.error(f"{label}: seeded IK failed for ({x:.3f},{y:.3f},{z:.3f})")
            return False
        manipulator.set_start_state(robot_state=seed)
        manipulator.set_goal_state(robot_state=target)
        return plan_and_execute(manipulator, robot, logger, label, plan_params)

    def fine_move(x0, y0, z0, x1, y1, z1, label_prefix,
                  tolerate_blocked_end=False, max_step=MAX_STEP):
        return stepped_cartesian_move(manipulator, robot, robot_model, logger, plan_params,
                                      x0, y0, z0, x1, y1, z1,
                                      max_step=max_step,
                                      get_start_state=get_safe_start_state,
                                      label_prefix=label_prefix,
                                      tolerate_blocked_end=tolerate_blocked_end)

    # --- main sequence ---
    move_home()
    if not set_gripper("Open"):
        logger.warning("Initial gripper Open failed; continuing.")

    logger.info("Waiting for the cube detection...")
    color, pick_pose_msg = cube_sub.find_cube(logger)
    if color is None:
        logger.error(f"No cube detected within {DETECTION_TIMEOUT:.0f} s. Is cube_color_detector running?")
        os._exit(1)

    px = pick_pose_msg.pose.position.x
    py = pick_pose_msg.pose.position.y
    place_x, place_y, place_z = PLACE_POSES[color]
    logger.info(f"Detected {color} cube at ({px:.3f}, {py:.3f}); "
                f"target slot ({place_x:.2f}, {place_y:.2f}).")

    ok = False
    if not move_to_pose(px, py, PREGRASP_Z, f"{color}: pregrasp"):
        logger.error(f"{color}: pregrasp planning failed.")
    elif not fine_move(px, py, PREGRASP_Z, px, py, GRASP_Z, f"{color}: descend to grasp"):
        logger.error(f"{color}: descent to grasp failed.")
    elif not set_gripper("Close"):
        logger.error(f"{color}: gripper close failed (cube NOT grasped).")
    elif not fine_move(px, py, GRASP_Z, px, py, PREGRASP_Z, f"{color}: lift to pregrasp"):
        logger.error(f"{color}: lift failed (still gripping).")
    elif not fine_move(px, py, PREGRASP_Z, place_x, place_y, PREGRASP_Z,
                       f"{color}: transfer to place", max_step=TRANSFER_STEP):
        logger.error(f"{color}: transfer failed (still gripping).")
    elif not fine_move(place_x, place_y, PREGRASP_Z, place_x, place_y, place_z,
                       f"{color}: descend to place", tolerate_blocked_end=True):
        logger.error(f"{color}: descent to place failed (still gripping).")
    else:
        if not set_gripper("Open"):
            logger.error(f"{color}: gripper Open failed at place location -- cube may still be held.")
        elif not fine_move(place_x, place_y, place_z, place_x, place_y, PREGRASP_Z,
                           f"{color}: retreat to pregrasp"):
            logger.error(f"{color}: retreat failed -- continuing to Home anyway.")
            ok = True
        else:
            ok = True

    move_home()
    logger.info("Pick-and-place complete." if ok else "Sequence aborted; arm returned home.")
    os._exit(0 if ok else 1)


if __name__ == "__main__":
    main()