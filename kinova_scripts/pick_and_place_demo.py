#!/usr/bin/env python3
"""
Stage 4: full color-sorting pick-and-place demo.

Subscribes to /detected_cube/{red,green,blue} (PoseStamped, base_link frame,
published by cube_color_detector.py) and for each color runs:

    home -> pregrasp -> stepped descent -> grasp -> lift
          -> stepped horizontal transfer -> stepped descent(place) -> release
          -> stepped retreat -> home

All fine motions (descent/lift/transfer) are done via a generalized
stepped Cartesian interpolation. Each step's target joint configuration is
found via IK SEEDED FROM THE ARM'S CURRENT JOINT STATE, then handed to the
planner as a joint-space goal (not a pose goal), so the solver picks the
nearest valid configuration instead of an arbitrary branch.

Changes in this revision (on top of the snapshot-state version):
  1. stepped_cartesian_move() retries each step once. After a timed-out or
     cancelled trajectory the arm is usually just slightly short, so we wait
     for it to settle, re-read the state, re-solve IK, and try again before
     aborting.
  2. move_home() waits 0.5 s before snapshotting the start state and retries
     once (a snapshot taken right after a cancel caused the joint_2/joint_4
     "start point deviates" rejections).
  3. fine_move() accepts a max_step override; the horizontal transfer uses
     TRANSFER_STEP (30 mm) since it is free-space motion, which halves the
     number of plan/execute cycles.

Reuses the validated primitives from grasp_isolation_test.py
(make_plan_params, plan_and_execute) and the continuous-joint-safe
start-state pattern from pose_goal_demo.py.
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

# joint_6 is NOT continuous -- hard limit confirmed at runtime from
# "should be in the range [-2.09], [2.09]". If joint_2/joint_4 ever produce
# a similar CheckStartStateBounds error, add their limits here too (check
# gen3.srdf). Without this, a joint that lands even 0.00001 rad past its
# limit (floating-point/controller overshoot, not a real mechanical issue)
# makes MoveIt reject the live current state for EVERY future plan call.
JOINT_LIMITS = {
    "joint_6": (-2.09, 2.09),
    # joint_3 sits at ~pi (Home pose) and its live reading jitters across
    # +pi: the log showed joint_3 = 3.141602516 (pi + 1e-5), which makes
    # CheckStartStateBounds reject every plan from then on. Clamping to
    # pi - margin changes the start state by ~1e-3 rad -- well inside the
    # 0.01 rad trajectory-start tolerance, unlike a 2*pi wrap.
    "joint_3": (-math.pi, math.pi),
}
JOINT_LIMIT_MARGIN = 0.001  # stay a hair inside the limit, not exactly on it

# Gripper: only the MASTER joint is clamped. The other five finger joints
# (inner knuckles, finger tips, right knuckle) are mimic joints slaved to
# this one; they are recomputed from it when positions are set.
GRIPPER_MASTER_LIMITS = {
    "robotiq_85_left_knuckle_joint": (0.0, 0.8),
}

# Calibrated Z offsets (mesh-geometry quirk, see grasp_isolation_test.py).
# Table top sits at z=0.4 in base_link; these hold regardless of (x, y).
PREGRASP_Z = 0.38
GRASP_Z = 0.28

# Fixed straight-down gripper orientation used throughout.
GRASP_QUAT = dict(qx=1.0, qy=0.0, qz=0.0, qw=0.0)

# Max Cartesian distance (m) allowed per planning step in fine-motion moves
# (descents / lifts, where precision matters).
MAX_STEP = 0.015

# Larger step (m) for the free-space horizontal transfer at pregrasp height.
TRANSFER_STEP = 0.03

# Timeout (s) for each seeded IK solve.
IK_TIMEOUT = 0.2

# Seconds to wait after each execute so the arm settles (and any cancelled
# trajectory finishes stopping) before the state is read back.
SETTLE_TIME = 0.5

# Attempts per Cartesian step (1 = no retry).
STEP_ATTEMPTS = 2

COLORS = ["red", "green", "blue"]

# Table spawned at (0.4, 0.0, 0.2) in base_link. Marker offsets from
# simple_table.sdf:
#   red   (0.15, -0.10, 0.201)  -> world (0.55, -0.10, 0.401)
#   green (0.15,  0.00, 0.201)  -> world (0.55,  0.00, 0.401)
#   blue  (0.15,  0.10, 0.201)  -> world (0.55,  0.10, 0.401)
# Z pinned to GRASP_Z (same fingertip-contact height as pickup).
PLACE_POSES = {
    "red":   (0.55, -0.10, GRASP_Z),
    "green": (0.55, 0.00, GRASP_Z),
    "blue":  (0.55, 0.10, GRASP_Z),
}


def wrap_to_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


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
    # planning_attempts = 1: a single path search per step, not a random
    # pick among several parallel RRTConnect runs.
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
    """
    Return an independent RobotState copy of live_state.

    NOT independently verified against this moveit_py build: tries the
    RobotState.joint_positions dict property first (copies every joint,
    including the gripper while it holds a cube), and falls back to copying
    the two planning groups' positions if that property isn't exposed.
    """
    snap = RobotState(robot_model)
    try:
        snap.joint_positions = dict(live_state.joint_positions)
    except Exception:
        for group in (PLANNING_GROUP, GRIPPER_GROUP):
            snap.set_joint_group_positions(group, live_state.get_joint_group_positions(group))
    snap.update()
    return snap


def dump_joints(logger, tag, joint_dict):
    """Log every joint value (9 dp) so a bounds failure can be traced to a joint."""
    if not joint_dict:
        logger.error(f"{tag}: no raw joint dump available")
        return
    logger.error(f"{tag}: live joints: " +
                 ", ".join(f"{k}={v:.9f}" for k, v in sorted(joint_dict.items())))


def make_safe_state_getter(robot, group_name, logger, continuous_joints=None, joint_limits=None):
    """
    Returns a function that fetches a SNAPSHOT of the current state, wraps
    any continuous joint past +/-pi back into range, and clamps any listed
    joint at/past a hard limit back to just inside bounds.

    The snapshot is taken while holding the planning-scene write lock, and
    all fixes are applied to the copy, never to the live scene state (which
    MoveIt's state monitor keeps overwriting). Positions are always written
    back with set_joint_group_positions() so mimic joints are recomputed
    consistently from their master.
    """
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
    """True if every joint of group_name agrees within tol rad (wrap-aware,
    so a continuous joint at +3.14 and -3.14 counts as the same angle)."""
    a = state_a.get_joint_group_positions(group_name)
    b = state_b.get_joint_group_positions(group_name)
    return all(abs(wrap_to_pi(x - y)) <= tol for x, y in zip(a, b))


def make_gripper_state_getter(robot, logger, open_snap=0.01):
    """
    Start-state snapshot for the gripper group.

    When the fingers are (nearly) fully open the live joint values sit a
    hair below their 0.0 bound (seen: -1.5e-10 .. -5e-4 rad), and
    CheckStartStateBounds rejects the start state. Clamping individual
    entries of the group vector did NOT work: get_joint_group_positions()
    returns one entry per group variable INCLUDING the five mimic joints,
    in model order, so indexing it by the master joint's position in
    active_joint_model_names hit the wrong slot, and set_joint_group_
    positions() then recomputed the mimics from the unchanged master.
    Snapping the whole vector to exactly 0.0 needs no index at all. A
    gripper holding a cube (master ~0.36) is left untouched.
    """
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
    """Distance (m) between the arm's end effector in `state` and (x, y, z),
    or None if this moveit_py build's RobotState.get_pose() doesn't return a
    Pose message."""
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
    """
    IK solve for (x, y, z, quat) seeded from seed_state's current joint
    values, so the result is the NEAREST valid configuration to where the
    arm already is -- not an arbitrary branch.

    Returns a RobotState with the solved joint config, or None on failure.
    """
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

    ok = candidate.set_from_ik(group_name, pose, link_name, timeout)
    if not ok:
        return None

    candidate.update()
    return candidate


def stepped_cartesian_move(manipulator, robot, robot_model, logger, plan_params,
                           x_start, y_start, z_start,
                           x_end, y_end, z_end,
                           max_step=MAX_STEP, get_start_state=None,
                           label_prefix="move", tolerate_blocked_end=False):
    """
    Stepped Cartesian interpolation between two 3D points, with each
    waypoint's joint target found via seeded IK. The seed for each step is
    the arm's actual current state, so consecutive steps chain into a
    continuous, deterministic joint-space path.

    Each step gets up to STEP_ATTEMPTS tries. A timed-out/cancelled
    trajectory normally leaves the arm only slightly short, so the retry
    waits for it to settle, re-reads the state, re-solves IK and goes again.
    """
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
            time.sleep(SETTLE_TIME)  # let the arm settle after a possible cancel

            # robot.execute() gives no usable success signal: the controller
            # reports SUCCEEDED even if the arm stopped short (no goal
            # tolerance configured), and MoveIt can drop a trajectory yet we
            # still log "Executed". So check where the end effector actually is.
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
    """
    Standalone node whose only job is holding the latest PoseStamped per
    color. Spun on a background thread via its own executor so that
    MoveItPy's blocking plan()/execute() calls on the main thread don't
    starve the subscription callbacks.
    """

    def __init__(self):
        super().__init__("pick_and_place_cube_sub")
        self._lock = threading.Lock()
        self._latest = {c: None for c in COLORS}
        for color in COLORS:
            self.create_subscription(
                PoseStamped,
                f"/detected_cube/{color}",
                self._make_callback(color),
                10,
            )

    def _make_callback(self, color):
        def cb(msg):
            with self._lock:
                self._latest[color] = msg
        return cb

    def get_latest(self, color):
        with self._lock:
            return self._latest[color]

    def wait_for_pose(self, color, timeout_sec=10.0, poll_period=0.1):
        elapsed = 0.0
        while elapsed < timeout_sec:
            pose = self.get_latest(color)
            if pose is not None:
                return pose
            time.sleep(poll_period)
            elapsed += poll_period
        return None


def main():
    rclpy.init()
    logger = get_logger("pick_and_place_demo")

    # --- perception subscriber, spun independently in the background ---
    cube_sub = CubePoseSubscriber()
    executor = MultiThreadedExecutor()
    executor.add_node(cube_sub)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # --- MoveItPy setup (mirrors grasp_isolation_test.py) ---
    robot = MoveItPy(node_name="pick_and_place_demo")
    time.sleep(2.0)
    manipulator = robot.get_planning_component(PLANNING_GROUP)
    gripper = robot.get_planning_component(GRIPPER_GROUP)
    robot_model = robot.get_robot_model()

    plan_params = make_plan_params(robot)

    # --- continuous-joint-safe, limit-clamped start-state snapshots ---
    # NOTE: continuous joints are deliberately NOT wrapped to +/-pi here.
    # MoveIt validates a trajectory's first point against the controller's
    # real joint state (tolerance 0.01 rad). Wrapping the start state by
    # 2*pi makes that check fail ("start point deviates ... at joint_3"),
    # and the trajectory is silently dropped.
    get_safe_start_state = make_safe_state_getter(
        robot, PLANNING_GROUP, logger,
        continuous_joints=[], joint_limits=JOINT_LIMITS)
    get_safe_gripper_state = make_gripper_state_getter(robot, logger)

    def move_home():
        for attempt in range(2):
            time.sleep(SETTLE_TIME)  # settle before snapshotting the start state
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
        # Single plan via seeded IK (not a raw pose goal) -- used only for
        # Home <-> pregrasp hops.
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

    for color in COLORS:
        logger.info(f"=== Processing {color} cube ===")

        pick_pose_msg = cube_sub.wait_for_pose(color, timeout_sec=10.0)
        if pick_pose_msg is None:
            logger.error(f"No detection received for '{color}' within timeout, skipping.")
            continue

        px = pick_pose_msg.pose.position.x
        py = pick_pose_msg.pose.position.y
        # PREGRASP_Z / GRASP_Z stay as the calibrated constants (mesh-offset
        # quirk); only (x, y) come from perception.

        place_x, place_y, place_z = PLACE_POSES[color]

        # Every step below is gated on the previous one succeeding; on any
        # failure the rest of this cube's sequence is aborted and the arm
        # returns home rather than computing the next goal from a wrong
        # assumption about where it is.

        if not move_to_pose(px, py, PREGRASP_Z, f"{color}: pregrasp"):
            logger.error(f"{color}: pregrasp planning failed, skipping this cube.")
            continue

        if not fine_move(px, py, PREGRASP_Z, px, py, GRASP_Z, f"{color}: descend to grasp"):
            logger.error(f"{color}: descent to grasp failed, skipping this cube.")
            move_home()
            continue

        if not set_gripper("Close"):
            logger.error(f"{color}: gripper close failed, returning home "
                         f"(cube was NOT grasped -- aborting rather than continuing empty-handed).")
            move_home()
            continue

        if not fine_move(px, py, GRASP_Z, px, py, PREGRASP_Z, f"{color}: lift to pregrasp"):
            logger.error(f"{color}: lift failed, returning home (still gripping).")
            move_home()
            continue

        # Horizontal transfer, held at pregrasp height the whole way.
        # Free-space motion, so use the larger TRANSFER_STEP.
        if not fine_move(px, py, PREGRASP_Z, place_x, place_y, PREGRASP_Z,
                         f"{color}: transfer to place", max_step=TRANSFER_STEP):
            logger.error(f"{color}: transfer failed, returning home (still gripping).")
            move_home()
            continue

        if not fine_move(place_x, place_y, PREGRASP_Z, place_x, place_y, place_z,
                         f"{color}: descend to place", tolerate_blocked_end=True):
            logger.error(f"{color}: descent to place failed, returning home (still gripping).")
            move_home()
            continue

        if not set_gripper("Open"):
            logger.error(f"{color}: gripper Open failed at place location -- cube may still be held.")

        if not fine_move(place_x, place_y, place_z, place_x, place_y, PREGRASP_Z,
                         f"{color}: retreat to pregrasp"):
            logger.error(f"{color}: retreat failed -- continuing to Home anyway.")

        move_home()

    logger.info("Pick-and-place sequence complete.")
    # os._exit skips normal interpreter/destructor teardown, avoiding the
    # known moveit_py C++ destructor segfault during spin-thread cleanup.
    os._exit(0)


if __name__ == "__main__":
    main()