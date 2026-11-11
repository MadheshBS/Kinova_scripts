import os
import random

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

MODELS_DIR = os.path.expanduser("~/kinova_ws/src/kinova_scripts/models")
CONFIG_DIR = os.path.expanduser("~/kinova_ws/src/kinova_scripts/config")
MOVEIT_CPP_YAML = os.path.join(CONFIG_DIR, "moveit_cpp.yaml")

# Random spawn region (base_link, metres). Stops at x=0.48 to stay clear of
# the place markers at x=0.55.
SPAWN_X_RANGE = (0.30, 0.48)
SPAWN_Y_RANGE = (-0.20, 0.20)
SPAWN_Z = 0.42

CUBE_MODELS = {
    "red": "simple_cube",
    "green": "simple_cube_green",
    "blue": "simple_cube_blue",
}


def generate_launch_description():
    # Re-rolled on every `ros2 launch`.
    color = random.choice(list(CUBE_MODELS))
    cube_x = round(random.uniform(*SPAWN_X_RANGE), 3)
    cube_y = round(random.uniform(*SPAWN_Y_RANGE), 3)

    kortex_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("kortex_bringup"),
                         "launch", "kortex_sim_control.launch.py")
        ),
        launch_arguments={
            "robot_type": "gen3",
            "dof": "7",
            "gripper": "robotiq_2f_85",
        }.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("kinova_gen3_7dof_robotiq_2f_85_moveit_config"),
                         "launch", "move_group.launch.py")
        )
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("kinova_gen3_7dof_robotiq_2f_85_moveit_config"),
                         "launch", "moveit_rviz.launch.py")
        )
    )

    spawn_table = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create",
             "-file", os.path.join(MODELS_DIR, "simple_table", "model.sdf"),
             "-name", "my_table", "-x", "0.4", "-y", "0.0", "-z", "0.2"],
        output="screen",
    )

    spawn_pedestal = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create",
             "-file", os.path.join(MODELS_DIR, "robot_pedestal", "model.sdf"),
             "-name", "robot_pedestal", "-x", "0.0", "-y", "0.0", "-z", "0.15"],
        output="screen",
    )

    spawn_camera = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create",
             "-file", os.path.join(MODELS_DIR, "overhead_camera", "model.sdf"),
             "-name", "overhead_camera",
             "-x", "0.4", "-y", "0.0", "-z", "1.5",
             "-R", "0", "-P", "1.5708", "-Y", "0"],
        output="screen",
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p",
                   f"config_file:={os.path.join(CONFIG_DIR, 'camera_bridge.yaml')}"],
        output="screen",
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0.4", "--y", "0.0", "--z", "1.5",
                   "--qx", "0.7071", "--qy", "-0.7071", "--qz", "0", "--qw", "0",
                   "--frame-id", "base_link",
                   "--child-frame-id", "overhead_camera/camera_link/overhead_rgbd"],
        output="screen",
    )

    spawn_cube = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create",
             "-file", os.path.join(MODELS_DIR, CUBE_MODELS[color], "model.sdf"),
             "-name", f"cube_{color}",
             "-x", str(cube_x), "-y", str(cube_y), "-z", str(SPAWN_Z)],
        output="screen",
    )

    color_detector = Node(
        package="kinova_scripts",
        executable="cube_color_detector",
        output="screen",
    )

    # MoveItPy needs the full MoveIt config as node parameters; without it
    # you get "Failed to load planning pipelines from parameter server".
    moveit_config = (
        MoveItConfigsBuilder(
            "gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(file_path=MOVEIT_CPP_YAML)
        .to_moveit_configs()
    )

    pick_and_place = Node(
        package="kinova_scripts",
        executable="pick_and_place_single",
        name="pick_and_place_single",  # must match MoveItPy(node_name=...)
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    return LaunchDescription([
        LogInfo(msg=f"[random cube] colour={color}  x={cube_x}  y={cube_y}"),
        kortex_sim,
        TimerAction(period=8.0, actions=[move_group]),
        TimerAction(period=10.0, actions=[moveit_rviz]),
        TimerAction(period=12.0, actions=[spawn_table, spawn_pedestal]),
        TimerAction(period=14.0, actions=[spawn_camera, camera_bridge, static_tf]),
        TimerAction(period=17.0, actions=[spawn_cube]),
        TimerAction(period=19.0, actions=[color_detector]),
        # Start the pick-and-place once the detector is publishing.
        TimerAction(period=25.0, actions=[pick_and_place]),
    ])