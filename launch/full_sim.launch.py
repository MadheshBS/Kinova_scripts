import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Models/config are read from the source tree directly (matching your manual
# commands), not from an installed share dir -- adjust if you later move
# these into kinova_scripts' install/share via CMakeLists/setup.py.
MODELS_DIR = os.path.expanduser("~/kinova_ws/src/kinova_scripts/models")
CONFIG_DIR = os.path.expanduser("~/kinova_ws/src/kinova_scripts/config")


def generate_launch_description():
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

    spawn_cube_red = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create",
             "-file", os.path.join(MODELS_DIR, "simple_cube", "model.sdf"),
             "-name", "cube_red", "-x", "0.4", "-y", "0.1", "-z", "0.42"],
        output="screen",
    )
    spawn_cube_green = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create",
             "-file", os.path.join(MODELS_DIR, "simple_cube_green", "model.sdf"),
             "-name", "cube_green", "-x", "0.4", "-y", "0.0", "-z", "0.42"],
        output="screen",
    )
    spawn_cube_blue = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create",
             "-file", os.path.join(MODELS_DIR, "simple_cube_blue", "model.sdf"),
             "-name", "cube_blue", "-x", "0.4", "-y", "-0.1", "-z", "0.42"],
        output="screen",
    )

    color_detector = Node(
        package="kinova_scripts",
        executable="cube_color_detector",
        output="screen",
    )

    # Fixed-delay staging: Gazebo + ros2_control + robot spawn need time to
    # come up before move_group/RViz/entity spawns are safe to fire. These
    # delays are a starting guess -- if you see spawn failures or move_group
    # race errors, bump the relevant TimerAction period up.
    return LaunchDescription([
        kortex_sim,
        TimerAction(period=8.0, actions=[move_group]),
        TimerAction(period=10.0, actions=[moveit_rviz]),
        TimerAction(period=12.0, actions=[spawn_table]),
        TimerAction(period=14.0, actions=[spawn_camera, camera_bridge, static_tf]),
        TimerAction(period=17.0, actions=[spawn_cube_red, spawn_cube_green, spawn_cube_blue]),
        TimerAction(period=19.0, actions=[color_detector]),
    ])