import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config"
        )
        .robot_description_semantic(file_path="config/gen3.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])          # <-- OMPL only, matches grasp_isolation_test
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("kinova_scripts"),
                "config",
                "moveit_cpp.yaml",
            )
        )
        .to_moveit_configs()
    )

    pick_and_place_demo = Node(
        package="kinova_scripts",
        executable="pick_and_place_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},   # sim clock from Gazebo
        ],
    )

    return LaunchDescription([pick_and_place_demo])