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
        .robot_description(file_path="config/gen3.urdf.xacro")
        .robot_description_semantic(file_path="config/gen3.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])          # <-- OMPL only, matches yaml above
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("kinova_scripts"),
                "config",
                "moveit_cpp.yaml",
            )
        )
        .to_moveit_configs()
    )

    grasp_isolation_test = Node(
        package="kinova_scripts",
        executable="grasp_isolation_test",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},   # important: no /clock publisher for this isolation test
        ],
    )

    return LaunchDescription([grasp_isolation_test])