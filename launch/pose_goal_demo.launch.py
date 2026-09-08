import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config"
        )
        .planning_pipelines("ompl", ["ompl"])
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("kinova_scripts"),
                "config",
                "moveit_cpp.yaml",
            )
        )
        .to_moveit_configs()
    )

    pose_goal_demo_node = Node(
        package="kinova_scripts",
        executable="pose_goal_demo",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    return LaunchDescription([pose_goal_demo_node])