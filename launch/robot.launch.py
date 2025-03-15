import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_name = "my_robot"  # Change to your package name
    urdf_file_name = "my_robot.urdf"  # Ensure this matches your URDF filename

    urdf_file_path = os.path.join(
        get_package_share_directory(package_name), "urdf", urdf_file_name
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "debug", default_value="false", description="Enable debug mode"
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    get_package_share_directory(package_name), "rviz", "robot.rviz"
                ),
                description="Full path to RViz config file",
            ),
            LogInfo(
                condition=IfCondition(LaunchConfiguration("debug")),
                msg="Loading URDF file...",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": Command(["xacro ", urdf_file_path])}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        ]
    )
