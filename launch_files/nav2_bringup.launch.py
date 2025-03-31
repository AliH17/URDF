import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to your robot's URDF and nav2 parameter file
    pkg_share = get_package_share_directory('my_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # Get the nav2_bringup package directory (provided by Nav2)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        # Include your robot launch file (publishes the robot_description, TF, etc.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'robot.launch.py'))
        ),
        # Include Nav2 bringup, and disable its RViz instance with use_rviz:=false
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': nav2_params, 'use_rviz': 'false'}.items(),
        )
    ])
