import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the share directory of your package where the YAML file is stored.
    pkg_share = get_package_share_directory('my_robot')
    params_file = os.path.join(pkg_share, 'config', 'realsense_frames.yaml')
    
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            output='screen',
            parameters=[params_file]
        )
    ])
