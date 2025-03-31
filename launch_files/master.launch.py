import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the share directory for your package
    pkg_share = get_package_share_directory('my_robot')
    
    # Paths to your existing launch files
    realsense_launch = os.path.join(pkg_share, 'launch', 'my_realsense.launch.py')
    rtabmap_launch = os.path.join(pkg_share, 'launch', 'my_rtabmap.launch.py')
    nav2_launch = os.path.join(pkg_share, 'launch', 'nav2_bringup.launch.py')
    robot_launch = os.path.join(pkg_share, 'launch', 'robot.launch.py')

    return LaunchDescription([
        # Launch the robot description (URDF, state publishers, and RViz)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch)
        ),
        # Launch the RealSense camera node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch)
        ),
        # Launch RTAB‑Map (SLAM) node with updated memory parameters
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),
            launch_arguments={'use_sim_time': 'false'}.items()
        ),
        # Launch the Nav2 bringup (with your nav2_params.yaml)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch)
        )
    ])
