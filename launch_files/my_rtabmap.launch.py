import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions.opaque_function import OpaqueFunction  
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    return [
        # Declare whether to use simulation time:
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time if true'),

        # Static Transform Publisher: Publish an identity transform from "odom" to "dummy_link"
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_odom_to_dummy_link',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'dummy_link']
        ),

        # RTAB-Map SLAM node (from rtabmap_slam package)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                "frame_id": "dummy_link",
                "odom_frame_id": "odom",
                "map_frame_id": "map",
                "subscribe_rgb": True,
                "subscribe_depth": True,
                "subscribe_camera_info": True,
                "subscribe_imu": True,
                "subscribe_odom": False,
                "approx_sync": True,
                "RGBD/PublishOccupancyGrid": True,
                "Grid/FromDepth": True,
                "Grid/OccupancyGridResolution": 0.05,
                
		# (Optional) Adjust resolution and other occupancy grid parameters
       	"RGBD/OccupancyGridResolution": 0.05,   # meters per cell
       	"RGBD/MapRate": 1.0,  # Publishes the map once per second

        	"RGBD/OccupancyGridDepthRange": 5.0     # max depth for the grid
       	 # Add more if needed
                
            }],
            remappings=[
                ('/rgb/image', '/color/image_raw'),
                ('/depth/image', '/depth/image_rect_raw'),
                ('/rgb/camera_info', '/color/camera_info'),
                ('/imu', '/imu')
            ]
            # No namespace is specified.
        ),

        # RTAB-Map Visualization node (from rtabmap_viz package)
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                "frame_id": "dummy_link",
                "odom_frame_id": "odom",
                "map_frame_id": "map",
                "subscribe_rgb": True,
                "subscribe_depth": True,
                "approx_sync": True
            }],
            remappings=[
                ('/rgb/image', '/color/image_raw'),
                ('/depth/image', '/depth/image_rect_raw'),
                ('/rgb/camera_info', '/color/camera_info')
            ]
            # Visualization node will open its GUI window if a display is available.
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
