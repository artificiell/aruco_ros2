import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Launch configuration
    display_markers = LaunchConfiguration('display_markers')

    # Launch arguments
    display_markers_arg = DeclareLaunchArgument(
        'display_markers',
        default_value = 'False'
    )

    zed_camera_node = Node(
        package = 'zed_ros2',
        executable = 'zed_node',
        name = 'minimal_zed_node',
        parameters=[{
            'resolution': 'HD1080'
        }]
    )
    
    aruco_detection_node = Node(
        package = 'aruco_ros2',
        executable = 'aruco_detection',
        name = 'aruco_detection_node',
        parameters=[{
            'marker_size': 0.10,
            'aruco_dictionary_id': 'DICT_ARUCO_ORIGINAL',
            'image_topic': '/zed/image',
            'camera_info_topic': '/zed/info',
            'display_markers': LaunchConfiguration('display_markers')
        }]
    )

    aruco_transform_node = Node(
        package = 'aruco_ros2',
        executable='aruco_transform',
        name = 'aruco_transform_node',
        parameters=[{
            'scalar_x': 1.0,
            'scalar_y': 1.0,
            'scalar_z': 1.0
        }]

    )

    return LaunchDescription([
        display_markers_arg,
        zed_camera_node,
        aruco_detection_node,
        aruco_transform_node
    ])
