import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    aruco_detection_node = Node(
        package = 'ros2_aruco',
        executable = 'aruco_detection',
        name = 'aruco_detection_node',
        parameters=[{
            'marker_size': 0.10,
            'aruco_dictionary_id': 'DICT_ARUCO_ORIGINAL',
            'image_topic': '/zed/zed_node/rgb/image_rect_color',
            'camera_info_topic': '/zed/zed_node/rgb/camera_info'
        }]
    )

    aruco_transform_node = Node(
        package = 'ros2_aruco',
        executable='aruco_transform',
        name = 'aruco_transform_node',
        parameters=[{
            'scalar_x': 1.0,
            'scalar_y': 1.2,
            'scalar_z': 1.0
        }]

    )
    
    return LaunchDescription([
        aruco_detection_node,
        aruco_transform_node
    ])
