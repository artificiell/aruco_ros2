import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    aruco_params = os.path.join(
        get_package_share_directory('aruco_ros2'),
        'config',
        'aruco_parameters.yaml'
    )

    aruco_detection_node = Node(
        package = 'aruco_ros2',
        executable = 'aruco_detection',
        name = 'aruco_detection_node',
        parameters = [aruco_params]
    )

    return LaunchDescription([
        aruco_detection_node
    ])
