from setuptools import setup
import os
from glob import glob

package_name = 'ros2_aruco'

setup(
    name=package_name,
    version='0.1.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andreas Persson',
    maintainer_email='andreas.persson@oru.se',
    description='ROS2 ArUco Marker package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detection = ros2_aruco.aruco_detection:main',
            'aruco_generation = ros2_aruco.aruco_generation:main',
            'aruco_transform = ros2_aruco.aruco_transform:main',
            'aruco_affirm = ros2_aruco.aruco_affirm:main',
            'aruco_display = ros2_aruco.aruco_display:main'
        ],
    },
)
