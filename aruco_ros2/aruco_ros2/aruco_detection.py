
"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco/poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco/markers (aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to
                         (default /camera/camera_info)

Author: Nathan Sprague
Version: 10/26/2020

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
from packaging.version import Version
import tf_transformations
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from aruco_interfaces.msg import ArucoMarker, ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class ArucoDetection(Node):
    def __init__(self):
        super().__init__("aruco_detection_node")

        # Declare and read parameters
        self.declare_parameter(
            name = "marker_size",
            value = 0.0625,
            descriptor = ParameterDescriptor(
                type = ParameterType.PARAMETER_DOUBLE,
                description = "Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name = "aruco_dictionary_id",
            value = "DICT_ARUCO_ORIGINAL",
            descriptor = ParameterDescriptor(
                type = ParameterType.PARAMETER_STRING,
                description = "Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name = "image_topic",
            value = "/camera/image_raw",
            descriptor = ParameterDescriptor(
                type = ParameterType.PARAMETER_STRING,
                description = "Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name = "camera_info_topic",
            value = "/camera/camera_info",
            descriptor = ParameterDescriptor(
                type = ParameterType.PARAMETER_STRING,
                description = "Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name = "camera_frame",
            value = "",
            descriptor = ParameterDescriptor(
                type = ParameterType.PARAMETER_STRING,
                description = "Camera optical frame to use.",
            ),
        )

        self.declare_parameter(
            name = "display_markers",
            value = False,
            descriptor = ParameterDescriptor(
                type = ParameterType.PARAMETER_BOOL,
                description = "Display detected markers (or not).",
            ),
        )
        
        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {dictionary_id_name}")

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image topic: {image_topic}")

        info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image info topic: {info_topic}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )
        
        self.display_markers = (
            self.get_parameter("display_markers").get_parameter_value().bool_value
        )
        self.get_logger().info(f"Markers displayed: {self.display_markers}")
        
        # Make sure we have a valid dictionary id:
        self.get_logger().info(f"OpenCV version: {cv2.__version__}")
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(dictionary_id_name)
            )
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data
        )
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.bridge = CvBridge()
        
        # Set up publishers
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco/markers/original", qos_profile_sensor_data)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # Set up aruco detector
        if Version(cv2.__version__) > Version("4.7.0"):
            aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            aruco_parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(aruco_dictionary, aruco_parameters)
        else:
            self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
            self.aruco_parameters = cv2.aruco.DetectorParameters_create()
            
    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        image = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        markers = ArucoMarkers()
        markers.markers = []
        
        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
        markers.header.stamp = img_msg.header.stamp

        # Detect markers
        if Version(cv2.__version__) > Version("4.7.0"):
            corners, marker_ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, marker_ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dictionary, parameters = self.aruco_parameters
            )

        # Process each detected marker
        if marker_ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.intrinsic_mat, self.distortion
            )
            for i, marker_id in enumerate(marker_ids):
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                marker = ArucoMarker()
                marker.id = int(marker_id[0])
                marker.pose = pose
                markers.markers.append(marker)

        # Publish markers
        self.markers_pub.publish(markers)

        # Draw and display markers 
        if self.display_markers:
            try:
                cv2.aruco.drawDetectedMarkers(image, corners)
                cv2.imshow('Aruco Markers', image)
                cv2.waitKey(1)
            except Expection as e:
                self.get_logger().error(f"Error displaying markers: {e}")            

# Main function
def main(args = None):
    rclpy.init(args = args)
    node = ArucoDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
