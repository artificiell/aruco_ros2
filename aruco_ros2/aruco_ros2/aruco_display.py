import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class ArucoDisplay(Node):
    def __init__(self):
        super().__init__('aruco_display_node')

        # Set up subscribers
        self.create_subscription(
            Image,
            'aruco/image',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

    # Get and display marker image
    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('Aruco Markers', image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f"Bridge error: {e}")
        
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = ArucoDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node (explicitly)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
