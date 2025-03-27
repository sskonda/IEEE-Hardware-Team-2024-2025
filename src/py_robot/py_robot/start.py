# when robot sees white light publish topic /go 
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class camera_flash_start_node(Node):
    def __init__(self):
        super().__init__('camera_flash_start_node')
        self.bridge = CvBridge()
        self.flash_detected = False
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10
        )
        self.go_pub = self.create_publisher(Bool, 'go', 10)
        self.get_logger().info("Waiting for camera flash trigger to start robot")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        brightness = np.mean(gray)

        if brightness > 240 and not self.flash_detected:
            self.flash_detected = True
            self.get_logger().info(" Bright flash. Starting robot")
            self.trigger_start_sequence()

    def trigger_start_signal(self):
        msg = Bool()
        msg.data = True
        self.go_pub.publish(msg)
        self.get_logger().info(" Published GO message to start.")
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = camera_flash_start_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


