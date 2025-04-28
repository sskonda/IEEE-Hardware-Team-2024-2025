import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

SCALE = 2

class ThresholdCalibrator(Node):
    def __init__(self):
        super().__init__('threshold_calibrator')

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.current_frame = None
        self.hsv_frame = None

        # Initialize thresholds to extreme values
        self.lower = np.array([255, 255, 255])
        self.upper = np.array([0, 0, 0])

        self.timer = self.create_timer(0.01, self.timer_callback)
        
        cv2.namedWindow('preview')
        cv2.namedWindow('mask')
        cv2.setMouseCallback('preview', self.click_event)

    def timer_callback(self):
        if self.hsv_frame is None:
            return

        self.display_image()
    
    def click_event(self, event, x, y, flags, param):
        if self.hsv_frame is None:
            return
        
        # Since we're resizing the image for preview, adjust click coordinates
        x = int(x * SCALE)
        y = int(y * SCALE)

        pixel = self.hsv_frame[y, x]

        if event == cv2.EVENT_LBUTTONDOWN or (event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_LBUTTON):
            self.get_logger().info(f"Left click: Incorporate {pixel}")
            self.lower = np.minimum(self.lower, pixel)
            self.upper = np.maximum(self.upper, pixel)

        elif event == cv2.EVENT_MBUTTONDOWN:
            pixel = self.hsv_frame[y, x]
            self.get_logger().info(f"Middle click: Exclude {pixel}")

            best_channel = None
            best_adjustment = None
            best_direction = None

            for i in range(3):  # H, S, V channels
                if self.lower[i] <= pixel[i] <= self.upper[i]:
                    dist_to_lower = abs(pixel[i] - self.lower[i])
                    dist_to_upper = abs(self.upper[i] - pixel[i])

                    if best_adjustment is None or min(dist_to_lower, dist_to_upper) < best_adjustment:
                        best_adjustment = min(dist_to_lower, dist_to_upper)
                        best_channel = i
                        best_direction = 'lower' if dist_to_lower < dist_to_upper else 'upper'

            if best_channel is not None:
                if best_direction == 'lower':
                    self.lower[best_channel] = min(pixel[best_channel] + 1, 255)
                    self.get_logger().info(f"Adjusted lower[{best_channel}] to {self.lower[best_channel]}")
                else:
                    self.upper[best_channel] = max(pixel[best_channel] - 1, 0)
                    self.get_logger().info(f"Adjusted upper[{best_channel}] to {self.upper[best_channel]}")
            else:
                self.get_logger().info("Pixel already excluded; no adjustment needed.")
        
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.get_logger().info("Right click: Reset thresholds")
            self.lower = np.array([255, 255, 255])
            self.upper = np.array([0, 0, 0])

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.hsv_frame = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)

    def display_image(self):
        if self.hsv_frame is None:
            return

        if np.any(self.lower != self.upper):
            mask = cv2.inRange(self.hsv_frame, self.lower, self.upper)
            preview = cv2.bitwise_and(self.current_frame, self.current_frame, mask=cv2.bitwise_not(mask))
        else:
            mask = np.zeros((self.current_frame.shape[0], self.current_frame.shape[1]), dtype=np.uint8)
            preview = self.current_frame.copy()

        # Resize both images
        preview_small = cv2.resize(preview, (0, 0), fx=1/SCALE, fy=1/SCALE)
        mask_small = cv2.resize(mask, (0, 0), fx=1/SCALE, fy=1/SCALE)

        cv2.imshow('preview', preview_small)
        cv2.imshow('mask', mask_small)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info(f"Final thresholds: lower={self.lower.tolist()}, upper={self.upper.tolist()}")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ThresholdCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
