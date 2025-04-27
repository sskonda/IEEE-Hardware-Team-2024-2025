import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PointStamped, PoseStamped, PoseArray, Pose
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs.tf2_geometry_msgs as tf2_utils
from cv_bridge import CvBridge
import numpy as np
import cv2

M = 0.00048386
B = 0.00429777

class ObjectDetection(Node):
    def __init__(self, debug=False):
        super().__init__("object_detection")

        self.fx = self.fy = self.cx = self.cy = None
        self.camera_info_received = False
        self.camera_frame = "camera_frame"
        self.map_frame = "base_link"

        # transform buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # camera info subscriber
        self.create_subscription(CameraInfo, "/camera/camera_info", self.camera_info_callback, 10)
        self.create_subscription(Image, "/camera/image_rect", self.image_callback, 10)

        self.debug = debug
        if self.debug:
            self.debug_pub = self.create_publisher(Image, "/camera/debug", 10)
        self.purple_dots_pub = self.create_publisher(PoseArray, "/purple_dots", 10)

        self.bridge = CvBridge()

    # intrinsics 
    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_frame = msg.header.frame_id
            self.camera_info_received = True

    def calculate_distance(self, radius):
        if radius <= 0:
            return float('inf')
        return 0.01 / (M * radius + B)

    def image_to_camera_3d(self, x_pixel, y_pixel, distance):
        x = (x_pixel - self.cx) / self.fx * distance
        y = (y_pixel - self.cy) / self.fy * distance
        z = distance
        return np.array([x, y, z])
    
    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        height, width, depth = frame.shape

        if self.debug:
            debug_frame = np.zeros((height*3, width, depth), dtype=frame.dtype)
            debug_frame[0*height:1*height] = frame

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_purple = np.array([113, 195, 150])
        upper_purple = np.array([127, 255, 255])
        mask = cv2.inRange(hsv, lower_purple, upper_purple)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=4)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        if self.debug:
            debug_frame[1*height:2*height][mask[:, :] == 255] = frame[mask[:, :] == 255]
            debug_frame[2*height:3*height] = frame

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        poses = []

        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            area = cv2.contourArea(contour)
            if area < 50:
                self.get_logger().warn(f"Contour area too small: {area}")
                return

            (x, y), r = cv2.minEnclosingCircle(contour)

            if self.debug:
                debug_frame[2*height:3*height] = cv2.circle(debug_frame[2*height:3*height], (int(x), int(y)), int(r), (0, 255, 0), 2)

            distance = self.calculate_distance(r)
            point_cam = self.image_to_camera_3d(x, y, distance)

            point_msg = PointStamped()
            point_msg.header.frame_id = self.camera_frame
            point_msg.header.stamp = msg.header.stamp
            point_msg.point.x, point_msg.point.y, point_msg.point.z = point_cam.tolist()
            
            point_world = self.tf_buffer.transform(point_msg, self.map_frame, timeout=Duration(seconds=2.5))

            pose = Pose()
            pose.position = point_world.point
            poses.append(pose)        # purple nodes arer added to pose array 

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "base_link"
        pose_world = self.tf_buffer.transform(pose, self.map_frame, timeout=Duration(seconds=2.5))

        poses.insert(0, pose_world.pose)

        pose_array_msg = PoseArray()
        pose_array_msg.header = msg.header
        pose_array_msg.header.frame_id = self.map_frame
        pose_array_msg.poses = poses

        if self.debug:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8"))
        self.purple_dots_pub.publish(pose_array_msg)

        # self.get_logger().info(f"Published {len(poses)} purple dot(s).")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection(debug=True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
