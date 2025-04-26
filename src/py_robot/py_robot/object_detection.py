import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PointStamped, PoseStamped, PoseArray, Pose
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs.tf2_geometry_msgs as tf2_utils
from cv_bridge import CvBridge
import numpy as np
import cv2

class ObjectDetection(Node):
    def __init__(self):
        super().__init__("object_detection")

        self.fx = self.fy = self.cx = self.cy = None
        self.camera_info_received = False
        self.camera_frame = "camera_frame"
        self.map_frame = "map"

        self.calibration_coefficient = 5000

        # transform buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # camera info subscriber
        self.create_subscription(CameraInfo, "/camera/camera_info", self.camera_info_callback, 10)
        self.create_subscription(Image, "/camera/image_color_rect", self.image_callback, 10)

        self.robot_pose_pub = self.create_publisher(PoseStamped, "/robot/pose", 10)
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

    def calculate_distance(self, area):
        if area <= 0:
            return float('inf')
        return self.calibration_coefficient / np.sqrt(area)

    def image_to_camera_3d(self, x_pixel, y_pixel, distance):
        x = (x_pixel - self.cx) / self.fx * distance
        y = (y_pixel - self.cy) / self.fy * distance
        z = distance
        return np.array([x, y, z])

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_purple = np.array([120, 80, 40])
        upper_purple = np.array([170, 255, 255])
        mask = cv2.inRange(hsv, lower_purple, upper_purple)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
        poses = []
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            area = cv2.contourArea(contour)
            if area < 50:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            x_center = x + w // 2
            y_center = y + h // 2
            distance = self.calculate_distance(area)
            point_cam = self.image_to_camera_3d(x_center, y_center, distance)


            point_msg = PointStamped()
            point_msg.header.frame_id = self.camera_frame
            point_msg.header.stamp = msg.header.stamp
            point_msg.point.x, point_msg.point.y, point_msg.point.z = point_cam.tolist()

            point_world = self.tf_buffer.transform(point_msg, self.map_frame, timeout=rclpy.duration.Duration(seconds=0.5))

            pose = Pose()
            pose.position = point_world.point
            poses.append(pose)

        # first pose is the  robot
        robot_pose_msg = PoseStamped()
        robot_pose_msg.header.frame_id = self.map_frame
        robot_pose_msg.header.stamp = msg.header.stamp
        robot_pose_msg.pose = poses[0]
        self.robot_pose_pub.publish(robot_pose_msg)

        # purple nodes arer added to pose array 
        pose_array_msg = PoseArray()
        pose_array_msg.header = robot_pose_msg.header
        pose_array_msg.poses = poses[1:]
        self.purple_dots_pub.publish(pose_array_msg)

        self.get_logger().info(f"Published robot pose + {len(poses) - 1} purple dot(s).")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
