import rclpy
from typing import cast

from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseWithCovariance, Pose, TransformStamped
from std_msgs.msg import Header, Int32
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from tf2_msgs.msg import TFMessage

class CustomTransformListener(TransformListener):
    def __init__(self, buffer: Buffer, node: Node, callback, **kwargs):
        super().__init__(buffer, node, **kwargs)
        self.extra_callback = callback

    def callback(self, data: TFMessage):
        super().callback(data)
        self.extra_callback(data)


class TagPosePublisher(Node):
    def __init__(self):
        super().__init__('tag_pose_publisher')
        
        self.covariance = self.declare_parameter('covariance', [0.0 for i in range(36)]).get_parameter_value().double_array_value

        static_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        detection_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        self.pub_bin_drop_id = self.create_publisher(Int32, "/sensors/bin_drop_id", static_qos)
        self.sub_detections = self.create_subscription(AprilTagDetectionArray, "/camera/detections", self._process_detections, detection_qos)
        self.pub_tag_pose = [
            self.create_publisher(PoseWithCovarianceStamped, f"/sensors/tag_{n}", qos_profile_sensor_data)
            for n in range(8)
        ]

        self._tf_buffer = Buffer()
        self._tf_listener = CustomTransformListener(self._tf_buffer, self, self._process_transform)
        self._last_header = None

        self.bin_drop_id_message = Int32(data=-1)

    def _process_transform(self, msg: TFMessage):
        for transform in msg.transforms:
            if not transform.child_frame_id.startswith('detected_tag:'):
                continue

            n = int(transform.child_frame_id.split(':')[1])
            transform = cast(TransformStamped, transform)

            robot_pose_local = PoseWithCovarianceStamped(
                    header=transform.header,
                )
            robot_pose_local.header.frame_id = 'base_link'
            
            try:
                robot_pose_msg = self._tf_buffer.transform(robot_pose_local, f'detected_tag:{n}')
                robot_pose_msg.header.frame_id = f'tag:{n}'

                robot_map_pose = self._tf_buffer.transform(robot_pose_msg, 'map')
                self.pub_tag_pose[n].publish(robot_map_pose)
            except:
                pass

    def _process_detections(self, msg: AprilTagDetectionArray):
        for detection in cast(list[AprilTagDetection], msg.detections):
            n = detection.id
            if n in (0, 1, 2, 3, 4):
                self.bin_drop_id_message = Int32(data=n)
            
        self.pub_bin_drop_id.publish(self.bin_drop_id_message)
                    

def main(args=None):
    rclpy.init(args=args)

    tag_publisher = TagPosePublisher()

    rclpy.spin(tag_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tag_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
