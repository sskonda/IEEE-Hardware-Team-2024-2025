from psutil import sensors_temperatures
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, TwistWithCovariance, PoseWithCovariance, Vector3
from nav_msgs.msg import Odometry

import math

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # self.position_tolerance = self.declare_parameter("position_tolerance", 0.01).get_parameter_value().double_value
        # self.angle_tolerance = self.declare_parameter("angle_tolerance", 0.01).get_parameter_value().double_value
        # self.linear_gain = self.declare_parameter("max_speed", 0.5).get_parameter_value().double_value
        # self.angular_gain = self.declare_parameter("max_angular_speed", 0.5).get_parameter_value().double_value

        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', qos_profile=qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.publish_odom)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.heading = 0.0
        self.x = 0.0
        self.y = 0.0
        
        self.angular_velocity = 0.0
        self.vx = 0.0
        self.vy = 0.0

        self.latest_header = None
        self.imu_sub = self.create_subscription(Imu, '/sensors/imu', self._imu, qos_profile=qos_profile_sensor_data) 
        self._last_imu = None   
        self.odom_sub = self.create_subscription(Odometry, '/drive/odometry', self._odom, qos_profile=qos_profile_sensor_data)
        self._last_odom = None

    def _imu(self, msg: Imu):
        if self._last_imu is None:
            self._last_imu = msg
            return
        
        dt = msg.header.stamp.sec - self._last_imu.header.stamp.sec + (msg.header.stamp.nanosec - self._last_imu.header.stamp.nanosec) / 1_000_000_000
        self.heading += msg.angular_velocity.x * dt
        self.angular_velocity = msg.angular_velocity.x

        self._last_imu = msg
        self.publish_odom(msg.header)

    def _odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y

        self.publish_odom(msg.header)

    def publish_odom(self, header):
        header.frame_id = 'odom'
        self.latest_header = header

        self.odom_pub.publish(
            Odometry(
                header=header,
                child_frame_id="base_link",
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(
                            x=self.x,
                            y=self.y,
                            z=0.0,
                        ),
                        orientation=Quaternion(
                            x=0.0,
                            y=0.0,
                            z=math.sin(self.heading/2),
                            w=math.cos(self.heading/2),
                        )
                    )
                ),
                twist=TwistWithCovariance(
                    twist=Twist(
                        linear=Vector3(
                            x=self.vx,
                            y=self.vy,
                            z=0.0
                        ),
                        angular=Vector3(
                            x=0.0,
                            y=0.0,
                            z=self.angular_velocity
                        )
                    ),
                    covariance=[
                        0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.005, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.005, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.005,
                    ]
                )
            )
        )
    
    def timer_callback(self, *args):
        if self.latest_header is None:
            return
        
        # Now broadcast the TF
        t = TransformStamped()
        t.header = self.latest_header
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.heading/2)
        t.transform.rotation.w = math.cos(self.heading/2)

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusion()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Initial (x =0.79375, y=0.1524, z=0 )