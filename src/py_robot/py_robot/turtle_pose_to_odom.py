import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose as TurtlePose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler

class TurtleOdom(Node):
    def __init__(self):
        super().__init__('turtle_odom')
        self.sub = self.create_subscription(TurtlePose, '/turtle1/pose', self.callback, 10)
        self.pub = self.create_publisher(Odometry, '/filtered_odom', 10)

    def callback(self, msg: TurtlePose):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = msg.x
        odom.pose.pose.position.y = msg.y
        odom.pose.pose.position.z = 0.0

        # Orientation from theta
        q = quaternion_from_euler(0, 0, msg.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Velocity
        odom.twist.twist.linear.x = msg.linear_velocity
        odom.twist.twist.angular.z = msg.angular_velocity

        self.pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
