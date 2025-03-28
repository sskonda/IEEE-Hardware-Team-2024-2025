import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Bool
from tf2_ros import LookupException
import math
from nav_msgs.msg import Odometry

from py_robot.pid import PID

class DriveToPose(Node):
    def __init__(self):
        super().__init__('DriveToPose')

        # PARAMETERS 
        self.declare_parameter("position_tolerance", 0.05)
        self.declare_parameter("angle_tolerance", 0.01)
        self.position_tolerance = self.get_parameter("position_tolerance").value
        self.angle_tolerance = self.get_parameter("angle_tolerance").value

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_done_pub = self.create_publisher(Bool, '/goal_done', 10)

        initial_twist = Twist()
        initial_twist.linear.x = 0.0
        initial_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(initial_twist)
        self.get_logger().info("Published initial zero velocity on /cmd_vel")

        self.timer = self.create_timer(0.1, self.control_loop)

        self.goal_sub = self.create_subscription(Pose2D, '/goal_pose', self.set_goal, 1)    
        self.filtered_odom = self.create_subscription(Odometry, '/filtered_odom', self.filtered_odom_callback, 1)

        self.enable_sub = self.create_subscription(Bool,'/enable',self.enable_callback, 10)
        self.enabled = False 

        self.x_odom = 0.0
        self.y_odom = 0.0
        self.yaw_odom = 0.0
        self.linear_gain = 1.0
        self.angular_gain = 1.5
        self.goal = Pose2D()
        self.prev_linear_x = 0.0
        self.prev_angular_z = 0.0
        self.smoothing_alpha = 0.1
        self.smoothing_position = 2.0

        self.goal_reached = False  

    def control_loop(self):
        if not self.enabled:
            return
        dx = self.goal.x - self.current_x
        dy = self.goal.y - self.current_y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.current_yaw)

        twist = Twist()
        max_linear_speed = self.linear_gain

        if distance < self.smoothing_position :
            linear_speed = max_linear_speed * (distance / self.smoothing_position)
        else:
            linear_speed = max_linear_speed

        if distance > self.position_tolerance:
            if abs(angle_error) < self.angle_tolerance:
                twist.linear.x = linear_speed
            twist.angular.z = self.angular_gain * angle_error
            self.goal_reached = False  # Still navigating

            twist.linear.x = self.smoothing_alpha * twist.linear.x + (1 - self.smoothing_alpha) * self.prev_linear_x
            twist.angular.z = 0.3 * twist.angular.z + 0.7 * self.prev_angular_z

            self.prev_linear_x = twist.linear.x
            self.prev_angular_z = twist.angular.z
        else:
            final_angle_error = self.normalize_angle(self.goal.theta - self.current_yaw)
            if abs(final_angle_error) > self.angle_tolerance:
                twist.angular.z = self.angular_gain * final_angle_error
                self.goal_reached = False

                twist.linear.x = 0.0
                twist.angular.z = 0.3 * twist.angular.z + 0.7 * self.prev_angular_z

                self.prev_linear_x = twist.linear.x
                self.prev_angular_z = twist.angular.z

            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                if not self.goal_reached:
                    self.get_logger().info("Goal reached.")
                    self.goal_done_pub.publish(Bool(data=True))
                    self.goal_reached = True

        self.cmd_vel_pub.publish(twist)

    def filtered_odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        from tf_transformations import euler_from_quaternion
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_yaw = yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def set_goal(self, msg: Pose2D):
        self.goal = msg
        self.goal_reached = False 
        self.get_logger().info(f" New goal received: ({msg.x:.2f}, {msg.y:.2f}, {math.degrees(msg.theta):.1f}°)")

    def enable_callback(self, msg: Bool):
        self.enabled = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = DriveToPose()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
