import rclpy
from rclpy.qos import qos_profile_sensor_data, qos_profile_best_available
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Bool
import math
from nav_msgs.msg import Odometry


class DriveToPose(Node):
    def __init__(self):
        super().__init__('DriveToPose')

        # PARAMETERS 
        self.position_tolerance = self.declare_parameter("position_tolerance", 0.01).get_parameter_value().double_value
        self.angle_tolerance = self.declare_parameter("angle_tolerance", 0.01).get_parameter_value().double_value
        self.linear_gain = self.declare_parameter("max_speed", 0.5).get_parameter_value().double_value
        self.angular_gain = self.declare_parameter("max_angular_speed", 0.5).get_parameter_value().double_value

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_done_pub = self.create_publisher(Bool, '/goal_done', 1)

        initial_twist = Twist()
        initial_twist.linear.x = 0.0
        initial_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(initial_twist)
        self.get_logger().info("Published initial zero velocity on /cmd_vel")

        self.timer = self.create_timer(0.005, self.control_loop)

        self.goal_sub = self.create_subscription(Pose2D, '/goal_pose', self.set_goal, qos_profile=qos_profile_best_available)    
        self.filtered_odom = self.create_subscription(Odometry, '/filtered_odom', self.filtered_odom_callback, qos_profile=qos_profile_sensor_data)

        self.x_odom = 0.0
        self.y_odom = 0.0
        self.yaw_odom = 0.0
        self.prev_linear_x = 0.0
        self.prev_angular_z = 0.0
        self.smoothing_alpha = 0.1
        self.smoothing_position = 0.05

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.goal = None

    def control_loop(self):
        if self.goal is None or self.current_x == None or self.current_y == None or self.current_yaw == None:
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
            self.get_logger().info("DRIVING TO POINT")
            if abs(angle_error) < self.angle_tolerance:
                twist.linear.x = linear_speed
            twist.angular.z = self.angular_gain * angle_error

            twist.linear.x = self.smoothing_alpha * twist.linear.x + (1 - self.smoothing_alpha) * self.prev_linear_x
            twist.angular.z = 0.3 * twist.angular.z + 0.7 * self.prev_angular_z

            self.prev_linear_x = twist.linear.x
            self.prev_angular_z = twist.angular.z
        else:
            final_angle_error = self.normalize_angle(self.goal.theta - self.current_yaw)
            if abs(final_angle_error) > self.angle_tolerance:
                self.get_logger().info("AT POINT (ROTATING)")
                twist.angular.z = self.angular_gain * final_angle_error

                twist.linear.x = 0.0
                twist.angular.z = 0.3 * twist.angular.z + 0.7 * self.prev_angular_z

                self.prev_linear_x = twist.linear.x
                self.prev_angular_z = twist.angular.z

            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("Goal reached.")
                self.goal_done_pub.publish(Bool(data=True))
                self.goal = None

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
        self.get_logger().info(f" New goal received: ({msg.x:.2f}, {msg.y:.2f}, {math.degrees(msg.theta):.1f}°)")


def main(args=None):
    rclpy.init(args=args)
    node = DriveToPose()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Initial (x =0.79375, y=0.1524, z=0 )