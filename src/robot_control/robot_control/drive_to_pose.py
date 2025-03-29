from numpy import nonzero
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, qos_profile_best_available
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Bool
import math
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

def pos_neg(angle) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


class DriveToPose(Node):
    def __init__(self):
        super().__init__('DriveToPose')

        # PARAMETERS 
        self.position_tolerance = self.declare_parameter("position_tolerance", 0.01).get_parameter_value().double_value
        self.angle_tolerance = self.declare_parameter("angle_tolerance", 0.01).get_parameter_value().double_value
        self.linear_speed = self.declare_parameter("max_speed", 0.5).get_parameter_value().double_value
        self.angular_speed = self.declare_parameter("max_angular_speed", 0.333).get_parameter_value().double_value

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_done_pub = self.create_publisher(Bool, '/goal_done', 1)

        initial_twist = Twist()
        initial_twist.linear.x = 0.0
        initial_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(initial_twist)
        self.get_logger().info("Published initial zero velocity on /cmd_vel")

        self.timer = self.create_timer(0.001, self.control_loop)

        self.goal_sub = self.create_subscription(Pose2D, '/goal_pose', self.set_goal, qos_profile=qos_profile_best_available)    
        self.filtered_odom = self.create_subscription(Odometry, '/filtered_odom', self.filtered_odom_callback, qos_profile=qos_profile_sensor_data)

        self.smoothing_alpha = 0.6
        self.smoothing_position = 0.10
        self.smoothing_angle = math.pi / 6 

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.goal = None
        self._last_twist = Twist()
    
    def to_robot(self):
        return np.linalg.inv(self.to_world())
    
    def to_world(self):
        angle = -self.current_yaw
        return np.array([[np.cos(angle), np.sin(angle), self.current_x], [-np.sin(angle), np.cos(angle), self.current_y], [0, 0, 1]])

    def control_loop(self):
        twist = Twist()
        if self.goal is None or self.current_x == None or self.current_y == None or self.current_yaw == None:
            self.cmd_vel_pub.publish(twist)
            return

        if self.goal.x >= 0 and self.goal.y >= 0:
            error = (self.to_robot() @ np.array([self.goal.x, self.goal.y, 1]))[:2]
            distance = np.linalg.norm(error)
            if distance > self.position_tolerance:
                angle_error = math.atan2(error[1], error[0])
                
                twist.linear.x = np.copysign(self.linear_speed, error[0])
                if twist.linear.x > 0:
                    twist.angular.z = pos_neg(angle_error)
                else:
                    twist.angular.z = pos_neg(angle_error - math.pi)
                
                if distance < self.smoothing_position:
                    twist.linear.x *= distance / self.smoothing_position
            else:
                self.get_logger().info("Position reached.")
                self.goal_done_pub.publish(Bool(data=True))
                self.goal = None
                self._last_twist = Twist()
        else:
            angle_error = pos_neg(self.goal.theta - self.current_yaw)

            if abs(angle_error) > self.angle_tolerance:
                twist.linear.x = 0.0
                twist.angular.z = np.copysign(self.angular_speed, angle_error) 
                if abs(angle_error) > self.smoothing_angle:
                    twist.angular.z *= abs(angle_error) / self.smoothing_angle
            else:
                self.get_logger().info("Heading reached.")
                self.goal_done_pub.publish(Bool(data=True))
                self.goal = None
                self._last_twist = Twist()

        if twist.linear.x != 0.0:
            twist.linear.x = self.smoothing_alpha * twist.linear.x + (1 - self.smoothing_alpha) * self._last_twist.linear.x       
        if twist.angular.z != 0.0:
            twist.angular.z = self.smoothing_alpha * twist.angular.z + (1 - self.smoothing_alpha) * self._last_twist.angular.z

        self._last_twist = twist
        self.cmd_vel_pub.publish(twist)

    def filtered_odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_yaw = yaw

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
# For Backward drive, check if the cosine of the two vectors using the dot product is negative. 