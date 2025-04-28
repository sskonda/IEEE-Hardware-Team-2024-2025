import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data, qos_profile_best_available

class DriveToPath(Node):
    def __init__(self):
        super().__init__('drive_to_path')

        # PARAMETERS 
        self.position_tolerance = self.declare_parameter("position_tolerance", 0.05).get_parameter_value().double_value
        self.angle_tolerance = self.declare_parameter("angle_tolerance", 0.5).get_parameter_value().double_value
        self.linear_gain = self.declare_parameter("max_speed", 0.05).get_parameter_value().double_value
        self.angular_gain = self.declare_parameter("max_angular_speed", 0.5).get_parameter_value().double_value

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_done_pub = self.create_publisher(Bool, '/goal_done', 1)

        self.timer = self.create_timer(0.005, self.control_loop)

        self.path_sub = self.create_subscription(PoseArray, '/ordered_path', self.path_callback, qos_profile_best_available)
        self.filtered_odom = self.create_subscription(Odometry, '/filtered_odom', self.filtered_odom_callback, qos_profile_sensor_data)

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.path = None
        self.current_goal_idx = 0

        self.prev_linear_x = 0.0
        self.prev_angular_z = 0.0
        self.smoothing_alpha = 0.1
        self.smoothing_position = 0.05

        self.drive_straight = 0


    def path_callback(self, msg: PoseArray):
        if self.path == None:
            self.path = []
            for pose in msg.poses:
                pose2d = Pose2D()
                pose2d.x = pose.position.x
                pose2d.y = pose.position.y
                _, _, yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                pose2d.theta = yaw
                self.path.append(pose2d)
            self.get_logger().info(f"Received path with {len(self.path)} waypoints.")
            self.current_goal_idx = 0

    def control_loop(self):
        if self.drive_straight > 0:
            self.drive_straight -= 1
            twist = Twist()
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        if self.path is None:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.2
            self.cmd_vel_pub.publish(twist)
            return

        goal = self.path[self.current_goal_idx]

        dx = goal.x - self.current_x
        dy = goal.y - self.current_y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)

        angle_error = self.normalize_angle(target_angle - self.current_yaw)

        robot_heading_x = math.cos(self.current_yaw)
        robot_heading_y = math.sin(self.current_yaw)

        to_goal_x = dx
        to_goal_y = dy
        norm = math.hypot(to_goal_x, to_goal_y)
        if norm != 0:
            to_goal_x /= norm
            to_goal_y /= norm
        dot_product = robot_heading_x * to_goal_x + robot_heading_y * to_goal_y
        drive_backward = dot_product < 0

        if drive_backward:
            target_angle = self.normalize_angle(target_angle + math.pi)

        twist = Twist()
        max_linear_speed = self.linear_gain

        if distance < self.smoothing_position:
            linear_speed = max_linear_speed * (distance / self.smoothing_position)
        else:
            linear_speed = max_linear_speed

        if distance > self.position_tolerance:
            if abs(angle_error) < self.angle_tolerance:
                twist.linear.x = -linear_speed if drive_backward else linear_speed
            twist.angular.z = self.angular_gain * angle_error

            twist.linear.x = self.smoothing_alpha * twist.linear.x + (1 - self.smoothing_alpha) * self.prev_linear_x
            twist.angular.z = 0.3 * twist.angular.z + 0.7 * self.prev_angular_z

            self.prev_linear_x = twist.linear.x
            self.prev_angular_z = twist.angular.z

        else:
            final_angle_error = self.normalize_angle(goal.theta - self.current_yaw)
            if abs(final_angle_error) > self.angle_tolerance:
                twist.angular.z = self.angular_gain * final_angle_error
                if abs(twist.angular.z) < 0.5:
                    twist.angular.z = 0.5 * (1 if twist.angular.z > 0 else -1)
                twist.linear.x = 0.0

                twist.angular.z = 0.3 * twist.angular.z + 0.7 * self.prev_angular_z

                self.prev_linear_x = twist.linear.x
                self.prev_angular_z = twist.angular.z

            else:
                self.get_logger().info(f"Waypoint {self.current_goal_idx+1}/{len(self.path)} reached.")
                self.current_goal_idx += 1
                self.drive_straight = 100
                if self.current_goal_idx >= len(self.path):
                    self.get_logger().info("Path complete.")
                    self.path = None
                    self.goal_done_pub.publish(Bool(data=True))
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

    def filtered_odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_yaw = yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = DriveToPath()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Needs path, needs listener to command velocity, path to follow. 