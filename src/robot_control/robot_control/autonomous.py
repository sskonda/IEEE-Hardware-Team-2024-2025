import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Pose2D
from goals import Goals
import math

class autonomous_node(Node):
    def __init__(self):
        super().__init__('autonomous_node')
    
        self.goal_pub = self.create_publisher(Pose2D, '/goal_pose', 10)
        self.lift_pub = self.create_publisher(Bool, '/lift', 10)
        self.clamp_pub = self.create_publisher(Float32, '/clamp', 10)
        self.beacon_pub = self.create_publisher(Float32, '/beacon', 10)
        self.intake_pub = self.create_publisher(Float32, '/intake', 10)

        self.goals_helper = Goals()# external goals function
        self.goals_helper.goal_pub = self.goal_pub
        self.goals_helper.lift = self.lift_pub
        self.goals_helper.clamp = self.clamp_pub
        self.goals_helper.beacon = self.beacon_pub
        self.goals_helper.intake = self.intake_pub
        self.goals_helper.get_logger = self.get_logger
        self.start_location= (31.25, 4.625, 0.0)

        self.goals = [
            self.goals_helper.goal_pose(0.79375, 0.254, 0.0),
            self.goals_helper.goal_pose(0.79375, 0.254, math.radians(180)),
            self.goals_helper.goal_pose(1.17375, 0.2032, math.radians(180)),
            self.goals_helper.clamp(0.0),  # release clamp
            self.goals_helper.goal_pose(0.5747625, 0.581552, 0.0),
            self.goals_helper.goal_pose(0.5747625, 0.581552, math.radians(-90)),
            self.goals_helper.goal_pose(0.5747625, 0.581552, math.radians(90)),
            self.goals_helper.goal_pose(0.5747625, 0.873125, math.radians(90)),
            self.goals_helper.clamp(0.0),
            self.goals_helper.clamp(.8),
            self.goals_helper.goal_pose(0.5747625, 0.873125, math.radian(0)),
            self.goals_helper.goals_pose(1.1811, 0.5715 ,math.radians(0) ),
            self.goals_helper.goals_pose(1.7907, 0.5715, math.radians(0)),
            self.goals_helper.goals_pose(1.7907, 0.5715, math.radians(90)),
            self.goals_helper.goals_pose(1.7907, 0.9652, math.radians(90)),
            self.goals_helper.goals_pose(1.7907, 0.9652, math.radians(90)),
            self.goals_helper.goals_pose(1.7907, 0.9652, math.radians(0)),
            self.goals_helper.goals_pose(2.159, 0.9652, math.radians(0)),
            self.goals_helper.goals_pose(2.159, 0.9652, math.radians(0)),
            self.goals_helper.goals_pose(2.159, 0.9652, math.radians(-90)),
            self.goals_helper.goals_pose(2.159, 0.1524, math.radians(-90)),
            self.goals_helper.goals_pose(2.159, 0.1524, math.radians(-90)),
            self.goals_helper.goals_pose(2.159, 0.1524, math.radians(90)),
            self.goals_helper.goals_pose(2.159, 0.5715, math.radians(180)),
            self.goals_helper.goals_pose(1.1811, 0.5715, math.radians(180)),
            self.goals_helper.goals_pose(1.1811, 0.5715, math.radians(90)),
            self.goals_helper.goals_pose(1.1811, 0.9652, math.radians(90)),
            self.goals_helper.goals_pose(1.1811, 0.9652, math.radians(180)),
            self.goals_helper.goals_pose(0.1524, 0.9652, math.radians(180)),
            self.goals_helper.goals_pose(0.1524, 0.9652, math.radians(-90)),
            self.goals_helper.goals_pose(0.1524, 0.6604, math.radians(-90)),
            self.goals_helper.goals_pose(0.1524, 0.6604, math.radians(0)),
            self.goals_helper.goals_pose(1.2192, 0.6604, math.radians(0)),
            self.goals_helper.goals_pose(1.2192, 0.6604, math.radians(-90)),
            self.goals_helper.goals_pose(1.2192 , 0.5715 ,math.radians(90)),
            self.goals_helper.goals_pose(1.2192 , 0.5715 ,math.radians(180)),
            self.goals_helper.goals_pose(0.1524 , 0.5715 ,math.radians(180)),
            self.goals_helper.goals_pose(0.1524 , 0.5715 ,math.radians(180)),
            self.goals_helper.goals_pose(0.1524 , 0.5715 ,math.radians(-90)),
            self.goals_helper.goals_pose(0.1524 , 0.4826 ,math.radians(-90)),
            self.goals_helper.goals_pose(0.1524 , 0.4826 ,math.radians(0)),
            self.goals_helper.goals_pose(1.2192 , 0.4826 ,math.radians(0)),
            self.goals_helper.goals_pose(1.2192 , 0.4826 ,math.radians(-90)),
            self.goals_helper.goals_pose(1.2192 , 0.4826 ,math.radians(-90)),
            self.goals_helper.goals_pose(1.2192 , 0.2032 ,math.radians(-90)),
            self.goals_helper.goals_pose(1.2192 , 0.2032 ,math.radians(180)),
            self.goals_helper.goals_pose(0.1524 , 0.2032 ,math.radians(180)),
        #beacon
            self.goals_helper.goals_pose( 0.5715 , 0.5715 ,math.radians(180)),
            self.goals_helper.goals_pose( 0.1524 , 0.5715 ,math.radians(180)),
            self.goals_helper.goals_pose( 0.1524 , 0.67818 ,math.radians(180))
        ]

        self.current_goal_index = 0
        self.goal_pub = self.create_publisher(Pose2D, '/goal_pose', 10)
        self.go_sub = self.create_subscription(Bool, 'go', self.go_callback, 10)
        self. get_logger().info("Wating for the GO")

        self.timer = None
        self.goal_sent = False
        self.active = False

    def go_callback(self, msg):
        if msg.data and not self.active:
            self.active = True
            self.get_logger().info(" GO signal received. Starting goal sequence") # ONCE THE GO LIGHT TURNS ON
            self.timer = self.create_timer(1.0, self.publish_next_goal)
    
    def publish_next_goal(self):
        if self.active:
            if self.current_goal_index >= len(self.goals):
                self.get_logger().info(" Goal sequence complete.")
                return
            action = self.goals[self.current_goal_index]
            action()
            self.waiting_for_completion = True
            self.get_logger().info(f"Waiting for goal {self.current_goal_index} to be done")

    def goal_done_callback(self, msg):
        if msg.data and self.waiting_for_completion:
            self.get_logger().info(f" Goal {self.current_goal_index} completed.")
            self.current_goal_index += 1
            self.waiting_for_completion = False
            self.publish_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = autonomous_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()




