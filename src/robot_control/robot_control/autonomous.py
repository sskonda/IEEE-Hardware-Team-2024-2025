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
            self.goals_helper.goal_pose(31.25, 10.0, 0.0),
            self.goals_helper.goal_pose(31.25, 10.0, math.radians(180)),
            self.goals_helper.goal_pose(46.25, 8.0, math.radians(180)),
            self.goals_helper.clamp(0.0),  # release clamp
            self.goals_helper.goal_pose(22.63, 22.88, 0.0),
            self.goals_helper.goal_pose(22.63, 22.88, math.radians(-90)),
            self.goals_helper.goal_pose(22.63, 22.88, math.radians(90)),
            self.goals_helper.goal_pose(22.63, 34.375, math.radians(90)),
            self.goals_helper.clamp(0.0),
            self.goals_helper.clamp(.8),
            self.goals_helper.goal_pose(22.63, 34.375, math.radian(0)),
            self.goals_helper.goals_pose(46.5, 22.5 ,math.radians(0) ),
            self.goals_helper.goals_pose(70.5, 22.5, math.radians(0)),
            self.goals_helper.goals_pose(70.5, 22.5, math.radians(90),
            self.goals_helper.goals_pose(70.5, 38, math.radians(90)),
            self.goals_helper.goals_pose(70.5, 38, math.radians(90)),
            self.goals_helper.goals_pose( 70.5, 38, math.radians(0)),
            self.goals_helper.goals_pose( 85, 38, math.radians(0)),
            self.goals_helper.goals_pose( 85, 38, math.radians(0)),
            self.goals_helper.goals_pose( 85, 38, math.radians(-90)),
            self.goals_helper.goals_pose( 85, 6, math.radians(-90)),
            self.goals_helper.goals_pose( 85, 6, math.radians(-90)),
            self.goals_helper.goals_pose( 85, 6, math.radians(90)),
            self.goals_helper.goals_pose( 85, 22.5,math.radians(180)),
            self.goals_helper.goals_pose( 46.5, 22.5,math.radians(180)),
            self.goals_helper.goals_pose( 46.5, 22.5,math.radians(90)),
            self.goals_helper.goals_pose( 46.5, 38,math.radians(90)),
            self.goals_helper.goals_pose( 46.5, 38,math.radians(180)),
            self.goals_helper.goals_pose( 6, 38,math.radians(180)),
            self.goals_helper.goals_pose( 6, 38,math.radians(-90)),
            self.goals_helper.goals_pose( 6, 26,math.radians(-90)),
            self.goals_helper.goals_pose( 6, 26,math.radians(0)),
            self.goals_helper.goals_pose( 48, 26,math.radians(0)),
            self.goals_helper.goals_pose( 48, 26,math.radians(-90)),
            self.goals_helper.goals_pose( 48 , 22.5 ,math.radians(90)),
            self.goals_helper.goals_pose( 48 , 22.5 ,math.radians(180)),
            self.goals_helper.goals_pose( 6 , 22.5 ,math.radians(180)),
            self.goals_helper.goals_pose( 6 , 22.5 ,math.radians(180)),
            self.goals_helper.goals_pose( 6 , 22.5 ,math.radians(-90)),
            self.goals_helper.goals_pose( 6 , 19 ,math.radians(-90)),
            self.goals_helper.goals_pose( 6 , 19 ,math.radians(0)),
            self.goals_helper.goals_pose( 48 , 19 ,math.radians(0)),
            self.goals_helper.goals_pose( 48 , 19 ,math.radians(-90)),
            self.goals_helper.goals_pose( 48 , 19 ,math.radians(-90)),
            self.goals_helper.goals_pose( 48 , 8 ,math.radians(-90)),
            self.goals_helper.goals_pose( 48 , 8 ,math.radians(180)),
            self.goals_helper.goals_pose( 6 , 8 ,math.radians(180)),
#beacon
            self.goals_helper.goals_pose( 22.5 , 22.5 ,math.radians(180)),
            self.goals_helper.goals_pose( 6 , 22.5 ,math.radians(180)), # fix experimentally?
            self.goals_helper.goals_pose( 6 , 26.7 ,math.radians(180)) # fix experimentally?

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




