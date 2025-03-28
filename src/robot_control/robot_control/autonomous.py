import rclpy
from rclpy.node import Node
#from goals import Goals
import math
from .goal import *

DRIVE_FORWARD = lambda node: [
    DriveToPose(node, 0.5, 0.0, 0.0),
    DriveToPose(node, 0.0, 0.0, 0.0),
    DriveToPose(node, 0.5, 0.0, 0.0),
]

class autonomous_node(Node):
    def __init__(self):
        super().__init__('autonomous_node')

        self.goals = DRIVE_FORWARD(self)
        
        _ = [
            DriveToPose(self, 0.79375, 0.254, math.radians(0.0)),
            DriveToPose(self, 0.79375, 0.254, math.radians(180)),
            DriveToPose(self, 1.17375, 0.2032, math.radians(180)),
            Clamp(self, 0.0),  # release clamp
            DriveToPose(self,0.5747625, 0.581552, 0.0),
            DriveToPose(self,0.5747625, 0.581552, math.radians(-90)),
            DriveToPose(self,0.5747625, 0.581552, math.radians(90)),
            DriveToPose(self,0.5747625, 0.873125, math.radians(90)),
            Clamp(self,0.0),
            Clamp(self,0.8),
            DriveToPose(self,0.5747625, 0.873125, math.radians(0)),
            Intake(self, 0.8),
            DriveToPose(self,1.1811, 0.5715 ,math.radians(0) ),
            DriveToPose(self,1.7907, 0.5715, math.radians(0)),
            DriveToPose(self,1.7907, 0.5715, math.radians(90)),
            DriveToPose(self,1.7907, 0.9652, math.radians(90)),
            DriveToPose(self,1.7907, 0.9652, math.radians(90)),
            DriveToPose(self,1.7907, 0.9652, math.radians(0)),
            DriveToPose(self,2.159, 0.9652, math.radians(0)),
            DriveToPose(self,2.159, 0.9652, math.radians(0)),
            DriveToPose(self,2.159, 0.9652, math.radians(-90)),
            DriveToPose(self,2.159, 0.1524, math.radians(-90)),
            DriveToPose(self, 2.159, 0.1524, math.radians(-90)),
            DriveToPose(self, 2.159, 0.1524, math.radians(90)),
            DriveToPose(self, 2.159, 0.5715, math.radians(180)),
            DriveToPose(self,1.1811, 0.5715, math.radians(180)),
            DriveToPose(self, 1.1811, 0.5715, math.radians(90)),
            DriveToPose(self,1.1811, 0.9652, math.radians(90)),
            DriveToPose(self,1.1811, 0.9652, math.radians(180)),
            DriveToPose(self,0.1524, 0.9652, math.radians(180)),
            DriveToPose(self,0.1524, 0.9652, math.radians(-90)),
            DriveToPose(self,0.1524, 0.6604, math.radians(-90)),
            DriveToPose(self,0.1524, 0.6604, math.radians(0)),
            DriveToPose(self,1.2192, 0.6604, math.radians(0)),
            DriveToPose(self,1.2192, 0.6604, math.radians(-90)),
            DriveToPose(self,1.2192 , 0.5715 ,math.radians(90)),
            DriveToPose(self,1.2192 , 0.5715 ,math.radians(180)),
            DriveToPose(self,0.1524 , 0.5715 ,math.radians(180)),
            DriveToPose(self,0.1524 , 0.5715 ,math.radians(180)),
            DriveToPose(self,0.1524 , 0.5715 ,math.radians(-90)),
            DriveToPose(self,0.1524 , 0.4826 ,math.radians(-90)),
            DriveToPose(self,0.1524 , 0.4826 ,math.radians(0)),
            DriveToPose(self,1.2192 , 0.4826 ,math.radians(0)),
            DriveToPose(self, 1.2192 , 0.4826 ,math.radians(-90)),
            DriveToPose(self, 1.2192 , 0.4826 ,math.radians(-90)),
            DriveToPose(self,1.2192 , 0.2032 ,math.radians(-90)),
            DriveToPose(self,1.2192 , 0.2032 ,math.radians(180)),
            DriveToPose(self,0.1524 , 0.2032 ,math.radians(180)),
        #beacon
            Intake(self, 0.0),
            DriveToPose(self, 0.5715 , 0.5715 ,math.radians(180)),
            DriveToPose(self, 0.1524 , 0.5715 ,math.radians(180)),
            DriveToPose(self, 0.1524 , 0.67818 ,math.radians(180)),
            Beacon(self, 1),
            Lift(self, 1)
        ]

        self.current_goal_index = 0
        self.go_sub = self.create_subscription(Bool, '/go', self.go_callback, 10)
        self. get_logger().info("Wating for the GO")

        self.timer = None
        self.goal_sent = False
        self.active = False

    def go_callback(self, msg):
        if msg.data and not self.active:
            self.active = True
            self.get_logger().info(" GO signal received. Starting goal sequence") # ONCE THE GO LIGHT TURNS ON
            self.timer = self.create_timer(0.020, self.publish_next_goal)
            self.current_goal_index = 0
            self.goals[self.current_goal_index].start()

    def publish_next_goal(self):
        if self.active:
            action = self.goals[self.current_goal_index]

            if action.is_finished():
                self.get_logger().info(f"Finished Goal {self.current_goal_index}")
                action.end()
                self.current_goal_index += 1
                if self.current_goal_index < len(self.goals):
                    self.get_logger().info(f"Starting Goal {self.current_goal_index}")
                    self.goals[self.current_goal_index].start()
                else:
                    self.finished()
            else:
                action.repeated()


    def finished(self):
        self.active = False

def main(args=None):
    rclpy.init(args=args)
    node = autonomous_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()




