import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64
from geometry_msgs.msg import Pose2D
#from goals import Goals
import math

class Goal:
    def __init__(self, node: Node):
        self.node = node

    def start(self):
        pass

    def repeated(self):
        pass
    
    def end(self):
        pass

    def is_finished(self) -> bool:
        return True

class DriveToPose(Goal):
    def __init__(self, node, x, y, theta, position_tolerance=0.05, angle_tolerance=0.01):
        super().__init__(node)
        self.x=x
        self.y=y
        self.theta= theta
        self.done = False
    def start(self):
        super().start()
        self.goal_done_sub = self.node.create_subscription(Bool, '/drive/goal_done', self.drive_done_callback, 1)
        self.goal_pub = self.node.create_publisher(Pose2D, '/drive/goal_pose', 1)
        self.node.get_logger().info(f" Published goal: x={self.x:.2f}, y={self.y:.2f}, θ={self.theta:.2f}")

    def repeated(self):
        pose = Pose2D(x=self.x, y=self.y, theta=self.theta)
        self.goal_pub.publish(pose)
        
    def is_finished(self):
        return self.done
    def end(self): 
        self.node.destroy_subscription(self.goal_done_sub)
        self.node.destroy_publisher(self.goal_pub)
    def drive_done_callback(self, msg):
        self.done= msg.data

class Clamp(Goal):
    def __init__(self, node, position):
        super().__init__(node)
        self.position= Float64(data=position)
        self.done=False
    def start(self):
        super().start() 
        self.goal_done_sub = self.node.create_subscription(Float64, '/clamp/goal_done', self.clamp_done_callback, 1)
        self.goal_pub = self.node.create_publisher(Float64, '/clamp', 10)
        self.goal_pub.publish(self.position)
        self.node.get_logger().info(f" Published goal: {self.position}")
    def is_finished(self):
        return self.done
    def end(self):
        self.node.destroy_subscription(self.goal_done_sub)
        self.node.destroy_publisher(self.goal_pub)    
    def clamp_done_callback(self, msg):
        self.done = msg.data

class Lift(Goal):
    def __init__(self, node, position):
        super().__init__(node)
        self.position= Float64(data=position)
    def start(self):
        super().start() 
        self.goal_pub = self.node.create_publisher(Float64, '/lift', 10)
        self.goal_pub.publish(self.position)
        self.node.get_logger().info(f" Published goal: {self.position}")
    def end(self):
        self.node.destroy_publisher(self.goal_pub)    

class Beacon(Goal): 
    def __init__(self, node, position):
        super().__init__(node)
        self.position= Float64(data=position)
    def start(self):
        super().start() 
        self.goal_pub = self.node.create_publisher(Float64, '/beacon', 10)
        self.goal_pub.publish(self.position)
        self.node.get_logger().info(f" Published goal: {self.position}")
    def end(self):
        self.node.destroy_publisher(self.goal_pub)    

class Intake(Goal): 
    def __init__(self, node, duty):
        super().__init__(node)
        self.duty= Float64(data=duty)
    def start(self):
        super().start() 
        self.goal_pub = self.node.create_publisher(Float64, '/intake', 10)
        self.goal_pub.publish(self.duty)
        self.node.get_logger().info(f" Published goal: {self.duty}")
    def end(self):
        self.node.destroy_publisher(self.goal_pub)    


class autonomous_node(Node):
    def __init__(self):
        super().__init__('autonomous_node')

        self.goals = [
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
            self.timer = self.create_timer(0.020, self.publish_next_goal)
            self.current_goal_index = 0
            self.goals[self.current_goal_index].start()

    def publish_next_goal(self):
        if self.active:
            action = self.goals[self.current_goal_index]

            if action.is_finished():
                action.end()
                self.current_goal_index += 1
                if self.current_goal_index < len(self.goals):
                    self.goals[self.current_goal_index].start()
                else:
                    self.finished()
            else:
                action.repeated()

            self.get_logger().info(f"Waiting for goal {self.current_goal_index} to be done")

    def finished(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = autonomous_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()




