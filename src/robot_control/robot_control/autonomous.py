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
        super().__init__(self, node)
        self.x=x
        self.y=y
        self.theta= theta
        self.done = False
    def start(self):
        super().start()
        pose = Pose2D(x=self.x, y=self.y, theta=self.theta)
        self.goal_done_sub = self.node.create_subscription(Bool, '/goal_done', self.drive_done_callback, 1)
        self.goal_pub = self.node.create_publisher(Pose2D, '/drive/goal_position', 1)
        self.goal_pub.publish(pose)
        self.get_logger().info(f" Published goal: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")
    def is_finished(self):
        return self.done
    def end(self): 
        self.node.destroy_subscription(self.goal_done_sub)
        self.node.destroy_publisher(self.goal_pub)
    def drive_done_callback(self, msg):
        self.done= msg.data

class Clamp(Goal):
    def __init__(self, node, position)
        super().__init__(self.node)
        self.position= position
        self.done=done
    def start(self):
        super().start() 
        self.goal_done_sub = self.node.create_subscription(Bool, '/goal_done', self.clamp_done_callback, 1)
        self.goal_pub = self.node.create_publisher(Float64, '/clamp', 10)
        self.goal_pub.publish(self.position)
        self.get_logger().info(f" Published goal: {position}")
    def is_finished(self):
        return self.done
    def end(self):
        self.node.destroy_subscription(self.goal_done_sub)
        self.node.destroy_publisher(self.goal_pub)    
    def clamp_done_callback(self):
        self.done = msg.data

class Lift(Goal):
    def __init__(self, node, position)
        super().__init__(self.node)
        self.position= position
    def start(self):
        super().start() 
        self.goal_pub = self.node.create_publisher(Bool, '/lift', 10)
        self.goal_pub.publish(self.position)
        self.get_logger().info(f" Published goal: {position}")
    def end(self):
        self.node.destroy_publisher(self.goal_pub)    


class Beacon(Goal): 
    def __init__(self, node, position)
        super().__init__(self.node)
        self.position= position
    def start(self):
        super().start() 
        self.goal_pub = self.node.create_publisher(Float64, '/beacon', 10)
        self.goal_pub.publish(self.position)
        self.get_logger().info(f" Published goal: {position}")
    def is_finished(self):
        return self.done
    def end(self):
        self.node.destroy_publisher(self.goal_pub)    

class Intake(Goal): 
    def __init__(self, node, duty)
        super().__init__(self.node)
        self.duty= duty
        self.done=done
    def start(self):
        super().start() 
        self.goal_pub = self.node.create_publisher(Float64, '/intake', 10)
        self.goal_pub.publish(self.duty)
        self.get_logger().info(f" Published goal: {duty}")
    def is_finished(self):
        return self.done
    def end(self):
        self.node.destroy_subscription(self.goal_done_sub)
        self.node.destroy_publisher(self.goal_pub)    
    def intake_done_callback(self):
        self.done = msg.data

    

    def goal_pose(self, x, y, theta):
        def send():
            pose = Pose2D(x=x, y=y, theta=theta)
            self.goal_pub.publish(pose)
            self.get_logger().info(f" Published goal: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")
        return send
    def lift(self, up: bool):
        def send():
            self.lift_pub.publish(Bool(data=up))
            self.get_logger().info(f" Lift {'up' if up else 'down'}")
        return send
    def clamp(self, position: float):
        def send():
            self.clamp_pub.publish(Float64(data=position))
            self.get_logger().info(f" Clamp set to {position:.2f}")
        return send
    def beacon(self, position: float):
        def send():
            self.beacon_pub.publish(Float64(data=position))
            self.get_logger().info(f" Beacon set to {position:.2f}")
        return send
    def intake(self, speed: float):
        def send():
            self.intake_pub.publish(Float64(data=speed))
            self.get_logger().info(f" Intake speed set to {speed:.2f}")
        return send

class autonomous_node(Node):
    def __init__(self):
        super().__init__('autonomous_node')
        self.goal_done_sub = self.create_subscription(Bool, '/goal_done', self.goal_done_callback, 10)
        self.goal_pub = self.create_publisher(Pose2D, '/goal_pose', 10)
        self.lift_pub = self.create_publisher(Bool, '/lift', 10)
        self.clamp_pub = self.create_publisher(Float64, '/clamp', 10)
        self.beacon_pub = self.create_publisher(Float64, '/beacon', 10)
        self.intake_pub = self.create_publisher(Float64, '/intake', 10)
        #self.start_location= (31.25, 4.625, 0.0)

        self.goals = [
            Goals.goal_pose(self, 0.79375, 0.254, math.radians(0.0)),
            Goals.goal_pose(self, 0.79375, 0.254, math.radians(180)),
            Goals.goal_pose(self, 1.17375, 0.2032, math.radians(180)),
            Goals.clamp(self, 0.0),  # release clamp
            Goals.goal_pose(self,0.5747625, 0.581552, 0.0),
            Goals.goal_pose(self,0.5747625, 0.581552, math.radians(-90)),
            Goals.goal_pose(self,0.5747625, 0.581552, math.radians(90)),
            Goals.goal_pose(self,0.5747625, 0.873125, math.radians(90)),
            Goals.clamp(self,0.0),
            Goals.clamp(self,.8),
            Goals.goal_pose(self,0.5747625, 0.873125, math.radians(0)),
            Goals.goal_pose(self,1.1811, 0.5715 ,math.radians(0) ),
            Goals.goal_pose(self,1.7907, 0.5715, math.radians(0)),
            Goals.goal_pose(self,1.7907, 0.5715, math.radians(90)),
            Goals.goal_pose(self,1.7907, 0.9652, math.radians(90)),
            Goals.goal_pose(self,1.7907, 0.9652, math.radians(90)),
            Goals.goal_pose(self,1.7907, 0.9652, math.radians(0)),
            Goals.goal_pose(self,2.159, 0.9652, math.radians(0)),
            Goals.goal_pose(self,2.159, 0.9652, math.radians(0)),
            Goals.goal_pose(self,2.159, 0.9652, math.radians(-90)),
            Goals.goal_pose(self,2.159, 0.1524, math.radians(-90)),
            Goals.goal_pose(self, 2.159, 0.1524, math.radians(-90)),
            Goals.goal_pose(self, 2.159, 0.1524, math.radians(90)),
            Goals.goal_pose(self, 2.159, 0.5715, math.radians(180)),
            Goals.goal_pose(self,1.1811, 0.5715, math.radians(180)),
            Goals.goal_pose(self, 1.1811, 0.5715, math.radians(90)),
            Goals.goal_pose(self,1.1811, 0.9652, math.radians(90)),
            Goals.goal_pose(self,1.1811, 0.9652, math.radians(180)),
            Goals.goal_pose(self,0.1524, 0.9652, math.radians(180)),
            Goals.goal_pose(self,0.1524, 0.9652, math.radians(-90)),
            Goals.goal_pose(self,0.1524, 0.6604, math.radians(-90)),
            Goals.goal_pose(self,0.1524, 0.6604, math.radians(0)),
            Goals.goal_pose(self,1.2192, 0.6604, math.radians(0)),
            Goals.goal_pose(self,1.2192, 0.6604, math.radians(-90)),
            Goals.goal_pose(self,1.2192 , 0.5715 ,math.radians(90)),
            Goals.goal_pose(self,1.2192 , 0.5715 ,math.radians(180)),
            Goals.goal_pose(self,0.1524 , 0.5715 ,math.radians(180)),
            Goals.goal_pose(self,0.1524 , 0.5715 ,math.radians(180)),
            Goals.goal_pose(self,0.1524 , 0.5715 ,math.radians(-90)),
            Goals.goal_pose(self,0.1524 , 0.4826 ,math.radians(-90)),
            Goals.goal_pose(self,0.1524 , 0.4826 ,math.radians(0)),
            Goals.goal_pose(self,1.2192 , 0.4826 ,math.radians(0)),
            Goals.goal_pose(self, 1.2192 , 0.4826 ,math.radians(-90)),
            Goals.goal_pose(self, 1.2192 , 0.4826 ,math.radians(-90)),
            Goals.goal_pose(self,1.2192 , 0.2032 ,math.radians(-90)),
            Goals.goal_pose(self,1.2192 , 0.2032 ,math.radians(180)),
            Goals.goal_pose(self,0.1524 , 0.2032 ,math.radians(180)),
        #beacon
            Goals.goal_pose(self, 0.5715 , 0.5715 ,math.radians(180)),
            Goals.goal_pose(self, 0.1524 , 0.5715 ,math.radians(180)),
            Goals.goal_pose(self, 0.1524 , 0.67818 ,math.radians(180))
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
            #self.goal_reached = False
            self.waiting_for_completion = False
            self.publish_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = autonomous_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()




