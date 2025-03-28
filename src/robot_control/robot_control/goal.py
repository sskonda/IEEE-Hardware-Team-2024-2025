from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Pose2D

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

