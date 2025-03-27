import math
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Pose2D

class Goals:
    def goal_pose(self, x, y, theta):
        def send():
            pose = Pose2D(x=x, y=y, theta=theta)
            self.goal_pub.publish(pose)
            self.get_logger().info(f" Published goal: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")
        return send
    def lift(self, up: bool):
        def send():
            self.lift.publish(Bool(data=up))
            self.get_logger().info(f" Lift {'up' if up else 'down'}")
        return send
    def clamp(self, position: float):
        def send():
            self.clamp.publish(Float32(data=position))
            self.get_logger().info(f" Clamp set to {position:.2f}")
        return send
    def beacon(self, position: float):
        def send():
            self.beacon.publish(Float32(data=position))
            self.get_logger().info(f" Beacon set to {position:.2f}")
        return send
    def intake(self, speed: float):
        def send():
            self.intake.publish(Float32(data=speed))
            self.get_logger().info(f" Intake speed set to {speed:.2f}")
        return send
