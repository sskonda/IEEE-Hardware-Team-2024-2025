import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pigpio

class HardwareStopNode(Node):
    def __init__(self):
        super().__init__('hardware_stop_node')

        self.get_logger().info(" Waiting for emergency stop" )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist_stop = Twist()

        self.pi = pigpio.pi()
        self.gpio_pin = 7 # CHANGE AT DISCRETION DONT RUN BEFORE CHANGE GPIO PIN
        self.pi.set_mode(self.gpio_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.gpio_pin, pigpio.PUD_UP) # ACTIVE LOW PULL UP IN SOFTWARE 

        self.kill_triggered = False # FLAG FOR THE KILL SWITCH 

        # Check GPIO periodically, maybe should be done more efficiently 
        self.timer = self.create_timer(0.05, self.monitor_button)

    def monitor_button(self):
        if not self.kill_triggered and self.pi.read(self.gpio_pin) == 0:
            self.get_logger().error(" HARDWARE KILL SWITCH ACTIVATED")
            self.kill_triggered = True

        if self.kill_triggered:
            self.cmd_vel_pub.publish(self.twist_stop)

    def destroy_node(self):
        self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
