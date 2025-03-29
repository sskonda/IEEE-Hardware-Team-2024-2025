import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import pigpio

class HardwareStopNode(Node):
    def __init__(self):
        super().__init__('hardware_stop_node')

        self.get_logger().info("Waiting for hardware stop input...")

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, '/enable', 10)
        self.go_pub = self.create_publisher(Bool, '/go', 10)

        self.twist_stop = Twist()
        self.robot_initialized = False
        self.kill_triggered = False

        self.pi = pigpio.pi()
        self.gpio_pin = 7  # !!! CHANGE THIS TO YOUR ACTUAL PIN
        self.pi.set_mode(self.gpio_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.gpio_pin, pigpio.PUD_UP)  # Active low

        self.last_state = 1  # Track last button state
        self.timer = self.create_timer(0.05, self.monitor_button)

    def monitor_button(self):
        current_state = self.pi.read(self.gpio_pin)

        if self.last_state == 1 and current_state == 0:  # falling edge
            if not self.robot_initialized:
                self.get_logger().info("Button pressed: initializing robot.")
                self.enable_pub.publish(Bool(data=True))
                self.go_pub.publish(Bool(data=True))
                self.robot_initialized = True
            else:
                self.get_logger().error("Hardware kill switch triggered.")
                self.kill_triggered = True

        if self.kill_triggered:
            self.cmd_vel_pub.publish(self.twist_stop)

        self.last_state = current_state

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
