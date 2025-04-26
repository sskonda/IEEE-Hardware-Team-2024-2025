import rclpy  # use rospy if ROS 1
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import math

class NearestNeighborPlanner(Node):
    def __init__(self):
        super().__init__('nearestneighbor_planner') 
        self.subscription = self.create_subscription( #subscribes to topic
            PoseArray,
            'points_to_visit',
            self.listener_callback,
            10
        ) 
        self.publisher = self.create_publisher(PoseArray, 'ordered_path', 10) #creates a topic to publish to

    def listener_callback(self, msg): #called when receives points
        if len(msg.poses) < 2: #cannnot perform path planning with less than 2 points
            self.get_logger().warn('Need at least 2 points for path planning.')
            return

        current = msg.poses[0] #starting point
        goals = msg.poses[1:] #rest of points given

        ordered = self.greedy_path(current, goals) #calls path planning function

        out_msg = PoseArray() #new PoseArray message
        out_msg.header = msg.header #copies header in received message
        out_msg.header.frame_id = 'map' 
        out_msg.poses = ordered 
        self.publisher.publish(out_msg) #publishes ordered points

    def distance(self, p1, p2): #calculates distance between two points
        dx = p1.position.x - p2.position.x
        dy = p1.position.y - p2.position.y
        return math.hypot(dx, dy)

    def greedy_path(self, start, points): #orders points
        unvisited = points.copy()
        current = start
        path = []

        while unvisited:
            next_point = min(unvisited, key=lambda p: self.distance(current, p))
            path.append(next_point)
            unvisited.remove(next_point)
            current = next_point

        return path

def main(args=None):
    rclpy.init(args=args)
    node = NearestNeighborPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()