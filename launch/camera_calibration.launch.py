from launch import LaunchDescription
from launch_ros.actions import Node

camera_parameters = {'camera': 0, 'width': 1640, 'height': 1232, 'sensor_mode': '1640:1232', 'format': 'YUYV'}

def generate_launch_description():
    return LaunchDescription([
        # Camera Node
        Node(
            package="camera_ros",
            executable="camera_node",
            name="camera",
            parameters=[camera_parameters],
            remappings=[
                ('/camera/image_raw', '/camera/image_raw_fast'),
                ('/camera/image_raw/compressed', '/camera/image_raw_fast/compressed'),
            ]
        ),

        # Throttle Node (Topic Tools)
        Node(
            package="topic_tools",
            executable="throttle",
            name="throttle",
            arguments=["messages", "/camera/image_raw_fast", "1.0", "/camera/image_raw"]
        ),

        # Throttle Node (Topic Tools)
        Node(
            package="topic_tools",
            executable="throttle",
            name="throttle",
            arguments=["messages", "/camera/image_raw_fast/compressed", "1.0", "/camera/image_raw/compressed"]
        ),
    ])

