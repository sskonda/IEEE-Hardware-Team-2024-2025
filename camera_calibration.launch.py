import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Camera Node
        launch_ros.actions.Node(
            package="camera_ros",
            executable="camera_node",
            name="camera",
            parameters=[{'camera': 0, 'width': 1640, 'height': 1232, 'sensor_mode': '1640:1232', 'format': 'YUYV'}],
            remappings=[
                ('/camera/image_raw', '/camera/image_raw_fast'),
                ('/camera/image_raw/compressed', '/camera/image_raw_fast/compressed'),
            ]
        ),

        # Throttle Node (Topic Tools)
        launch_ros.actions.Node(
            package="topic_tools",
            executable="throttle",
            name="throttle",
            arguments=["messages", "/camera/image_raw_fast", "1.0", "/camera/image_raw"]
        ),

        # Throttle Node (Topic Tools)
        launch_ros.actions.Node(
            package="topic_tools",
            executable="throttle",
            name="throttle",
            arguments=["messages", "/camera/image_raw_fast/compressed", "1.0", "/camera/image_raw/compressed"]
        ),
    ])

