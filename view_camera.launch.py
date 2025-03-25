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
        ),

        launch_ros.actions.Node(
            package="web_video_server",
            executable="web_video_server",
            name="video_server",
        ),
    ])

