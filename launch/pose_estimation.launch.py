from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import xacro

camera_parameters = {
    'camera': 0,
    'width': 1640,
    'height': 1232,
    'sensor_mode': '1640:1232',
    'format': 'YUYV'
}

apriltag_parameters = {
    'image_transport': 'raw',
    'family': '36h11',
    'size': 0.080,
    'max_hamming': 0,
    'detector': {
        'threads': 2,
        'decimate': 2.0,
        'blur': 0.0,
        'refine': True,
        'sharpening': 0.25,
        'debug': False
    }
}

kf_parameters = {
    
}

def generate_launch_description():
    apriltag_container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                name='camera',
                parameters=[camera_parameters],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::DebayerNode',
                name='debayer_node',
                namespace='camera',
                remappings=[
                    ('image_mono', 'image'),
                ],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify',
                namespace='camera',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag',
                namespace='camera',
                parameters=[apriltag_parameters],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )
    
    robot_description = xacro.process_file('robot.xacro').toprettyxml(indent='  ') # type: ignore
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state',
            parameters=[{'robot_description': robot_description}]
        ),
        apriltag_container,
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='camera_server',
        )
    ])
