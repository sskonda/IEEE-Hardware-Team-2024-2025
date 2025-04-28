import math
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Camera & AprilTag settings
    camera_parameters = {
        'camera': 0,
        'width': 1640,
        'height': 1232,
        'sensor_mode': '1640:1232',
        'format': 'RGB888',
    }

    robot_description = xacro.process_file(
        'robot.xacro').toprettyxml(indent='  ')  # type: ignore

    camera_container = ComposableNodeContainer(
        name='purple_container',
        namespace='camera',  # <--- Put everything in 'camera' namespace
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            
            # 1) Camera driver node
            ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                name='camera',
                parameters=[camera_parameters],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),

            # 3) Rectify node
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify',
                namespace='camera',
                remappings=[
                    ('image',       'image_raw'),       # input from debayer output
                    ('camera_info', 'camera_info'),
                    ('image_rect',  'image_rect'),  # output topic
                ],
                extra_arguments=[{'use_intra_process_comms': False}]
            ),
        ],
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state',
            parameters=[{'robot_description': robot_description}]
        ),

        # IMU
        Node(
            package='py_robot',
            executable='mpu6500',
            name='imu',
            parameters=[
                {'axis_order': 'xyz'}
            ]
        ),
        # AprilTag detection container
        camera_container,
        Node(
            package='py_robot',
            executable='object_detection',
            name='object_detection',
        ),
        # Additional full-stack nodes
        Node(
            package='py_robot',
            executable='hardware_interface',
            name='hardware_interface'
        ),

        # Differential drive logic
        Node(
            package='py_robot',
            executable='differential_drive',
            name='drive'
        ),
        Node(
            package='robot_control',
            executable='sensor_fusion',
            name='sensor_fusion',
        ),
    ])
