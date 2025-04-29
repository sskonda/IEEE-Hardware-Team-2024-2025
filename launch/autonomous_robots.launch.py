from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from py_robot.common import *

def generate_launch_description():
    camera_container = ComposableNodeContainer(
        name='purple_container',
        namespace='camera',  # <--- Put everything in 'camera' namespace
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
                plugin='image_proc::RectifyNode',
                name='rectify',
                namespace='camera',
                remappings=[
                    ('image',       'image_raw'),       # input from debayer output
                    ('camera_info', 'camera_info'),
                    ('image_rect',  'image_rect'),  # output topic
                ],
                extra_arguments=[{'use_intra_process_comms': True}] 
            ),
            # ComposableNode(
            #     package='apriltag_ros',
            #     plugin='AprilTagNode',
            #     name='apriltag',
            #     namespace='camera',
            #     parameters=[apriltag_parameters],
            #     extra_arguments=[{'use_intra_process_comms': True}],
            # )
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
            executable='drive_to_path',
            name='drive_controller',
            parameters=[
                {'position_tolerance': 0.0254},
                {'angle_tolerance': 0.25},
                {'linear_gain': 0.5},
                {'angular_gain': 1.5},
            ],
            remappings=[
                # ('/odometry/filtered', '/filtered_odom')
                ('/goal_pose', '/drive/goal_pose'),
                ('/goal_done', '/drive/goal_done'),
                ('/filtered_odom', '/odometry/filtered'),
                ('/enable', '/drive/enable')
            ]
        ),

        Node(
            package='robot_control',
            executable='sensor_fusion',
            name='sensor_fusion',
        ),
        Node(
            package='py_robot',
            executable='object_detection',
            name='object_detection',
            parameters=[color_thresholds],
        ),
        Node(
            package='py_robot',
            executable='path_planner',
            remappings=[
                ('/points_to_visit', '/purple_dots')
            ],
            name='path_planner'
        ),
    ])
