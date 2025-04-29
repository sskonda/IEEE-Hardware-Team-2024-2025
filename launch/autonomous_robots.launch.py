import math
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import json
import os


def generate_launch_description():
    if os.path.exists('color_thresholds.json'):
        with open('color_thresholds.json', 'r') as f:
            color_thresholds = json.load(f)
    else:
        color_thresholds = None
        
    # Camera & AprilTag settings
    camera_parameters = {
        'camera': 0,
        'width': 1640,
        'height': 1232,
        'sensor_mode': '1640:1232',
        'format': 'RGB888',
    }

    odom_kf_parameters = {
        'frequency': 50.0,
        'sensor_timeout': 0.02,
        'two_d_mode': True,
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_link_frame': 'base_link',
        'world_frame': 'odom',
        'imu0': '/sensors/imu',
        'imu0_config': [
            False, False, False,
            False, False, False,
            False, False, False,
            True, True, True,
            True, True, True,
        ],
        'imu0_remove_gravitational_acceleration': True,
        'odom0': '/drive/odometry',
        'odom0_config': [
            True, True, False,    # Linear Position
            False, False, False,    # Angular Position
            False, False, False,    # Linear Vel
            False, False, False,  # Angular Vel
            False, False, False,  # Linear Accel
        ],
        'initial_state': [0.0] * 15,
    }

    def to_args(d):
        return [str(val) for pair in (('--'+k, v) for k, v in d.items()) for val in pair]

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
        # EKF nodes for odometry and map
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='odometry_ekf',
        #     parameters=[odom_kf_parameters],
        #     arguments=['--log-level', 'debug']
        # ),
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='map_ekf',
        #     parameters=[map_kf_parameters]
        # ),
        Node(
            package='py_robot',
            executable='object_detection',
            name='object_detection',
            parameters=[color_thresholds] if color_thresholds is not None else [],
        ),
        Node(
            package='py_robot',
            executable='path_planner',
            remappings=[
                ('/points_to_visit', '/purple_dots')
            ],
            name='path_planner'
        ),
        # Node(
        #     package='web_video_server',
        #     executable='web_video_server',
        #     name='web_video_server',
        # )
    ])
