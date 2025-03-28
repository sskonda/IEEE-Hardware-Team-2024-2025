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
        },
        'tag': {
            'ids': list(range(8)),
            'frames': [f'detected_tag:{n}' for n in range(8)]
        }
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
        'imu0_remove_gravitational_acceleration': False,
        'odom0': '/drive/odometry',
        'odom0_config': [
            True, True, True,
            True, True, True,
            True, True, True,
            False, False, False,
            False, False, False,
        ],
        'initial_state': [0.0] * 15,
    }

    map_kf_parameters = dict(odom_kf_parameters)
    map_kf_parameters['world_frame'] = 'map'
    for n in range(8):
        map_kf_parameters.update({
            f'pose{n}': f'/sensors/tag_{n}',
            f'pose{n}_config': [
                True, True, True,
                True, True, True,
                False, False, False,
                False, False, False,
                False, False, False,
            ],
        })

    def to_args(d):
        return [str(val) for pair in (('--'+k, v) for k, v in d.items()) for val in pair]

    tag_poses = (
        *(Node(
            name=f'tag_{n}',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=to_args({
                'frame-id': 'map',
                'child-frame-id': f'tag:{n}',
                'x': 0.0091,
                'y': 0.5715,
                'z': 0.05,
                'roll': math.pi / 2,
                'pitch': 0.0,
                'yaw': math.pi / 2,
            })
        ) for n in range(5)),
        Node(
            name='tag_5',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=to_args({
                'frame-id': 'map',
                'child-frame-id': 'tag:5',
                'x': 0.812,
                'y': 1.1405,
                'z': 0.05,
                'roll': math.pi / 2,
                'pitch': 0.0,
                'yaw': 0.0,
            })
        ),
        Node(
            name='tag_6',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=to_args({
                'frame-id': 'map',
                'child-frame-id': 'tag:6',
                'x': 1.1168,
                'y': 0.0025,
                'z': 0.05,
                'roll': math.pi / 2,
                'pitch': 0.0,
                'yaw': math.pi,
            })
        ),
        Node(
            name='tag_7',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=to_args({
                'frame-id': 'map',
                'child-frame-id': 'tag:7',
                'x': 2.3597,
                'y': 0.5715,
                'z': 0.05,
                'roll': math.pi / 2,
                'pitch': 0.0,
                'yaw': 3 * math.pi / 2,
            })
        ),
    )

    robot_description = xacro.process_file('robot.xacro').toprettyxml(indent='  ')  # type: ignore

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
                remappings=[('image_mono', 'image')],
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

    return LaunchDescription([
        # Static TF frames for all tags
        *tag_poses,

        # Robot URDF
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
        ),

        # Tag pose processing
        Node(
            package='py_robot',
            executable='tag_pose_publisher',
            name='tag_pose_publisher'
        ),

        # Differential drive logic
        Node(
            package='py_robot',
            executable='differential_drive',
            name='drive'
        ),

        # EKF nodes for odometry and map
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='odometry_ekf',
        #     parameters=[odom_kf_parameters]
        # ),
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='map_ekf',
        #     parameters=[map_kf_parameters]
        # ),

        # AprilTag detection container
        apriltag_container,

        # Additional full-stack nodes
        Node(
            package='py_robot',
            executable='hardware_interface',
            name='hardware_interface'
        ),
        # Node(
        #     package='py_robot',
        #     executable='button',
        #     name='emergency_stop'
        # ),
        # Node(
        #     package='py_robot',
        #     executable='start',
        #     name='start_trigger'
        # ),
        Node(
            package='robot_control',
            executable='drive_to_pose',
            name='drive_controller',
            parameters=[
                {'position_tolerance': 0.1},
                {'angle_tolerance': 0.01}
            ],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                #('/odometry/filtered', '/filtered_odom')
                ('/drive/odometry','/filtered_odom')
            ]
        )
    ])
