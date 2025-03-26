from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import xacro
import math

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

odom_kf_parameters = {
    'frequency': 50.0,
    'sensor_timeout': 0.02,
    'two_d_mode': True,
    'map_frame': 'map',
    'odom_frame': 'odom',
    'base_link_frame': 'base_link',
#    'base_link_output_frame': 'base_link',
    'world_frame': 'odom',
    'imu0': '/sensors/imu',
    'imu0_config': [
        False, False, False,
        False, False, False,
        False, False, False,
        True, True, True,
        True, True, True,
    ],
    'imu0_remove_gravitational_acceleration': False,  # Removed manually in calibration
    'odom0': '/drive/odometry',
    'odom0_config': [
        True, True, True,
        True, True, True,
        True, True, True,
        False, False, False,
        False, False, False,
    ],
    'initial_state': [
        0.0, 0.0, 0.0, 
        0.0, 0.0, 1.57,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0,
    ]
}

map_kf_parameters = {
    **odom_kf_parameters,
    'world_frame': 'map'
}
for n in range(8):  # 8 possible tags
    map_kf_parameters.update({
        f'pose{n}': f'/sensors/tag_{n}',  # fuse robot pose from tag n
        f'pose{n}_config': [
            True, True, True,
            True, True, True,
            False, False, False,
            False, False, False,
            False, False, False,
        ],
    })


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
    
    robot_description = xacro.process_file('robot.xacro').toprettyxml(indent='  ') # type: ignore
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='py_robot',
            executable='mpu6500',
            name='imu',
        ),
        Node(
            package='py_robot',
            executable='differential_drive',
            name='drive'
        ),
        apriltag_container,
        *tag_poses,
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='odometry_ekf',
            parameters=[odom_kf_parameters]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='map_ekf',
            parameters=[map_kf_parameters]
        ),
    ])
