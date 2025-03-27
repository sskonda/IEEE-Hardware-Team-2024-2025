import math
from launch import LaunchDescription
from launch_ros.actions import Node

def to_args(d):
    return [
        str(d['x']), str(d['y']), str(d['z']),
        str(d['roll']), str(d['pitch']), str(d['yaw']),
        d['frame-id'], d['child-frame-id']
    ]


def generate_launch_description():
    tag_poses = [
        Node(
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
        ) for n in range(5)
    ] + [
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
        )
    ]

    return LaunchDescription([
        Node(
            package='robot_control',
            executable='drive_to_pose',
            name='drive_controller',
            parameters=[
                {"position_tolerance": 0.1},
                {"angle_tolerance": 0.01}
            ],
            remappings=[
                ('/cmd_vel', '/turtle1/cmd_vel')  # Replace this with actual robot cmd_vel if needed
            ]
        ),
        Node(
            package='py_robot',
            executable='mpu6500',
            name='imu'
        ),
        Node(
            package='py_robot',
            executable='tag_pose_publisher',
            name='tag_pose_publisher'
        ),
        Node(
            package='py_robot',
            executable='differential_drive',
            name='drive'
        ),
        *tag_poses
    ])
