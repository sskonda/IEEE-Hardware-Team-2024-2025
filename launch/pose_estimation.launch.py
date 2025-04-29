from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from py_robot.py_robot.common import *

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
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0,
    ]
}


def generate_launch_description():
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
            executable='tag_pose_publisher',
            name='tag_pose_publisher'
        ),
        Node(
            package='py_robot',
            executable='differential_drive',
            name='drive'
        ),
        camera_container,
        *tag_poses,
        Node(
            package='py_robot',
            executable='sensor_fusion',
            name='sensor_fusion',
        )
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='map_ekf',
            parameters=[map_kf_parameters]
        ),
    ])
