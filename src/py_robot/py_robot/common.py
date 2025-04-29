from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import math
import xacro
import json
import os

if os.path.exists('color_thresholds.json'):
    with open('color_thresholds.json', 'r') as f:
        color_thresholds = json.load(f)
else:
    color_thresholds = {}
    
# Camera & AprilTag settings
camera_parameters = {
    'camera': 0,
    'width': 1640,
    'height': 1232,
    'sensor_mode': '1640:1232',
    'format': 'RGB888',
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

camera_container = ComposableNodeContainer(
    name='camera_container',
    namespace='camera',
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