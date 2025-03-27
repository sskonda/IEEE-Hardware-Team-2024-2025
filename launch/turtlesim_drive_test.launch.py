from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='py_robot',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        Node(
            package='py_robot',  
            executable='turtle_pose_to_odom',
            name='alias'
        ),

        Node(
            package='robot_control', 
            executable='drive_to_pose',
            name='drive_controller',
            parameters=[
                {"position_tolerance": 0.1},
                {"angle_tolerance": 0.01}],
            remappings=[
                ('/cmd_vel', '/turtle1/cmd_vel')  
            ]
        )
    ])
