from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy_parameters_file = os.path.join(
        get_package_share_directory('break_controller'),
        'config', 'xbox_teleop.yaml'
    )

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[joy_parameters_file],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_parameters_file],
        ),
        Node(
            package='break_controller',
            executable='break_controller.py',    
            name='break_controller',
            output='screen',
        ),
    ])
