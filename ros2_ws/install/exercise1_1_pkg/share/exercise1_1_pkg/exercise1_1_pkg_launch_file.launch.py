from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='exercise1_1_pkg',
            executable='exercise1_1',
            output='screen'),
    ])
    