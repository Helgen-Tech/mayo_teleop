from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mayo_teleop',
            executable='drive_teleop',
            output='screen'),
        Node(
            package='joy',
            executable='joy_node',
            output='screen'),
    ])