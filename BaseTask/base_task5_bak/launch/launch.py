from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='base_task5_bak',
            executable='main',
            name='main'
        ),
        Node(
            package='base_task5_bak',
            executable='reciver',
            name='reciver'
        ),
    ])