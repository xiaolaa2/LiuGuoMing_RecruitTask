from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='base_task4',
            executable='main',
            name='sim'
        ),
        Node(
            package='base_task4',
            executable='reciver',
            name='sim'
        ),
    ])