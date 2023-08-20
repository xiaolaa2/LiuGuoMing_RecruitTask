from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    video_bag_path = r'src/advance_task/asset/vision_task'
    # 启动ros2bag读取视频信息
    read_bag = ExecuteProcess(
        cmd=[['ros2 ', 'bag ', 'play ', video_bag_path, ' --loop']],
        # output='screen',
        shell=True
    )

    # 启动节点
    my_node = Node(
        package='advance_task',
        namespace='advance_task1',
        executable='main',
        name='armor_detect',
    )
    return LaunchDescription([
        read_bag,
        my_node
    ])