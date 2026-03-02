from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sss_data',
            executable='sss_data_node.py',
            name='sss_data_node',
            output='screen'
        )
    ])
