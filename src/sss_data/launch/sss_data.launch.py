from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('sss_data')

    benchmark_config = os.path.join(pkg_share, 'config', 'benchmark_params.yaml')

    return LaunchDescription([
        Node(
            package='sss_data',
            executable='sss_data_node.py',
            name='sss_data_node',
            output='screen',
            parameters=[
                benchmark_config
            ]
        )
    ])
