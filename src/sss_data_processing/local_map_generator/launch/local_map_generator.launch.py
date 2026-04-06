from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('local_map_generator')

    local_map_generator_config = os.path.join(pkg_share, 'config', 'local_map_generator_params.yaml')

    log_arg = DeclareLaunchArgument(
        'log',
        default_value='false'
    )

    return LaunchDescription([
        log_arg,
        Node(
            package='local_map_generator',
            executable='swath_processing_node',
            name='swath_processing_node',
            output='screen',
            parameters=[
                local_map_generator_config,
                {'log': LaunchConfiguration('log')}
            ]
        ),
        Node(
            package='local_map_generator',
            executable='map_generation_node',
            name='map_generation_node',
            output='screen',
            parameters=[
                local_map_generator_config,
                {'log': LaunchConfiguration('log')}
            ]
        ),
    ])
