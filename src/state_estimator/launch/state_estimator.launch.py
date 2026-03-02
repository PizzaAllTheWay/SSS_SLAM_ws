from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('state_estimator')

    imu_config   = os.path.join(pkg_share, 'config', 'imu_params.yaml')
    depth_config = os.path.join(pkg_share, 'config', 'depth_params.yaml')
    dvl_config   = os.path.join(pkg_share, 'config', 'dvl_params.yaml')
    ukfm_config  = os.path.join(pkg_share, 'config', 'ukfm_params.yaml')

    log_arg = DeclareLaunchArgument(
        'log',
        default_value='false'
    )

    return LaunchDescription([
        log_arg,
        Node(
            package='state_estimator',
            executable='state_estimator_node.py',
            name='state_estimator_node',
            output='screen',
            parameters=[
                imu_config,
                depth_config,
                dvl_config,
                ukfm_config,
                {'log': LaunchConfiguration('log')}
            ]
        )
    ])