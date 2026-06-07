import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('tracker'), 'config', 'tracker.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='tracker_node 튜닝 파라미터 yaml (config/tracker.yaml). '
                        '토픽/게이트/Kalman 노이즈 등 모든 튜닝값 포함.',
        ),
        Node(
            package='tracker',
            executable='tracker_node',
            name='tracker_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
