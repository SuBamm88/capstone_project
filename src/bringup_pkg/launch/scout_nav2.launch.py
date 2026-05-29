import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('bringup_pkg')
    nav2_share = get_package_share_directory('nav2_bringup')

    nav2_launch = os.path.join(nav2_share, 'launch', 'bringup_launch.py')
    default_nav2_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    default_map = os.path.join(package_share, 'maps', 'scout_mini_map_3.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulated time if true',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the saved map yaml file',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params,
            description='Project Nav2 parameter file',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'slam': 'False',
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
            }.items(),
        ),
    ])
