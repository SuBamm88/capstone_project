import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_share  = get_package_share_directory('bringup_pkg')
    nav2_share     = get_package_share_directory('nav2_bringup')
    planning_share = get_package_share_directory('planning_pkg')

    nav2_launch        = os.path.join(nav2_share,     'launch', 'bringup_launch.py')
    default_params     = os.path.join(bringup_share,  'config', 'nav2_params_cctv.yaml')
    default_map        = os.path.join(bringup_share,  'maps',   'map.yaml')
    bt_xml             = os.path.join(planning_share,  'behavior_trees', 'navigate_cctv_risk.xml')
    risk_params        = os.path.join(planning_share,  'config', 'risk_params.yaml')

    params_file = LaunchConfiguration('params_file')

    # bt_navigator의 default_nav_to_pose_bt_xml을 런타임 경로로 치환
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'default_nav_to_pose_bt_xml': bt_xml},
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulated time if true',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml file',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 parameter file (Proposed mode)',
        ),

        # Nav2 스택
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'slam':         'False',
                'map':          LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file':  configured_params,
            }.items(),
        ),

        # PathRiskStatePublisher: /perception/tracked_objects + /plan → /planning/path_risk_state
        Node(
            package='planning_pkg',
            executable='path_risk_state_publisher',
            name='path_risk_state_publisher',
            output='screen',
            parameters=[risk_params],
        ),
    ])
