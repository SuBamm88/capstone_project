from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cloud_in_arg = DeclareLaunchArgument(
        'cloud_in', default_value='/velodyne_points',
        description='PointCloud2 input topic from an already running velodyne stack')
    scan_arg = DeclareLaunchArgument(
        'scan', default_value='/scan',
        description='LaserScan output topic for downstream consumers such as slam_toolbox')
    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value='',
        description='Optional target frame for projection. Leave empty to use the input cloud frame.')
    transform_tolerance_arg = DeclareLaunchArgument(
        'transform_tolerance', default_value='0.05',
        description='TF transform tolerance in seconds')
    min_height_arg = DeclareLaunchArgument(
        'min_height', default_value='-0.10',
        description='Minimum z height to keep during projection')
    max_height_arg = DeclareLaunchArgument(
        'max_height', default_value='0.10',
        description='Maximum z height to keep during projection')
    angle_min_arg = DeclareLaunchArgument(
        'angle_min', default_value='-3.14159',
        description='Minimum output scan angle in radians')
    angle_max_arg = DeclareLaunchArgument(
        'angle_max', default_value='3.14159',
        description='Maximum output scan angle in radians')
    angle_increment_arg = DeclareLaunchArgument(
        'angle_increment', default_value='0.01745',
        description='Output scan angular resolution in radians')
    scan_time_arg = DeclareLaunchArgument(
        'scan_time', default_value='0.1',
        description='Output scan_time field in seconds')
    range_min_arg = DeclareLaunchArgument(
        'range_min', default_value='0.5',
        description='Minimum range to report in meters')
    range_max_arg = DeclareLaunchArgument(
        'range_max', default_value='30.0',
        description='Maximum range to report in meters')
    use_inf_arg = DeclareLaunchArgument(
        'use_inf', default_value='true',
        description='Whether to use +inf for empty ranges')
    inf_epsilon_arg = DeclareLaunchArgument(
        'inf_epsilon', default_value='1.0',
        description='Offset used when use_inf is false')

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        output='both',
        remappings=[
            ('cloud_in', LaunchConfiguration('cloud_in')),
            ('scan', LaunchConfiguration('scan')),
        ],
        parameters=[{
            'target_frame': LaunchConfiguration('target_frame'),
            'transform_tolerance': LaunchConfiguration('transform_tolerance'),
            'min_height': LaunchConfiguration('min_height'),
            'max_height': LaunchConfiguration('max_height'),
            'angle_min': LaunchConfiguration('angle_min'),
            'angle_max': LaunchConfiguration('angle_max'),
            'angle_increment': LaunchConfiguration('angle_increment'),
            'scan_time': LaunchConfiguration('scan_time'),
            'range_min': LaunchConfiguration('range_min'),
            'range_max': LaunchConfiguration('range_max'),
            'use_inf': LaunchConfiguration('use_inf'),
            'inf_epsilon': LaunchConfiguration('inf_epsilon'),
        }],
    )

    return LaunchDescription([
        cloud_in_arg,
        scan_arg,
        target_frame_arg,
        transform_tolerance_arg,
        min_height_arg,
        max_height_arg,
        angle_min_arg,
        angle_max_arg,
        angle_increment_arg,
        scan_time_arg,
        range_min_arg,
        range_max_arg,
        use_inf_arg,
        inf_epsilon_arg,
        pointcloud_to_laserscan_node,
    ])
