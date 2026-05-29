from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_frame', default_value='map'),
        DeclareLaunchArgument('objects_topic', default_value='/cctv/objects_map'),
        DeclareLaunchArgument('publish_rate', default_value='5.0'),
        DeclareLaunchArgument('marker_radius', default_value='0.25'),
        Node(
            package='cctv_object_tools',
            executable='rviz_click_to_pose_array',
            name='rviz_click_to_pose_array',
            output='screen',
            parameters=[{
                'output_frame': LaunchConfiguration('output_frame'),
                'objects_topic': LaunchConfiguration('objects_topic'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'marker_radius': LaunchConfiguration('marker_radius'),
            }],
        ),
    ])
