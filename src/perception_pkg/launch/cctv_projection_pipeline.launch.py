from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("perception_pkg")
    ground_contact_params = PathJoinSubstitution(
        [package_share, "config", "ground_contact_extractor.yaml"]
    )
    floor_to_map_params = PathJoinSubstitution(
        [package_share, "config", "floor_to_map.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="perception_pkg",
                executable="ground_contact_extractor_node",
                name="ground_contact_extractor_node",
                output="screen",
                parameters=[ground_contact_params],
            ),
            Node(
                package="perception_pkg",
                executable="floor_to_map_node",
                name="floor_to_map_node",
                output="screen",
                parameters=[floor_to_map_params],
            ),
        ]
    )
