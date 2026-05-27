from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("perception_pkg")
    params = PathJoinSubstitution(
        [package_share, "config", "floor_to_map_calibrator.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="perception_pkg",
                executable="aruco_floor_to_map_calibrator.py",
                name="aruco_floor_to_map_calibrator",
                output="screen",
                parameters=[params],
            )
        ]
    )
