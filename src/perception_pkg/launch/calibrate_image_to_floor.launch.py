from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("perception_pkg")
    params = PathJoinSubstitution(
        [package_share, "config", "aruco_image_homography.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="perception_pkg",
                executable="aruco_image_homography_node.py",
                name="aruco_image_homography_node",
                output="screen",
                parameters=[params],
            )
        ]
    )
