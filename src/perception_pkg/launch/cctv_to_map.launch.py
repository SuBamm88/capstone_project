from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("perception_pkg")
    yolo_params = PathJoinSubstitution([package_share, "config", "yolo_detector.yaml"])
    cctv_to_map_params = PathJoinSubstitution(
        [package_share, "config", "cctv_to_map.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="perception_pkg",
                executable="yolo_detector_node",
                name="yolo_detector_node",
                output="screen",
                parameters=[yolo_params],
            ),
            Node(
                package="perception_pkg",
                executable="cctv_to_map",
                name="cctv_to_map",
                output="screen",
                parameters=[cctv_to_map_params],
            ),
        ]
    )
