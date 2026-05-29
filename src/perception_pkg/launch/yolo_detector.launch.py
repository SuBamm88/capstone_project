from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    yolo_params = PathJoinSubstitution(
        [FindPackageShare("perception_pkg"), "config", "yolo_detector.yaml"]
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
        ]
    )
