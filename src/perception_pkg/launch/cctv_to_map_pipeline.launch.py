from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("perception_pkg")
    yolo_params = PathJoinSubstitution(
        [package_share, "config", "yolo_detector.yaml"]
    )
    projector_params = PathJoinSubstitution(
        [package_share, "config", "yolo_to_map_projector.yaml"]
    )
    image_to_floor_params = PathJoinSubstitution(
        [package_share, "config", "image_to_floor.yaml"]
    )
    floor_to_map_tf_params = PathJoinSubstitution(
        [package_share, "config", "floor_to_map_tf.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="perception_pkg",
                executable="yolo_detector_node.py",
                name="yolo_detector_node",
                output="screen",
                parameters=[yolo_params],
            ),
            Node(
                package="perception_pkg",
                executable="floor_to_map_tf_broadcaster_node.py",
                name="floor_to_map_tf_broadcaster_node",
                output="screen",
                parameters=[floor_to_map_tf_params],
            ),
            Node(
                package="perception_pkg",
                executable="yolo_to_map_projector_node.py",
                name="yolo_to_map_projector_node",
                output="screen",
                parameters=[projector_params, image_to_floor_params],
            ),
        ]
    )
