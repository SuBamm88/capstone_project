"""Full perception layer: FOV markers + CCTV-to-map + fusion.

YOLO and cctv_to_map are expected to run per camera. This launch starts the
single-camera path plus FOV visualization and the fusion node. For a second
camera, add another yolo_detector_node + cctv_to_map with its own config and
distinct topics, then list both in fusion.yaml `cctv_topics`.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("perception_pkg")
    yolo_params = PathJoinSubstitution([share, "config", "yolo_detector.yaml"])
    cctv_to_map_params = PathJoinSubstitution([share, "config", "cctv_to_map.yaml"])
    fov_params = PathJoinSubstitution([share, "config", "cctv_fov.yaml"])
    fusion_params = PathJoinSubstitution([share, "config", "fusion.yaml"])
    lidar_params = PathJoinSubstitution([share, "config", "lidar_object.yaml"])

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
            Node(
                package="perception_pkg",
                executable="cctv_fov",
                name="cctv_fov",
                output="screen",
                parameters=[fov_params],
            ),
            Node(
                package="perception_pkg",
                executable="lidar_object",
                name="lidar_object",
                output="screen",
                parameters=[lidar_params],
            ),
            Node(
                package="perception_pkg",
                executable="fusion",
                name="fusion",
                output="screen",
                parameters=[fusion_params],
            ),
        ]
    )
