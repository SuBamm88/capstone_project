import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory("bringup_pkg")
    default_config = os.path.join(
        package_share, "config", "cctv_objects_bridge.yaml")

    from_domain = LaunchConfiguration("from_domain")
    to_domain = LaunchConfiguration("to_domain")
    config = LaunchConfiguration("config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "from_domain",
            default_value="72",
            description="ROS_DOMAIN_ID used by the CCTV computer",
        ),
        DeclareLaunchArgument(
            "to_domain",
            default_value="10",
            description="ROS_DOMAIN_ID used by the robot computer",
        ),
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Domain bridge config file",
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "domain_bridge", "domain_bridge",
                "--from", from_domain,
                "--to", to_domain,
                config,
            ],
            output="screen",
        ),
    ])
