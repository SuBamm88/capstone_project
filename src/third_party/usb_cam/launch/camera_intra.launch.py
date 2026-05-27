# camera_intra.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Mission manager parameter file
    camera4_parameter = DeclareLaunchArgument('camera4_parameter',
        default_value=PathJoinSubstitution([
            FindPackageShare('usb_cam'), 'config', 'params_4.yaml'
        ])
    )

    camera4_component = ComposableNode(
        package         = 'usb_cam',
        plugin          = 'usb_cam::UsbCamNode',
        name            = 'camera4',
        parameters      = [{LaunchConfiguration('camera4_parameter')}],
        extra_arguments = [{'use_intra_process_comms': True}]
    )

    # Node container
    container = ComposableNodeContainer(
        name       = 'camera_intra_container',
        namespace  = '',
        package    = 'rclcpp_components',
        executable = 'component_container_mt',
        output     = 'screen',
        composable_node_descriptions = [
            camera4_component
        ]
    )
    return LaunchDescription([
        camera4_parameter,
        container
    ])
