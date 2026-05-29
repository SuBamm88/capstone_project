from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parent_frame = 'base_link'
    child_frame = 'velodyne'

    x = '0.0'    # +x: robot forward
    y = '0.0'     # +y: robot left
    z = '0.50'    # +z: robot up

    roll = '0.0'   # rotation about x
    pitch = '0.0'  # rotation about y
    yaw = '0.0'    # rotation about z 

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='scout_velodyne_static_tf',
            output='screen',
            arguments=[x, y, z, roll, pitch, yaw, parent_frame, child_frame],
        )
    ])
