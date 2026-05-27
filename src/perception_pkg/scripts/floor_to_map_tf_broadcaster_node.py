#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class FloorToMapTfBroadcasterNode(Node):
    def __init__(self):
        super().__init__("floor_to_map_tf_broadcaster_node")

        self.parent_frame = self.declare_parameter("parent_frame", "map").value
        self.child_frame = self.declare_parameter("child_frame", "floor").value
        self.x = float(self.declare_parameter("x", 0.0).value)
        self.y = float(self.declare_parameter("y", 0.0).value)
        self.z = float(self.declare_parameter("z", 0.0).value)
        self.roll = float(self.declare_parameter("roll", 0.0).value)
        self.pitch = float(self.declare_parameter("pitch", 0.0).value)
        self.yaw = float(self.declare_parameter("yaw", 0.0).value)

        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_transform()

    def publish_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = self.z

        qx, qy, qz, qw = self._quaternion_from_euler(
            self.roll, self.pitch, self.yaw
        )
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.broadcaster.sendTransform(transform)
        self.get_logger().info(
            f"Broadcasting static TF {self.parent_frame} -> {self.child_frame}: "
            f"x={self.x:.6f}, y={self.y:.6f}, yaw={self.yaw:.6f}"
        )

    def _quaternion_from_euler(self, roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = FloorToMapTfBroadcasterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
