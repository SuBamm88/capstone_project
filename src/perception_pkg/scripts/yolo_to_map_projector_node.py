#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray


class YoloToMapProjectorNode(Node):
    def __init__(self):
        super().__init__("yolo_to_map_projector_node")

        self.input_topic = self.declare_parameter(
            "input_topic", "/yolo/detections"
        ).value
        self.floor_topic = self.declare_parameter(
            "floor_topic", "/cctv/objects_floor"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/cctv/objects_map"
        ).value
        self.floor_frame = self.declare_parameter("floor_frame", "floor").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.target_classes = [
            str(value)
            for value in self.declare_parameter("target_classes", ["person"]).value
        ]
        self.bbox_xy_origin = self.declare_parameter(
            "bbox_xy_origin", "center"
        ).value
        self.min_confidence = float(
            self.declare_parameter("min_confidence", 0.4).value
        )
        self.tf_timeout_sec = float(
            self.declare_parameter("tf_timeout_sec", 0.2).value
        )
        self.h_img_to_floor = np.array(
            [
                float(value)
                for value in self.declare_parameter(
                    "image_to_floor_homography",
                    [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                ).value
            ],
            dtype=np.float64,
        )
        if self.h_img_to_floor.size != 9:
            raise ValueError("image_to_floor_homography must contain 9 values")
        self.h_img_to_floor = self.h_img_to_floor.reshape((3, 3))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.floor_pub = self.create_publisher(PoseArray, self.floor_topic, 10)
        self.map_pub = self.create_publisher(PoseArray, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Detection2DArray, self.input_topic, self.detection_callback, 10
        )

        self.get_logger().info(
            "YOLO to map projector started: "
            f"{self.input_topic} (vision_msgs/msg/Detection2DArray) -> "
            f"{self.output_topic} (geometry_msgs/msg/PoseArray, frame={self.map_frame})"
        )

    def detection_callback(self, detections_msg):
        floor_output = self._new_pose_array(
            detections_msg.header.stamp,
            self.floor_frame,
        )
        map_output = self._new_pose_array(
            detections_msg.header.stamp,
            self.map_frame,
        )

        try:
            floor_to_map = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.floor_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warn(f"TF lookup failed: {exc}")
            return

        for detection in detections_msg.detections:
            class_id, score = self._best_hypothesis(detection)
            if self.target_classes and class_id not in self.target_classes:
                continue
            if score < self.min_confidence:
                continue

            bottom_u, bottom_v = self._bottom_center(detection)

            floor_x, floor_y = self._project_image_to_floor(bottom_u, bottom_v)
            floor_output.poses.append(self._new_xy_pose(floor_x, floor_y))

            map_x, map_y = self._transform_floor_to_map(floor_x, floor_y, floor_to_map)
            map_output.poses.append(self._new_xy_pose(map_x, map_y))

        self.floor_pub.publish(floor_output)
        self.map_pub.publish(map_output)

    def _new_pose_array(self, stamp, frame_id):
        output = PoseArray()
        output.header.stamp = stamp
        output.header.frame_id = frame_id
        return output

    def _new_xy_pose(self, x, y):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        return pose

    def _project_image_to_floor(self, u, v):
        point = np.array([float(u), float(v), 1.0], dtype=np.float64)
        projected = self.h_img_to_floor @ point

        if abs(projected[2]) < 1.0e-12:
            return float(projected[0]), float(projected[1])

        return float(projected[0] / projected[2]), float(projected[1] / projected[2])

    def _bottom_center(self, detection):
        x = detection.bbox.center.position.x
        y = detection.bbox.center.position.y
        width = detection.bbox.size_x
        height = detection.bbox.size_y

        if self.bbox_xy_origin == "top_left":
            return x + width / 2.0, y + height

        return x, y + height / 2.0

    def _transform_floor_to_map(self, floor_x, floor_y, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = self._yaw_from_quaternion(rotation)

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        map_x = cos_yaw * floor_x - sin_yaw * floor_y + translation.x
        map_y = sin_yaw * floor_x + cos_yaw * floor_y + translation.y
        return map_x, map_y

    def _best_hypothesis(self, detection):
        best_class_id = ""
        best_score = 0.0

        for result in detection.results:
            if result.hypothesis.score > best_score:
                best_class_id = result.hypothesis.class_id
                best_score = result.hypothesis.score

        return best_class_id, best_score

    def _yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = YoloToMapProjectorNode()
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
