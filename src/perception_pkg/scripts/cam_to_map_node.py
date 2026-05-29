#!/usr/bin/env python3

import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


class CamToMapNode(Node):
    def __init__(self):
        super().__init__("cam_to_map")

        self.bridge = CvBridge()
        self.detections_topic = self.declare_parameter(
            "detections_topic", "/yolo/detections"
        ).value
        self.slam_map_topic = self.declare_parameter(
            "slam_map_topic", "/cctv/slam_map"
        ).value
        map_image_path = self.declare_parameter(
            "map_image_path", "resource/scout_mini_map_3.pgm"
        ).value

        image_points = self.declare_parameter(
            "image_points",
            [
                441.0,
                366.0,
                583.0,
                200.0,
                622.0,
                152.0,
                440.0,
                157.0,
                306.0,
                163.0,
                208.0,
                199.0,
                84.0,
                243.0,
                101.0,
                411.0,
            ],
        ).value
        map_points = self.declare_parameter(
            "map_points",
            [
                208.0,
                471.0,
                262.0,
                468.0,
                343.0,
                456.0,
                348.0,
                412.0,
                337.0,
                365.0,
                269.0,
                369.0,
                208.0,
                375.0,
                207.0,
                432.0,
            ],
        ).value

        self.map_image_path = self._package_path(map_image_path)
        self.map_img_gray = cv2.imread(self.map_image_path, cv2.IMREAD_GRAYSCALE)
        if self.map_img_gray is None:
            raise RuntimeError(f"Failed to load SLAM map image: {self.map_image_path}")

        self.map_img = cv2.cvtColor(self.map_img_gray, cv2.COLOR_GRAY2BGR)
        self.image_points = self._points_from_parameter(image_points, "image_points")
        self.map_points = self._points_from_parameter(map_points, "map_points")

        self.h_cctv_to_map, self.mask = cv2.findHomography(
            self.image_points,
            self.map_points,
            method=cv2.RANSAC,
            ransacReprojThreshold=0.15,
        )
        if self.h_cctv_to_map is None:
            raise RuntimeError("Failed to compute CCTV-to-map homography")

        self.bbox_sub = self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self.bbox_callback,
            1,
        )
        self.slam_map_pub = self.create_publisher(
            Image,
            self.slam_map_topic,
            1,
        )

        self.get_logger().info(
            "cam_to_map started: "
            f"{self.detections_topic} -> {self.slam_map_topic}, "
            f"map={self.map_image_path}"
        )
        self.get_logger().info(f"H_cctv_to_map:\n{self.h_cctv_to_map}")

    def bbox_callback(self, msg):
        draw_img = self.map_img.copy()

        for det in msg.detections:
            bbox = det.bbox
            u = float(bbox.center.position.x)
            v = float(bbox.center.position.y + bbox.size_y / 2.0)
            map_x, map_y = self.pixel_to_map(u, v)

            if not self._in_map_image(map_x, map_y):
                self.get_logger().warn(
                    f"Projected point is outside map image: ({map_x}, {map_y})"
                )
                continue

            cv2.circle(draw_img, (map_x, map_y), 8, (255, 255, 255), -1)
            cv2.circle(draw_img, (map_x, map_y), 6, (0, 0, 255), -1)
            cv2.circle(draw_img, (map_x, map_y), 8, (0, 0, 0), 1)
            cv2.putText(
                draw_img,
                f"({map_x}, {map_y})",
                (map_x + 8, map_y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )

            self.get_logger().info(
                f"bbox bottom pixel=({u:.1f}, {v:.1f}) -> map image=({map_x}, {map_y})"
            )

        out_msg = self.bridge.cv2_to_imgmsg(draw_img, encoding="bgr8")
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = "map"
        self.slam_map_pub.publish(out_msg)

    def pixel_to_map(self, u, v):
        pixel_point = np.array([[[u, v]]], dtype=np.float32)
        map_point = cv2.perspectiveTransform(pixel_point, self.h_cctv_to_map)
        x, y = map_point[0, 0]
        return int(round(x)), int(round(y))

    def _in_map_image(self, x, y):
        height, width = self.map_img.shape[:2]
        return 0 <= x < width and 0 <= y < height

    def _points_from_parameter(self, values, name):
        if len(values) % 2 != 0 or len(values) < 8:
            raise ValueError(f"{name} must contain at least four x/y pairs")

        return np.array(values, dtype=np.float32).reshape((-1, 2))

    def _package_path(self, path):
        if os.path.isabs(path):
            return path

        package_share = get_package_share_directory("perception_pkg")
        return os.path.join(package_share, path)


def main(args=None):
    rclpy.init(args=args)
    node = CamToMapNode()
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
