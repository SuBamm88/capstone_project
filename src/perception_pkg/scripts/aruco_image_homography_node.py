#!/usr/bin/env python3

import os

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


class ArucoImageHomographyNode(Node):
    def __init__(self):
        super().__init__("aruco_image_homography_node")

        self.bridge = CvBridge()

        self.image_path = self.declare_parameter(
            "image_path", ""
        ).value
        self.image_topic = self.declare_parameter(
            "image_topic", "/camera1/image_raw"
        ).value
        self.detection_topic = self.declare_parameter(
            "detection_topic", "/yolo/detections"
        ).value
        self.topview_topic = self.declare_parameter(
            "topview_topic", "/cctv/topview_image"
        ).value
        self.output_yaml = self.declare_parameter(
            "output_yaml", "config/image_to_floor.yaml"
        ).value
        self.aruco_dictionary = self.declare_parameter(
            "aruco_dictionary", "DICT_4X4_250"
        ).value
        self.marker_ids = [
            int(value)
            for value in self.declare_parameter("marker_ids", [1, 2, 3, 4]).value
        ]
        self.marker_floor_points = [
            float(value)
            for value in self.declare_parameter(
                "marker_floor_points",
                [0.0, 0.0, 0.4, 0.0, 0.4, 0.3, 0.0, 0.3],
            ).value
        ]
        self.floor_frame = self.declare_parameter("floor_frame", "floor").value
        self.scale = float(self.declare_parameter("scale", 1000.0).value)
        self.margin = int(self.declare_parameter("margin", 100).value)
        self.world_width = float(self.declare_parameter("world_width", 0.4).value)
        self.world_height = float(self.declare_parameter("world_height", 0.3).value)
        self.publish_topview = bool(
            self.declare_parameter("publish_topview", True).value
        )

        self.marker_floor_pos = self._build_marker_floor_pos()
        self.topview_width = int(self.world_width * self.scale) + 2 * self.margin
        self.topview_height = int(self.world_height * self.scale) + 2 * self.margin
        self.h_img_to_floor = None
        self.h_img_to_top = None
        self.latest_detections = None

        self.image_path = self._package_path(self.image_path) if self.image_path else ""
        self.output_yaml = self._package_path(self.output_yaml)

        if self.image_path:
            image = cv2.imread(self.image_path)
            if image is None:
                self.get_logger().error(f"이미지 로딩 실패: {self.image_path}")
            else:
                self._compute_and_save_homography(image)

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        self.bbox_sub = self.create_subscription(
            Detection2DArray, self.detection_topic, self.bbox_callback, 10
        )
        self.topview_pub = self.create_publisher(Image, self.topview_topic, 10)

        self.get_logger().info("Aruco image homography node started")

    def _build_marker_floor_pos(self):
        if len(self.marker_floor_points) != len(self.marker_ids) * 2:
            raise ValueError("marker_floor_points must contain x/y for each marker id")

        marker_floor_pos = {}
        for index, marker_id in enumerate(self.marker_ids):
            point_index = index * 2
            marker_floor_pos[marker_id] = [
                self.marker_floor_points[point_index],
                self.marker_floor_points[point_index + 1],
            ]
        return marker_floor_pos

    def _compute_and_save_homography(self, image):
        h_img_to_floor = self.compute_homography(image)
        if h_img_to_floor is None:
            self.get_logger().error("Homography 계산 실패")
            return

        self.h_img_to_floor = h_img_to_floor
        self.h_img_to_top = self._world_to_top_matrix() @ self.h_img_to_floor
        self.save_homography()

    def _world_to_top_matrix(self):
        return np.array(
            [
                [self.scale, 0.0, self.margin],
                [0.0, -self.scale, self.margin + self.world_height * self.scale],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

    def compute_homography(self, image):
        aruco_dict = cv2.aruco.getPredefinedDictionary(
            self._dictionary_id(self.aruco_dictionary)
        )
        parameters = self._detector_parameters()

        if hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
            corners, ids, _ = detector.detectMarkers(image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                image, aruco_dict, parameters=parameters
            )

        image_points = []
        floor_points = []

        if ids is not None:
            for corner, marker_id in zip(corners, ids.flatten()):
                marker_id = int(marker_id)
                if marker_id not in self.marker_floor_pos:
                    continue

                pts = corner[0]
                center_u = float(np.mean(pts[:, 0]))
                center_v = float(np.mean(pts[:, 1]))
                image_points.append([center_u, center_v])
                floor_points.append(self.marker_floor_pos[marker_id])

        image_points = np.array(image_points, dtype=np.float32)
        floor_points = np.array(floor_points, dtype=np.float32)

        if len(image_points) < 4:
            self.get_logger().error(
                f"Homography needs at least 4 known markers, got {len(image_points)}"
            )
            return None

        h_img_to_floor, _ = cv2.findHomography(image_points, floor_points)
        if h_img_to_floor is None:
            return None

        self.get_logger().info(f"\nHomography image -> floor:\n{h_img_to_floor}")
        return h_img_to_floor.astype(np.float32)

    def save_homography(self):
        if self.h_img_to_floor is None:
            return

        output_dir = os.path.dirname(os.path.abspath(self.output_yaml))
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        data = {
            "yolo_to_map_projector_node": {
                "ros__parameters": {
                    "floor_frame": self.floor_frame,
                    "image_to_floor_homography": [
                        float(value) for value in self.h_img_to_floor.reshape(-1)
                    ],
                }
            }
        }

        with open(self.output_yaml, "w", encoding="utf-8") as yaml_file:
            yaml.safe_dump(data, yaml_file, sort_keys=False)

        self.get_logger().info(f"Saved image-to-floor homography: {self.output_yaml}")

    def bbox_callback(self, msg):
        self.latest_detections = msg

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if self.h_img_to_floor is None:
            self._compute_and_save_homography(frame)

        if not self.publish_topview or self.h_img_to_top is None:
            return

        topview = cv2.warpPerspective(
            frame, self.h_img_to_top, (self.topview_width, self.topview_height)
        )

        if self.latest_detections is not None:
            for det in self.latest_detections.detections:
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
                bottom_u = cx
                bottom_v = cy + det.bbox.size_y / 2.0

                img_pt = np.array([[[bottom_u, bottom_v]]], dtype=np.float32)
                top_pt = cv2.perspectiveTransform(img_pt, self.h_img_to_top)
                top_x = int(top_pt[0, 0, 0])
                top_y = int(top_pt[0, 0, 1])

                cv2.circle(topview, (top_x, top_y), 6, (0, 0, 255), -1)
                cv2.putText(
                    topview,
                    "obj",
                    (top_x + 8, top_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    2,
                )

        topview_msg = self.bridge.cv2_to_imgmsg(topview, encoding="bgr8")
        topview_msg.header = msg.header
        self.topview_pub.publish(topview_msg)

    def _dictionary_id(self, dictionary_name):
        if not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(f"Unsupported ArUco dictionary: {dictionary_name}")
        return getattr(cv2.aruco, dictionary_name)

    def _detector_parameters(self):
        if hasattr(cv2.aruco, "DetectorParameters"):
            return cv2.aruco.DetectorParameters()
        return cv2.aruco.DetectorParameters_create()

    def _package_path(self, path):
        if os.path.isabs(path):
            return path

        package_share = get_package_share_directory("perception_pkg")
        return os.path.join(package_share, path)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoImageHomographyNode()
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
