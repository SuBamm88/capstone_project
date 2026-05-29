#!/usr/bin/env python3

import os

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector_node")

        self.bridge = CvBridge()
        self.image_topic = self.declare_parameter(
            "image_topic", "/camera2/image_raw"
        ).value
        self.annotated_topic = self.declare_parameter(
            "annotated_topic", "/yolo/annotated_image"
        ).value
        self.detections_topic = self.declare_parameter(
            "detections_topic", "/yolo/detections"
        ).value
        self.confidence = float(self.declare_parameter("confidence", 0.5).value)
        self.publish_annotated = bool(
            self.declare_parameter("publish_annotated", True).value
        )
        model_path = self.declare_parameter("model_path", "resource/best.pt").value
        resolved_model_path = self._package_path(model_path)

        self.model = YOLO(resolved_model_path)

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            self.detections_topic,
            10,
        )
        self.annotated_pub = None
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(
                Image,
                self.annotated_topic,
                10,
            )

        self.get_logger().info(
            "YOLO detector started: "
            f"model={resolved_model_path}, "
            f"{self.image_topic} -> {self.detections_topic}"
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model(frame, conf=self.confidence, verbose=False)

        detection_array_msg = Detection2DArray()
        detection_array_msg.header = msg.header

        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            confidence = float(box.conf[0])
            class_index = int(box.cls[0])
            class_name = self.model.names[class_index]

            center_x = float((x1 + x2) / 2.0)
            center_y = float((y1 + y2) / 2.0)
            size_x = float(x2 - x1)
            size_y = float(y2 - y1)

            detection_msg = Detection2D()
            detection_msg.header = msg.header
            detection_msg.bbox.center.position.x = center_x
            detection_msg.bbox.center.position.y = center_y
            detection_msg.bbox.size_x = size_x
            detection_msg.bbox.size_y = size_y

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_name
            hypothesis.hypothesis.score = confidence
            detection_msg.results.append(hypothesis)
            detection_array_msg.detections.append(detection_msg)

            if self.publish_annotated:
                self._draw_detection(frame, x1, y1, x2, y2, class_name, confidence)

            bottom_u = int(center_x)
            bottom_v = int(y2)
            self.get_logger().info(
                f"class={class_name}, score={confidence:.2f}, "
                f"bbox_center=({center_x:.1f}, {center_y:.1f}), "
                f"bbox_size=({size_x:.1f}, {size_y:.1f}), "
                f"bottom_center=({bottom_u}, {bottom_v})"
            )

        self.detections_pub.publish(detection_array_msg)

        if self.publish_annotated:
            annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)

    def _draw_detection(self, frame, x1, y1, x2, y2, class_name, confidence):
        bottom_u = int((x1 + x2) / 2.0)
        bottom_v = int(y2)

        cv2.rectangle(
            frame,
            (int(x1), int(y1)),
            (int(x2), int(y2)),
            (0, 255, 0),
            2,
        )
        cv2.circle(frame, (bottom_u, bottom_v), 6, (0, 0, 255), -1)
        cv2.putText(
            frame,
            f"{class_name} {confidence:.2f}",
            (int(x1), int(y1) - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            frame,
            f"bottom: ({bottom_u}, {bottom_v})",
            (bottom_u + 10, bottom_v - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            2,
        )

    def _package_path(self, path):
        if os.path.isabs(path):
            return path

        package_share = get_package_share_directory("perception_pkg")
        return os.path.join(package_share, path)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
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
