#!/usr/bin/env python3

import os
import json

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from ultralytics import YOLO
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose
)

def resolve_model_path():
    candidate_paths = [
        os.path.join(
            get_package_share_directory("perception_pkg"),
            "resource",
            "best.pt",
        ),
    ]

    current_dir = os.path.dirname(os.path.abspath(__file__))
    search_dir = current_dir
    while True:
        candidate_paths.append(os.path.join(search_dir, "resource", "best.pt"))
        parent_dir = os.path.dirname(search_dir)
        if parent_dir == search_dir:
            break
        search_dir = parent_dir

    for candidate_path in candidate_paths:
        if os.path.exists(candidate_path):
            return candidate_path

    raise FileNotFoundError(
        "YOLO model not found. Checked: " + ", ".join(candidate_paths)
    )


MODEL_PATH = resolve_model_path()

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self.bridge = CvBridge()
        self.model = YOLO(MODEL_PATH)

        self.image_sub = self.create_subscription(
            Image,
            '/camera2/image_raw',
            self.image_callback,
            10
        )

        self.annotated_pub = self.create_publisher(
            Image,
            '/yolo/annotated_image',
            10
        )

        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10
        )

        self.markers_pub = self.create_publisher(
            MarkerArray,
            '/yolo/markers',
            10
        )

        self.get_logger().info('YOLO detector node started.')

    def get_anchor_point(self, name, x1, y1, x2, y2):
        center_x = float((x1 + x2) / 2.0)
        center_y = float((y1 + y2) / 2.0)
        height = float(y2 - y1)

        if name == 'person':
            # 사람: bbox 하단 중앙보다 살짝 위쪽의 보정된 발점
            k_person = 0.90
            anchor_u = center_x
            anchor_v = float(y1 + k_person * height)
            anchor_type = 'corrected_footpoint'

        elif name == 'box':
            # 상자: CCTV 비스듬한 시점 기준 바닥면 중심에 가까운 점
            k_box = 0.60
            anchor_u = center_x
            anchor_v = float(y1 + k_box * height)
            anchor_type = 'box_ground_center'

        else:
            anchor_u = center_x
            anchor_v = center_y
            anchor_type = 'bbox_center'

        return anchor_u, anchor_v, anchor_type

    def get_marker_color(self, name):
        if name == 'person':
            return ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        if name == 'box':
            return ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        return ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

    def get_cv_color(self, name):
        if name == 'person':
            return (0, 0, 255)
        if name == 'box':
            return (255, 0, 0)
        return (0, 255, 255)

    def make_point(self, x, y, z=0.0):
        return Point(x=float(x), y=float(y), z=float(z))

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(frame, conf=0.5, verbose=False)

        detection_array_msg = Detection2DArray()
        detection_array_msg.header = msg.header

        marker_array_msg = MarkerArray()
        marker_header = msg.header

        if not marker_header.frame_id:
            marker_header.frame_id = 'image'

        delete_marker = Marker()
        delete_marker.header = marker_header
        delete_marker.action = Marker.DELETEALL
        marker_array_msg.markers.append(delete_marker)

        for index, box in enumerate(results[0].boxes):
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            name = self.model.names[cls]

            center_x = float((x1 + x2) / 2.0)
            center_y = float((y1 + y2) / 2.0)
            size_x = float(x2 - x1)
            size_y = float(y2 - y1)

            anchor_u, anchor_v, anchor_type = self.get_anchor_point(
                name, x1, y1, x2, y2
            )

            detection_msg = Detection2D()
            detection_msg.header = msg.header

            # bbox 자체 정보
            detection_msg.bbox.center.position.x = center_x
            detection_msg.bbox.center.position.y = center_y
            detection_msg.bbox.size_x = size_x
            detection_msg.bbox.size_y = size_y

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = name
            hypothesis.hypothesis.score = conf

            # Homography에서 사용할 실제 anchor pixel 좌표
            hypothesis.pose.pose.position.x = float(anchor_u)
            hypothesis.pose.pose.position.y = float(anchor_v)
            hypothesis.pose.pose.position.z = 0.0

            detection_msg.results.append(hypothesis)

            # 보조 정보: echo로 확인하기 쉽게 JSON 저장
            detection_msg.id = json.dumps({
                "class": name,
                "anchor_type": anchor_type,
                "anchor_u": float(anchor_u),
                "anchor_v": float(anchor_v),
                "bbox_center_u": float(center_x),
                "bbox_center_v": float(center_y),
                "bbox": [
                    float(x1),
                    float(y1),
                    float(x2),
                    float(y2)
                ],
                "confidence": conf
            })

            detection_array_msg.detections.append(detection_msg)

            marker_color = self.get_marker_color(name)
            cv_color = self.get_cv_color(name)

            # bbox marker
            bbox_marker = Marker()
            bbox_marker.header = marker_header
            bbox_marker.ns = 'bbox'
            bbox_marker.id = index * 2
            bbox_marker.type = Marker.LINE_STRIP
            bbox_marker.action = Marker.ADD
            bbox_marker.scale.x = 3.0
            bbox_marker.color = marker_color
            bbox_marker.pose.orientation.w = 1.0
            bbox_marker.points = [
                self.make_point(x1, y1),
                self.make_point(x2, y1),
                self.make_point(x2, y2),
                self.make_point(x1, y2),
                self.make_point(x1, y1),
            ]
            marker_array_msg.markers.append(bbox_marker)

            # anchor marker
            anchor_marker = Marker()
            anchor_marker.header = marker_header
            anchor_marker.ns = 'anchor'
            anchor_marker.id = index * 2 + 1
            anchor_marker.type = Marker.SPHERE
            anchor_marker.action = Marker.ADD
            anchor_marker.scale.x = 10.0
            anchor_marker.scale.y = 10.0
            anchor_marker.scale.z = 10.0
            anchor_marker.color = marker_color
            anchor_marker.pose.orientation.w = 1.0
            anchor_marker.pose.position.x = float(anchor_u)
            anchor_marker.pose.position.y = float(anchor_v)
            anchor_marker.pose.position.z = 0.0
            marker_array_msg.markers.append(anchor_marker)

            # OpenCV 시각화
            cv2.rectangle(
                frame,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                (0, 255, 0),
                2
            )

            cv2.circle(
                frame,
                (int(anchor_u), int(anchor_v)),
                7,
                cv_color,
                -1
            )

            cv2.putText(
                frame,
                f'{name} {conf:.2f}',
                (int(x1), int(y1) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                f'{anchor_type}: ({int(anchor_u)}, {int(anchor_v)})',
                (int(anchor_u) + 10, int(anchor_v) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                cv_color,
                2
            )

            self.get_logger().info(
                f'class={name}, score={conf:.2f}, '
                f'bbox_center=({center_x:.1f}, {center_y:.1f}), '
                f'bbox_size=({size_x:.1f}, {size_y:.1f}), '
                f'anchor_type={anchor_type}, '
                f'anchor=({anchor_u:.1f}, {anchor_v:.1f})'
            )

        self.detections_pub.publish(detection_array_msg)
        self.markers_pub.publish(marker_array_msg)

        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
