#!/usr/bin/env python3

import os
import json
import math

import cv2
import numpy as np
import rclpy

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from ultralytics import YOLO

def resolve_pose_model_path():
    candidate_paths = [
        os.path.join(
            get_package_share_directory("perception_pkg"),
            "resource",
            "yolov8n-pose.pt",
        ),
    ]

    current_dir = os.path.dirname(os.path.abspath(__file__))
    search_dir = current_dir
    while True:
        candidate_paths.append(
            os.path.join(search_dir, "resource", "yolov8n-pose.pt")
        )
        parent_dir = os.path.dirname(search_dir)
        if parent_dir == search_dir:
            break
        search_dir = parent_dir

    for candidate_path in candidate_paths:
        if os.path.exists(candidate_path):
            return candidate_path

    raise FileNotFoundError(
        "Pose model not found. Checked: " + ", ".join(candidate_paths)
    )


def resolve_face_cascade_path():
    candidate_paths = [
        os.path.join(cv2.data.haarcascades, "haarcascade_frontalface_default.xml"),
    ]

    for candidate_path in candidate_paths:
        if os.path.exists(candidate_path):
            return candidate_path

    raise FileNotFoundError(
        "Face cascade not found. Checked: " + ", ".join(candidate_paths)
    )


POSE_MODEL_PATH = resolve_pose_model_path()
FACE_CASCADE_PATH = resolve_face_cascade_path()


class CCTVPoseNode(Node):
    def __init__(self):
        super().__init__("cctv_pose_node")

        self.bridge = CvBridge()
        self.model = YOLO(POSE_MODEL_PATH)
        self.face_cascade = cv2.CascadeClassifier(FACE_CASCADE_PATH)
        self.enable_pose_tracking = bool(
            self.declare_parameter("enable_pose_tracking", True).value
        )

        # 기본 카메라 파라미터
        self.camera_matrix = np.array([
            [498.64644, 0.0,       313.76504],
            [0.0,       500.76878, 248.53280],
            [0.0,       0.0,       1.0      ]
        ], dtype=np.float64)

        self.dist_coeffs = np.array(
            [0.073022, -0.19204, 0.001795, -0.005144, 0.0],
            dtype=np.float64
        )

        self.camera_info_received = False

        self.image_sub = self.create_subscription(
            Image,
            "/camera2/image_raw",
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera2/camera_info",
            self.camera_info_callback,
            10
        )

        self.annotated_pub = self.create_publisher(
            Image,
            "/cctv_pose/annotated_image",
            10
        )

        self.pose_info_pub = self.create_publisher(
            String,
            "/cctv_pose/person_pose_info",
            10
        )

        self.get_logger().info("CCTVPoseNode started.")

    def camera_info_callback(self, msg):
        if self.camera_info_received:
            return

        k = msg.k
        self.camera_matrix = np.array([
            [k[0], k[1], k[2]],
            [k[3], k[4], k[5]],
            [k[6], k[7], k[8]]
        ], dtype=np.float64)

        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        self.camera_info_received = True

        self.get_logger().info(
            f"camera_info received: fx={k[0]:.1f}, fy={k[4]:.1f}, "
            f"cx={k[2]:.1f}, cy={k[5]:.1f}"
        )

    def undistort_point(self, u, v):
        pt = np.array([[[float(u), float(v)]]], dtype=np.float64)

        dst = cv2.undistortPoints(
            pt,
            self.camera_matrix,
            self.dist_coeffs,
            P=self.camera_matrix
        )

        return float(dst[0][0][0]), float(dst[0][0][1])

    def detect_faces(self, gray_frame):
        faces = self.face_cascade.detectMultiScale(
            gray_frame,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )

        return faces if len(faces) > 0 else []

    def is_face_near_head(self, faces, nose_pt, shoulder_width):
        threshold = shoulder_width * 0.8

        for (fx, fy, fw, fh) in faces:
            fcx = fx + fw / 2.0
            fcy = fy + fh / 2.0

            if math.hypot(fcx - nose_pt[0], fcy - nose_pt[1]) < threshold:
                return True

        return False

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = self.detect_faces(gray)

        if self.enable_pose_tracking:
            try:
                results = self.model.track(
                    frame,
                    conf=0.4,
                    persist=True,
                    verbose=False,
                    tracker="bytetrack.yaml",
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"YOLO pose tracking failed, falling back to detection: {exc}"
                )
                results = self.model(frame, conf=0.4, verbose=False)
        else:
            results = self.model(frame, conf=0.4, verbose=False)
        output_list = []

        if len(results) > 0 and results[0].keypoints is not None:
            if results[0].keypoints.xy is None:
                self.publish_result(frame, msg, output_list)
                return

            keypoints_xy = results[0].keypoints.xy.cpu().numpy()

            if results[0].keypoints.conf is not None:
                keypoints_conf = results[0].keypoints.conf.cpu().numpy()
            else:
                keypoints_conf = np.ones(
                    (keypoints_xy.shape[0], keypoints_xy.shape[1]),
                    dtype=float
                )

            boxes = None
            if hasattr(results[0], "boxes"):
                boxes = list(results[0].boxes)

            for detection_idx, (kpts, confs) in enumerate(zip(keypoints_xy, keypoints_conf)):
                if kpts is None or len(kpts) < 17:
                    continue

                if confs is None or len(confs) < 17:
                    continue

                bbox = None
                if boxes is not None and detection_idx < len(boxes):
                    bbox = boxes[detection_idx]

                person_id = self.resolve_track_id(bbox, detection_idx)

                result = self.extract_pose_info(person_id, kpts, confs, faces, bbox)

                if result is None:
                    continue

                output_list.append(result)
                self.draw_result(frame, result, kpts, confs)

        self.publish_result(frame, msg, output_list)

    def resolve_track_id(self, bbox, fallback_idx):
        if bbox is None or not hasattr(bbox, "id"):
            return fallback_idx

        track_id = bbox.id
        if track_id is None:
            return fallback_idx

        try:
            if hasattr(track_id, "cpu"):
                values = track_id.cpu().numpy().reshape(-1)
                if len(values) > 0:
                    return int(values[0])
            if isinstance(track_id, (list, tuple)) and track_id:
                return int(track_id[0])
            return int(track_id)
        except Exception:
            return fallback_idx

    def extract_pose_info(self, person_id, kpts, confs, faces, bbox=None):
        min_conf = 0.35

        nose = kpts[0]
        left_ear = kpts[3]
        right_ear = kpts[4]
        left_shoulder = kpts[5]
        right_shoulder = kpts[6]

        nose_conf = float(confs[0])
        left_ear_conf = float(confs[3])
        right_ear_conf = float(confs[4])
        left_shoulder_conf = float(confs[5])
        right_shoulder_conf = float(confs[6])

        if left_shoulder_conf < min_conf or right_shoulder_conf < min_conf:
            return None

        # 원본 픽셀 좌표
        ls_raw_u = float(left_shoulder[0])
        ls_raw_v = float(left_shoulder[1])
        rs_raw_u = float(right_shoulder[0])
        rs_raw_v = float(right_shoulder[1])

        shoulder_width = math.hypot(
            ls_raw_u - rs_raw_u,
            ls_raw_v - rs_raw_v
        )

        if shoulder_width < 10.0:
            return None

        # 왜곡 보정된 픽셀 좌표
        ls_undist_u, ls_undist_v = self.undistort_point(ls_raw_u, ls_raw_v)
        rs_undist_u, rs_undist_v = self.undistort_point(rs_raw_u, rs_raw_v)

        shoulder_center_raw_u = (ls_raw_u + rs_raw_u) / 2.0
        shoulder_center_raw_v = (ls_raw_v + rs_raw_v) / 2.0

        shoulder_center_undist_u, shoulder_center_undist_v = self.undistort_point(
            shoulder_center_raw_u,
            shoulder_center_raw_v
        )

        bbox_bottom_center_raw = None
        bbox_bottom_center_undist = None
        bbox_coords = None
        if bbox is not None:
            try:
                x1, y1, x2, y2 = bbox.xyxy[0].cpu().numpy()
                bbox_coords = [float(x1), float(y1), float(x2), float(y2)]
                bbox_bottom_center_raw = ((x1 + x2) / 2.0, y2)
                bbox_bottom_center_undist = self.undistort_point(
                    bbox_bottom_center_raw[0],
                    bbox_bottom_center_raw[1],
                )
            except Exception:
                bbox_bottom_center_raw = None
                bbox_bottom_center_undist = None
                bbox_coords = None

        # 전방 / 후방 판단
        nose_visible = nose_conf >= min_conf
        left_ear_visible = left_ear_conf >= min_conf
        right_ear_visible = right_ear_conf >= min_conf
        ear_count = int(left_ear_visible) + int(right_ear_visible)
        ear_mean = (left_ear_conf + right_ear_conf) / 2.0
        ear_max = max(left_ear_conf, right_ear_conf)
        ear_min = min(left_ear_conf, right_ear_conf)

        nose_pt = (float(nose[0]), float(nose[1]))
        face_detected = self.is_face_near_head(faces, nose_pt, shoulder_width)

        if nose_visible and ear_count == 1:
            front_back = "FRONT_SIDE"
        elif face_detected and nose_visible:
            front_back = "FRONT"
        elif nose_visible:
            # 코가 분명히 보이면 기본 FRONT
            # 다만 양쪽 귀가 모두 강하고 코보다 훨씬 강하면 BACK으로 보정
            if ear_count == 2 and ear_mean > nose_conf * 1.2:
                front_back = "BACK"
            else:
                front_back = "FRONT"
        elif ear_count == 2:
            # 양쪽 귀가 모두 보이면 뒤돌아 본 것으로 판단
            front_back = "BACK"
        elif ear_count == 1:
            # 한쪽 귀만 보이고 코가 기준 미만이면 FRONT로 보지 않는다.
            if ear_max >= min_conf * 1.2 and ear_min < min_conf * 0.6:
                front_back = "BACK_SIDE"
            else:
                front_back = "BACK_SIDE"
        else:
            front_back = "BACK"

        base_conf = (left_shoulder_conf + right_shoulder_conf) / 2.0

        if face_detected:
            front_back_conf_scale = 1.0
        elif nose_visible:
            front_back_conf_scale = 0.85
        elif left_ear_visible or right_ear_visible:
            front_back_conf_scale = 0.50
        else:
            front_back_conf_scale = 0.35

        total_confidence = base_conf * front_back_conf_scale

        return {
            "person_id": person_id,

            "front_back": front_back,
            "face_detected": face_detected,
            "confidence": round(float(total_confidence), 3),

            "raw_pixel": {
                "left_shoulder": {
                    "u": round(ls_raw_u, 2),
                    "v": round(ls_raw_v, 2)
                },
                "right_shoulder": {
                    "u": round(rs_raw_u, 2),
                    "v": round(rs_raw_v, 2)
                },
                "shoulder_center": {
                    "u": round(shoulder_center_raw_u, 2),
                    "v": round(shoulder_center_raw_v, 2)
                },
                "bbox_bottom_center": {
                    "u": round(bbox_bottom_center_raw[0], 2) if bbox_bottom_center_raw is not None else None,
                    "v": round(bbox_bottom_center_raw[1], 2) if bbox_bottom_center_raw is not None else None
                },
                "bbox": bbox_coords,
            },

            "undistorted_pixel": {
                "left_shoulder": {
                    "u": round(ls_undist_u, 2),
                    "v": round(ls_undist_v, 2)
                },
                "right_shoulder": {
                    "u": round(rs_undist_u, 2),
                    "v": round(rs_undist_v, 2)
                },
                "shoulder_center": {
                    "u": round(shoulder_center_undist_u, 2),
                    "v": round(shoulder_center_undist_v, 2)
                },
                "bbox_bottom_center": {
                    "u": round(bbox_bottom_center_undist[0], 2) if bbox_bottom_center_undist is not None else None,
                    "v": round(bbox_bottom_center_undist[1], 2) if bbox_bottom_center_undist is not None else None
                }
            },

            "keypoint_confidence": {
                "nose": round(nose_conf, 3),
                "left_ear": round(left_ear_conf, 3),
                "right_ear": round(right_ear_conf, 3),
                "left_shoulder": round(left_shoulder_conf, 3),
                "right_shoulder": round(right_shoulder_conf, 3)
            },

            "_left_shoulder_raw": [ls_raw_u, ls_raw_v],
            "_right_shoulder_raw": [rs_raw_u, rs_raw_v],
            "_shoulder_center_raw": [shoulder_center_raw_u, shoulder_center_raw_v],
            "_nose_raw": [float(nose[0]), float(nose[1])]
        }

    def publish_result(self, frame, msg, output_list):
        clean_list = [
            {k: v for k, v in item.items() if not k.startswith("_")}
            for item in output_list
        ]
        clean_list = self._json_safe(clean_list)

        pose_msg = String()
        pose_msg.data = json.dumps(clean_list, ensure_ascii=False)
        self.pose_info_pub.publish(pose_msg)

        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

    def _json_safe(self, value):
        if isinstance(value, dict):
            return {str(k): self._json_safe(v) for k, v in value.items()}
        if isinstance(value, list):
            return [self._json_safe(v) for v in value]
        if isinstance(value, tuple):
            return [self._json_safe(v) for v in value]
        if isinstance(value, (np.integer, np.int32, np.int64)):
            return int(value)
        if isinstance(value, (np.floating, np.float32, np.float64)):
            return float(value)
        return value

    def draw_result(self, frame, result, kpts, confs):
        left_shoulder = result["_left_shoulder_raw"]
        right_shoulder = result["_right_shoulder_raw"]
        shoulder_center = result["_shoulder_center_raw"]
        nose = result["_nose_raw"]

        ls_pt = (int(left_shoulder[0]), int(left_shoulder[1]))
        rs_pt = (int(right_shoulder[0]), int(right_shoulder[1]))
        sc_pt = (int(shoulder_center[0]), int(shoulder_center[1]))
        nose_pt = (int(nose[0]), int(nose[1]))

        # 주요 keypoint 표시
        for idx in [0, 3, 4, 5, 6]:
            if idx < len(confs) and confs[idx] > 0.35:
                x, y = kpts[idx]
                cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 255), -1)

        # 어깨선
        cv2.line(frame, ls_pt, rs_pt, (255, 0, 0), 3)

        # 왼쪽 어깨, 오른쪽 어깨
        cv2.circle(frame, ls_pt, 7, (0, 255, 0), -1)
        cv2.circle(frame, rs_pt, 7, (0, 128, 255), -1)

        # 어깨 중심
        cv2.circle(frame, sc_pt, 7, (255, 0, 255), -1)

        # 코
        cv2.circle(frame, nose_pt, 6, (0, 0, 255), -1)

        txt_color = (0, 255, 0) if result["front_back"] == "FRONT" else (0, 165, 255)

        cv2.putText(
            frame,
            f"ID:{result['person_id']} {result['front_back']} conf={result['confidence']:.2f}",
            (sc_pt[0] + 15, sc_pt[1] - 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            txt_color,
            2
        )

        cv2.putText(
            frame,
            f"L({ls_pt[0]},{ls_pt[1]}) R({rs_pt[0]},{rs_pt[1]})",
            (sc_pt[0] + 15, sc_pt[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 0),
            1
        )


def main(args=None):
    rclpy.init(args=args)
    node = CCTVPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
