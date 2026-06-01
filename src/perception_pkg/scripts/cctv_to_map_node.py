#!/usr/bin/env python3
"""YOLO 탐지(또는 팀원 CCTV 추적기 출력)를 SLAM 맵의 미터 좌표로 투영하는 노드.

기존 cam_to_map(디버그용, 맵 *이미지*에 픽셀로 점을 찍음)과 달리, 이 노드는
바닥 평면 homography로 검출 지점을 곧바로 map frame의 미터 좌표로 변환한다.
이 미터 출력이 Nav2 CctvLayer와 fusion 노드가 사용하는 실제 입력이다.

팀원 연동: 입력은 vision_msgs/Detection2DArray 한 종류다. YOLO 직후든, 그 뒤에
붙는 추적기든 같은 메시지 타입이면 된다. Detection2D.id(추적 ID)가 채워져 있으면
그대로 보존하고, 그 ID로 map 위치를 시간 차분해 속도[m/s]까지 계산한다.

카메라가 2대면 이 노드를 카메라당 1개씩(서로 다른 토픽/캘리브레이션) 실행한다.
"""

import numpy as np
import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseArray, Vector3
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from vision_msgs.msg import Detection2DArray

from perception_msgs.msg import TrackedObject, TrackedObjectArray


class CctvToMapNode(Node):
    def __init__(self):
        super().__init__("cctv_to_map")

        # 입력: YOLO 또는 팀원 추적기의 Detection2DArray 토픽
        self.detections_topic = self.declare_parameter(
            "detections_topic", "/yolo/detections"
        ).value
        # 출력1: Nav2 CctvLayer가 쓰는 PoseArray (map frame, 단위 m)
        self.objects_topic = self.declare_parameter(
            "objects_topic", "/cctv/objects_map"
        ).value
        # 출력2: class/id/속도를 담은 TrackedObjectArray (fusion이 사용)
        self.objects_full_topic = self.declare_parameter(
            "objects_full_topic", "/cctv/objects"
        ).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.source_name = self.declare_parameter("source_name", "CCTV").value
        # 속도 EMA 평활 계수 [0~1]: 클수록 최신값 반영(노이즈↑), 작을수록 부드러움
        self.vel_alpha = float(self.declare_parameter("vel_alpha", 0.5).value)
        # 속도 차분 최소 시간 간격 [s]: 너무 짧은 dt는 픽셀 노이즈를 증폭하므로 건너뜀
        self.min_dt = float(self.declare_parameter("min_dt", 0.05).value)
        # 추적 ID 상태 보관 시간 [s]: 이 시간 넘게 안 보이면 ID 상태 폐기
        self.track_ttl = float(self.declare_parameter("track_ttl", 2.0).value)

        # image_points: 카메라 이미지의 픽셀 (u, v) 쌍 [px]
        # map_points:   대응하는 바닥 지점의 map frame 좌표 [m]
        #               (로봇을 마커 위에 올리고 `tf2_echo map base_link`로 측정)
        # 두 리스트는 같은 순서, 같은 길이(최소 4쌍, 권장 8쌍 이상)여야 한다.
        double_array = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        image_points = self.declare_parameter(
            "image_points", [0.0], double_array
        ).value
        map_points = self.declare_parameter("map_points", [0.0], double_array).value

        self.image_points = self._pairs(image_points, "image_points")  # [px]
        self.map_points = self._pairs(map_points, "map_points")        # [m]
        if len(self.image_points) != len(self.map_points):
            raise ValueError("image_points and map_points must have the same length")

        # 바닥 평면 가정 하의 픽셀→미터 변환 행렬(3x3). RANSAC 재투영 임계 0.05 m.
        self.h_cam_to_map, _ = cv2.findHomography(
            self.image_points, self.map_points, method=cv2.RANSAC,
            ransacReprojThreshold=0.05,
        )
        if self.h_cam_to_map is None:
            raise RuntimeError("Failed to compute camera-to-map homography")

        # 추적 ID별 속도 추정 상태: id -> (t[s], x[m], y[m], vx[m/s], vy[m/s])
        self.track_state = {}

        self.detections_sub = self.create_subscription(
            Detection2DArray, self.detections_topic, self.detections_callback, 10
        )
        self.pose_pub = self.create_publisher(PoseArray, self.objects_topic, 10)
        self.objects_pub = self.create_publisher(
            TrackedObjectArray, self.objects_full_topic, 10
        )

        self.get_logger().info(
            f"cctv_to_map started: {self.detections_topic} -> "
            f"{self.objects_topic} (PoseArray) + {self.objects_full_topic} "
            f"(TrackedObjectArray), frame={self.map_frame}"
        )
        self.get_logger().info(f"H_cam_to_map:\n{self.h_cam_to_map}")

    def detections_callback(self, msg):
        stamp = self.get_clock().now().to_msg()
        now = self.get_clock().now().nanoseconds * 1e-9  # 현재 시각 [s]

        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self.map_frame

        obj_array = TrackedObjectArray()
        obj_array.header.stamp = stamp
        obj_array.header.frame_id = self.map_frame

        seen_ids = set()
        for idx, det in enumerate(msg.detections):
            bbox = det.bbox
            # bbox 아래 중앙 = 바닥 접점. u,v 단위 [px]
            u = float(bbox.center.position.x)
            v = float(bbox.center.position.y + bbox.size_y / 2.0)
            map_x, map_y = self._pixel_to_map(u, v)  # [m], map frame

            pose = Pose()
            pose.position.x = map_x
            pose.position.y = map_y
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

            obj = TrackedObject()
            # 추적 ID: 팀원 추적기가 채운 det.id(문자열)를 보존. 숫자면 정수로,
            # 비어있으면(추적기 없음) 루프 인덱스를 임시 ID로 사용.
            track_id = self._resolve_id(det.id, idx)
            obj.id = track_id
            obj.source = self.source_name
            if det.results:
                obj.class_id = det.results[0].hypothesis.class_id
                obj.class_confidence = float(det.results[0].hypothesis.score)
            else:
                obj.class_id = "unknown"
                obj.class_confidence = 0.0
            obj.position = Point(x=map_x, y=map_y, z=0.0)
            # map frame 속도[m/s]: 같은 ID의 이전 위치를 시간 차분해 추정.
            # det.id가 비어 있으면 ID 매칭 불가 → 속도 0 (정상적 degrade).
            vx, vy = self._update_velocity(det.id, track_id, now, map_x, map_y)
            obj.velocity = Vector3(x=vx, y=vy, z=0.0)
            obj.tracking_stability = 1.0
            obj_array.objects.append(obj)
            seen_ids.add(track_id)

            self.get_logger().debug(
                f"id={track_id} pixel=({u:.0f},{v:.0f})px -> "
                f"map=({map_x:.2f},{map_y:.2f})m v=({vx:.2f},{vy:.2f})m/s "
                f"class={obj.class_id}"
            )

        self._prune_tracks(now)
        self.pose_pub.publish(pose_array)
        self.objects_pub.publish(obj_array)

    def _resolve_id(self, det_id, fallback_idx):
        """det.id(문자열 추적 ID)를 int32로 변환. 비었거나 숫자가 아니면 인덱스 사용."""
        if det_id:
            try:
                return int(det_id)
            except ValueError:
                return abs(hash(det_id)) % 100000
        return fallback_idx

    def _update_velocity(self, det_id, track_id, now, x, y):
        """같은 추적 ID의 직전 map 위치를 차분해 속도[m/s]를 EMA로 추정."""
        if not det_id:  # 안정적 ID가 없으면 속도 추정 불가
            return 0.0, 0.0
        prev = self.track_state.get(track_id)
        if prev is None:
            self.track_state[track_id] = (now, x, y, 0.0, 0.0)
            return 0.0, 0.0
        t0, x0, y0, vx0, vy0 = prev
        dt = now - t0
        if dt < self.min_dt:  # dt가 너무 작으면 직전 속도 유지
            return vx0, vy0
        vx = self.vel_alpha * ((x - x0) / dt) + (1 - self.vel_alpha) * vx0
        vy = self.vel_alpha * ((y - y0) / dt) + (1 - self.vel_alpha) * vy0
        self.track_state[track_id] = (now, x, y, vx, vy)
        return vx, vy

    def _prune_tracks(self, now):
        """track_ttl[s]보다 오래 갱신 안 된 추적 ID 상태를 제거."""
        stale = [tid for tid, st in self.track_state.items()
                 if now - st[0] > self.track_ttl]
        for tid in stale:
            del self.track_state[tid]

    def _pixel_to_map(self, u, v):
        """픽셀 (u,v)[px] → map frame (x,y)[m]."""
        pt = np.array([[[u, v]]], dtype=np.float32)
        mapped = cv2.perspectiveTransform(pt, self.h_cam_to_map)
        return float(mapped[0, 0, 0]), float(mapped[0, 0, 1])

    def _pairs(self, values, name):
        if len(values) % 2 != 0 or len(values) < 8:
            raise ValueError(f"{name} must contain at least four x/y pairs")
        return np.array(values, dtype=np.float32).reshape((-1, 2))


def main(args=None):
    rclpy.init(args=args)
    node = CctvToMapNode()
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
