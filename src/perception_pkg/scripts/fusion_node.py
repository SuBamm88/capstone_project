#!/usr/bin/env python3
"""CCTV 객체와 LiDAR 추적 객체를 하나의 map 레이어로 융합하는 노드.

입력 (CCTV와 LiDAR 모두 동일한 TrackedObjectArray 형식)
  - cctv_to_map의 /cctv/objects   (class 있음, map 위치[m] + 속도[m/s]), 카메라당 1토픽
  - lidar_object의 /lidar/objects (class="unknown", map 위치[m] + 속도[m/s])

매칭(헝가리안 전역 최적 1:1 할당 + 거리 게이팅):
  두 점의 map frame 거리[m] < match_threshold[m] 이면 동일 객체로 본다.
    매칭됨    -> FUSED  : 위치/속도는 LiDAR, class는 CCTV (LiDAR 위치가 더 정확)
    LiDAR만   -> LiDAR  : class "unknown"
    CCTV만    -> CCTV   : LiDAR 사각지대. cctv_to_map이 준 속도 사용

출력
  - /perception/tracked_objects (TrackedObjectArray) 통합 레이어
  - /cctv/objects_map (PoseArray) Nav2 CctvLayer가 계속 장애물을 받도록 재발행
  - /perception/markers (MarkerArray) RViz / Foxglove 시각화

이 노드는 외부 워크스페이스(racing_ws 등)에 의존하지 않는다. LiDAR 입력도
lidar_object 노드가 내는 TrackedObjectArray라서, 이 워크스페이스만으로 완결된다.

ID 정책: 카메라와 LiDAR가 같은 객체를 보면 ID는 서로 다르다(소스가 다르므로 당연).
FUSED는 LiDAR ID를, CCTV 전용은 CCTV ID + 오프셋(충돌 방지)을 그대로 유지해
프레임이 지나도 같은 객체로 보이게 한다.
"""

import math

import numpy as np
import rclpy
from scipy.optimize import linear_sum_assignment
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseArray
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from perception_msgs.msg import TrackedObjectArray

# 소스별 마커 색 (R, G, B) [0~1]
SOURCE_COLORS = {
    "FUSED": (0.1, 0.9, 0.1),  # 초록
    "LiDAR": (0.1, 0.5, 1.0),  # 파랑
    "CCTV": (1.0, 0.55, 0.0),  # 주황
}

# CCTV 전용 객체 ID 오프셋 (LiDAR track_id와 충돌 방지)
CCTV_ID_OFFSET = 100000


class FusionNode(Node):
    def __init__(self):
        super().__init__("fusion")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        # 카메라별 cctv_to_map 출력 토픽 목록
        cctv_topics = self.declare_parameter("cctv_topics", ["/cctv/objects"]).value
        # lidar_object 노드의 map frame 출력 토픽 (TrackedObjectArray)
        self.lidar_topic = self.declare_parameter("lidar_topic", "/lidar/objects").value
        # 동일 객체 판정 거리 임계 [m]. (정확도 측정 기반 튜닝, 보통 0.5~1.0)
        self.match_threshold = float(
            self.declare_parameter("match_threshold", 0.6).value
        )
        self.cctv_timeout = float(self.declare_parameter("cctv_timeout", 1.0).value)   # [s]
        self.lidar_timeout = float(self.declare_parameter("lidar_timeout", 0.5).value) # [s]
        self.pred_horizon = float(self.declare_parameter("pred_horizon", 2.0).value)   # [s]
        self.pred_dt = float(self.declare_parameter("pred_dt", 0.5).value)             # [s]
        self.rate = float(self.declare_parameter("publish_rate", 10.0).value)          # [Hz]

        self.cctv_by_topic = {}     # topic -> list[(stamp[s], TrackedObject)]
        self.lidar_objects = []     # list[(stamp[s], TrackedObject)]

        for topic in cctv_topics:
            self.cctv_by_topic[topic] = []
            self.create_subscription(
                TrackedObjectArray, topic,
                self._make_cctv_callback(topic), 10
            )

        self.create_subscription(
            TrackedObjectArray, self.lidar_topic, self._lidar_callback, 10
        )
        self.get_logger().info(f"LiDAR objects: subscribed to {self.lidar_topic}")

        self.objects_pub = self.create_publisher(
            TrackedObjectArray, "/perception/tracked_objects", 10
        )
        self.pose_pub = self.create_publisher(PoseArray, "/cctv/objects_map", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/perception/markers", 10)
        self.timer = self.create_timer(1.0 / max(self.rate, 1.0), self._fuse_and_publish)

        self.get_logger().info(
            f"fusion started: cctv={cctv_topics}, match_threshold="
            f"{self.match_threshold} m"
        )

    # ── 콜백 ────────────────────────────────────────────────────────────────
    def _make_cctv_callback(self, topic):
        # 토픽별 클로저: 카메라가 여러 대여도 서로 덮어쓰지 않게 토픽 단위로 저장
        def cb(msg):
            now = self._now()
            self.cctv_by_topic[topic] = [(now, obj) for obj in msg.objects]
        return cb

    def _lidar_callback(self, msg):
        # LiDAR도 CCTV와 동일한 TrackedObjectArray 형식 → 그대로 보관
        now = self._now()
        self.lidar_objects = [(now, obj) for obj in msg.objects]

    # ── 융합 ────────────────────────────────────────────────────────────────
    def _fuse_and_publish(self):
        now = self._now()
        # 타임아웃[s] 지난 관측은 버린다(오래된 유령 객체 방지)
        cctv = [
            o
            for objs in self.cctv_by_topic.values()
            for (t, o) in objs
            if now - t <= self.cctv_timeout
        ]
        lidar = [o for (t, o) in self.lidar_objects if now - t <= self.lidar_timeout]
        cctv = self._merge_cctv(cctv)  # 여러 카메라가 본 같은 객체 먼저 병합

        matched_cctv = set()
        fused = []

        # LiDAR↔CCTV 데이터연관: 헝가리안으로 전역 최적 1:1 할당 (그리디보다 안정적).
        # LiDAR가 기하(위치/속도)를 담당하고, 매칭된 CCTV의 class를 가져온다.
        pairs = self._associate(lidar, cctv)  # {lidar_idx: cctv_idx}
        for i, l in enumerate(lidar):
            obj = l
            j = pairs.get(i)
            if j is not None:
                matched_cctv.add(j)
                obj.source = "FUSED"
                obj.class_id = cctv[j].class_id
                obj.class_confidence = cctv[j].class_confidence
            fused.append(obj)

        # 매칭 안 된 CCTV 객체(LiDAR 사각지대) → ID 충돌 방지 오프셋 적용
        for j, c in enumerate(cctv):
            if j not in matched_cctv:
                c.id = c.id + CCTV_ID_OFFSET
                fused.append(c)

        # 예측 경로[m]는 속도 기반으로 계산
        for obj in fused:
            obj.predicted_trajectory = self._predict(obj)

        self._publish(fused)

    def _associate(self, lidar, cctv):
        """LiDAR↔CCTV 헝가리안 매칭. 반환: {lidar_idx: cctv_idx} (게이팅 통과분만).

        그리디는 한 검출을 먼저 차지하면 다른 트랙이 차선책으로 밀리는 문제가 있다.
        헝가리안은 전체 거리 합을 최소화하는 1:1 할당을 한 번에 구해 더 안정적이다.
        """
        if not lidar or not cctv:
            return {}
        cost = np.zeros((len(lidar), len(cctv)))
        for i, l in enumerate(lidar):
            for j, c in enumerate(cctv):
                cost[i, j] = math.hypot(
                    l.position.x - c.position.x, l.position.y - c.position.y)  # [m]
        rows, cols = linear_sum_assignment(cost)
        pairs = {}
        for i, j in zip(rows, cols):
            if cost[i, j] <= self.match_threshold:  # 게이팅: 임계 밖이면 동일 객체 아님
                pairs[i] = j
        return pairs

    def _merge_cctv(self, cctv):
        """여러 카메라가 본 같은 객체(거리 < match_threshold[m])를 하나로 합치고,
        더 높은 confidence의 class를 채택한다."""
        merged = []
        for c in cctv:
            hit = None
            for m in merged:
                if math.hypot(c.position.x - m.position.x,
                              c.position.y - m.position.y) < self.match_threshold:
                    hit = m
                    break
            if hit is None:
                merged.append(c)
            elif c.class_confidence > hit.class_confidence:
                hit.class_id = c.class_id
                hit.class_confidence = c.class_confidence
        return merged

    def _predict(self, obj):
        """등속 가정으로 pred_horizon[s] 동안 pred_dt[s] 간격의 미래 위치[m]를 생성.
        속도 0.1 m/s 미만(정지)은 예측 없음."""
        v = math.hypot(obj.velocity.x, obj.velocity.y)  # 속력 [m/s]
        if v < 0.1:
            return []
        traj = []
        steps = int(self.pred_horizon / self.pred_dt)
        for k in range(1, steps + 1):
            traj.append(Point(
                x=obj.position.x + obj.velocity.x * self.pred_dt * k,
                y=obj.position.y + obj.velocity.y * self.pred_dt * k,
                z=0.0,
            ))
        return traj

    # ── 출력 ────────────────────────────────────────────────────────────────
    def _publish(self, fused):
        stamp = self.get_clock().now().to_msg()

        obj_array = TrackedObjectArray()
        obj_array.header.stamp = stamp
        obj_array.header.frame_id = self.map_frame
        obj_array.objects = fused
        self.objects_pub.publish(obj_array)

        # Nav2 CctvLayer 호환용 PoseArray 재발행 (map frame [m])
        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self.map_frame
        for obj in fused:
            pose = Pose()
            pose.position = obj.position
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        self.pose_pub.publish(pose_array)

        self.marker_pub.publish(self._markers(fused, stamp))

    def _markers(self, fused, stamp):
        array = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL  # 매 프레임 갱신 위해 이전 마커 제거
        array.markers.append(delete)

        mid = 0
        for obj in fused:
            color = SOURCE_COLORS.get(obj.source, (0.6, 0.6, 0.6))

            # 객체 본체: 지름 0.4 m, 높이 0.2 m 원기둥
            body = Marker()
            body.header.frame_id = self.map_frame
            body.header.stamp = stamp
            body.ns = "objects"
            body.id = mid
            mid += 1
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position = Point(x=obj.position.x, y=obj.position.y, z=0.1)
            body.pose.orientation.w = 1.0
            body.scale.x = body.scale.y = 0.4  # [m]
            body.scale.z = 0.2                 # [m]
            body.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.85)
            array.markers.append(body)

            # 라벨: #id class [source] 속력[m/s]
            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = stamp
            label.ns = "labels"
            label.id = mid
            mid += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(x=obj.position.x, y=obj.position.y, z=0.6)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.25  # 글자 높이 [m]
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            speed = math.hypot(obj.velocity.x, obj.velocity.y)  # [m/s]
            label.text = f"#{obj.id} {obj.class_id} [{obj.source}] {speed:.1f}m/s"
            array.markers.append(label)

            # 예측 경로: 선 굵기 0.05 m
            if obj.predicted_trajectory:
                path = Marker()
                path.header.frame_id = self.map_frame
                path.header.stamp = stamp
                path.ns = "prediction"
                path.id = mid
                mid += 1
                path.type = Marker.LINE_STRIP
                path.action = Marker.ADD
                path.pose.orientation.w = 1.0
                path.scale.x = 0.05  # [m]
                path.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.9)
                path.points = [Point(x=obj.position.x, y=obj.position.y, z=0.1)]
                path.points.extend(obj.predicted_trajectory)
                array.markers.append(path)
        return array

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9  # [s]


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
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
