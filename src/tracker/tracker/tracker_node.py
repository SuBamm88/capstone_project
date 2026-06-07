#!/usr/bin/env python3
"""CCTV 객체를 Kalman + Hungarian으로 추적하고 예측 궤적을 채우는 노드.

데이터 흐름 (capstone 규약):
    perception_pkg/cctv_to_map_node
        -> /cctv/objects  (perception_msgs/TrackedObjectArray, map frame)   [입력]
    tracker_node (이 노드)
        -> /cctv/tracks   (perception_msgs/TrackedTrackArray, 안정 id/속도/예측)  [출력]
        -> /tracker/markers (visualization_msgs/MarkerArray, RViz 디버그)

cctv_to_map은 프레임마다 단순 차분으로 속도를 추정하므로 id가 불안정하고
예측 궤적이 없다. 이 노드는 등속(constant-velocity) Kalman 필터로 객체별
안정 트랙을 유지하고, predicted_trajectory 필드를 채워 RiskLayer가 미래
위험 영역까지 inflation할 수 있게 한다.

입력 메시지의 id는 cctv_to_map의 임시 id이므로 신뢰하지 않고, 위치만
measurement로 사용해 자체 트랙 id를 새로 부여한다. class_id/source는 매칭된
검출에서 그대로 가져온다.

추가로 비전 pose 기반 응시방향(facing)을 융합한다:
    person_direction_node -> /cctv_pose/person_direction_info (JSON)
        각 사람의 map 위치(bottom_center_map)와 바라보는 방향(direction)
이 토픽의 id(별도 pose 추적기의 ByteTrack id)는 본 노드의 트랙 id와 다르므로
직접 매칭하지 않고, map 위치 최근접으로 트랙에 facing yaw를 부착한다. 이렇게
하면 정지한 사람도 "바라보는 방향"을 갖게 되어 RiskLayer가 그 방향으로
비대칭(물방울) 비용을 칠할 수 있다. heading(이동방향)과는 별개 필드다.
"""

import json
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from perception_msgs.msg import TrackedObjectArray, TrackedTrack, TrackedTrackArray

from .kalman_track import (
    KalmanTrack, CONFIRMED, DELETED, DEFAULTS as KT_DEFAULTS)
from .hungarian import associate, GATE_MAHAL_SQ
from .uncertainty import compute_R


# 모션 상태별 마커 색 (r, g, b, a)
COLOR = {
    'moving': (0.2, 0.9, 0.2, 0.8),   # green
    'static': (0.6, 0.6, 0.6, 0.6),   # grey
}


class TrackerNode(Node):

    def __init__(self):
        super().__init__('tracker_node')

        # 입력: cctv_to_map이 내보내는 TrackedObjectArray (map frame, 단위 m)
        self.input_topic = self.declare_parameter(
            'input_topic', '/cctv/objects').value
        # 출력1: 안정 트랙 (RiskLayer / fusion이 사용)
        self.tracks_topic = self.declare_parameter(
            'tracks_topic', '/cctv/tracks').value
        # 출력2: RViz 디버그 마커
        self.markers_topic = self.declare_parameter(
            'markers_topic', '/tracker/markers').value
        # 입력3: 비전 pose 응시방향 (person_direction_node, JSON String)
        self.direction_topic = self.declare_parameter(
            'direction_topic', '/cctv_pose/person_direction_info').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        # facing 매칭: 응시방향의 map 위치와 트랙 위치가 이 거리[m] 안이면 부착.
        self.facing_match_dist = float(
            self.declare_parameter('facing_match_dist', 1.0).value)
        # facing 신선도[s]. 이보다 오래된 응시방향은 무시(has_facing=False).
        self.facing_max_age = float(
            self.declare_parameter('facing_max_age', 0.5).value)
        # coast 타이머 주기 [s]. 입력 콜백이 끊겨도 이 주기로 predict + publish
        # 하여 트랙을 등속 예측으로 유지한다(깜빡임 방지). 0이면 비활성.
        self.coast_period = float(
            self.declare_parameter('coast_period', 0.1).value)
        self.dt_min = float(
            self.declare_parameter('dt_min', 1.0e-3).value)
        self.dt_max = float(
            self.declare_parameter('dt_max', 0.5).value)
        self.time_jump_reset_threshold = float(
            self.declare_parameter('time_jump_reset_threshold', 0.1).value)

        # ── KalmanTrack/associate 튜닝값 (config/tracker.yaml) ──
        # 각 트랙 생성 시 주입할 dict. 미지정 항목은 kalman_track.DEFAULTS 사용.
        self.track_params = {
            'hits_to_confirm':
                int(self.declare_parameter(
                    'hits_to_confirm', KT_DEFAULTS['hits_to_confirm']).value),
            'max_miss':
                int(self.declare_parameter(
                    'max_miss', KT_DEFAULTS['max_miss']).value),
            'coast_time':
                float(self.declare_parameter(
                    'coast_time', KT_DEFAULTS['coast_time']).value),
            'q_pos':
                float(self.declare_parameter(
                    'q_pos', KT_DEFAULTS['q_pos']).value),
            'q_vel':
                float(self.declare_parameter(
                    'q_vel', KT_DEFAULTS['q_vel']).value),
            'static_enter':
                float(self.declare_parameter(
                    'static_enter', KT_DEFAULTS['static_enter']).value),
            'static_exit':
                float(self.declare_parameter(
                    'static_exit', KT_DEFAULTS['static_exit']).value),
            'heading_min_speed':
                float(self.declare_parameter(
                    'heading_min_speed', KT_DEFAULTS['heading_min_speed']).value),
            'predict_horizon':
                float(self.declare_parameter(
                    'predict_horizon', KT_DEFAULTS['predict_horizon']).value),
            'predict_steps':
                int(self.declare_parameter(
                    'predict_steps', KT_DEFAULTS['predict_steps']).value),
        }
        # 연관 게이트(Mahalanobis²): 빠른 보행자 fragmentation의 주 조절값.
        self.gate_mahal_sq = float(
            self.declare_parameter('gate_mahal_sq', GATE_MAHAL_SQ).value)

        self._sub = self.create_subscription(
            TrackedObjectArray, self.input_topic, self._objects_cb, 10)
        self._direction_sub = self.create_subscription(
            String, self.direction_topic, self._direction_cb, 10)
        self._tracks_pub = self.create_publisher(
            TrackedTrackArray, self.tracks_topic, 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, self.markers_topic, 10)

        # 최근 응시방향 목록: [(map_x, map_y, yaw)], 수신 벽시계 시각과 함께 보관
        self._facings: list[tuple[float, float, float]] = []
        self._facings_wall = None

        self._tracks: list[KalmanTrack] = []
        self._last_predict_stamp = None  # 마지막 Kalman predict 기준 시각 [s]
        self._last_input_time = None  # 마지막 입력을 받은 ROS 시각 [s]
        self._last_header = None      # coast publish에 재사용할 헤더

        if self.coast_period > 0.0:
            self._coast_timer = self.create_timer(
                self.coast_period, self._coast_cb)

        self.get_logger().info(
            f'tracker_node started: {self.input_topic} -> '
            f'{self.tracks_topic} (+ {self.markers_topic}), '
            f'coast_period={self.coast_period}s')

    # ------------------------------------------------------------------
    def _predict_tracks_to(self, stamp: float):
        if self._last_predict_stamp is None:
            self._last_predict_stamp = stamp
            return

        raw_dt = stamp - self._last_predict_stamp
        if raw_dt < -self.time_jump_reset_threshold:
            self.get_logger().warn(
                f'input time jumped backward by {-raw_dt:.3f}s; '
                'resetting active tracks')
            self._tracks.clear()
            self._last_predict_stamp = stamp
            return

        dt = min(max(raw_dt, self.dt_min), self.dt_max)
        for trk in self._tracks:
            trk.predict(dt)
        self._last_predict_stamp = stamp

    # ------------------------------------------------------------------
    def _objects_cb(self, msg: TrackedObjectArray):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self._predict_tracks_to(now)
        self._last_input_time = self.get_clock().now().nanoseconds * 1e-9
        # coast publish가 재사용할 수 있도록 가장 최근 헤더를 보관
        self._last_header = msg.header

        # 1. 입력 객체 -> measurement 리스트
        detections = []
        for obj in msg.objects:
            z = np.array([obj.position.x, obj.position.y])
            R = compute_R(obj.class_confidence)
            detections.append({
                'z': z, 'R': R,
                'class': obj.class_id or 'unknown',
                'confidence': float(obj.class_confidence),
                'source': obj.source or 'CCTV',
                # footprint_area로부터 박스 추정 (없으면 사람 기본값)
                'size': self._infer_size(obj.footprint_area),
            })

        # 2. 연관 (Mahalanobis gate + Hungarian)
        active = [t for t in self._tracks if t.state != DELETED]
        matched, unmatched_trk, unmatched_det = associate(
            active, detections, gate=self.gate_mahal_sq)

        # 3. 매칭된 트랙 갱신
        for ti, di in matched:
            det = detections[di]
            active[ti].update(det['z'], det['R'])
            # 분류/출처는 최신 검출 값으로 갱신
            active[ti].obj_class = det['class']
            active[ti].confidence = det['confidence']
            active[ti].source = det['source']

        # 4. 미검출 트랙: miss 카운트 증가 + 시간 기반 coast 만료 판정
        for ti in unmatched_trk:
            active[ti].mark_missed()
            active[ti].check_coast()

        # 5. 미매칭 검출 -> 새 트랙
        for di in unmatched_det:
            det = detections[di]
            trk = KalmanTrack(det['z'], det['R'], det['class'], det['size'],
                              params=self.track_params)
            trk.confidence = det['confidence']
            trk.source = det['source']
            self._tracks.append(trk)

        # 6. 삭제 트랙 정리
        self._tracks = [t for t in self._tracks if t.state != DELETED]

        # 7. publish (confirmed만)
        self._publish_tracks(msg.header)
        self._publish_markers(msg.header)

    # ------------------------------------------------------------------
    def _coast_cb(self):
        """입력 콜백과 독립적으로 도는 타이머. 최근에 입력이 들어왔으면
        아무것도 하지 않는다(_objects_cb가 이미 처리). 입력이 coast_period보다
        오래 끊겼을 때만 등속 예측으로 트랙을 전진시키고 다시 publish하여,
        domain bridge 누락 등으로 cost가 깜빡이는 것을 막는다."""
        if not self._tracks or self._last_input_time is None:
            return

        wall = self.get_clock().now().nanoseconds * 1e-9
        gap = wall - self._last_input_time
        # 입력이 정상적으로 들어오는 동안에는 _objects_cb에 맡기고 빠진다.
        if gap < self.coast_period:
            return

        self._predict_tracks_to(wall)
        # measurement 없이 예측만 전진 → time_since_update 증가 → 만료 시 삭제
        for trk in self._tracks:
            trk.check_coast()
        self._tracks = [t for t in self._tracks if t.state != DELETED]

        if self._last_header is None:
            return
        header = self._last_header
        header.stamp = self.get_clock().now().to_msg()
        self._publish_tracks(header)
        self._publish_markers(header)

    # ------------------------------------------------------------------
    def _direction_cb(self, msg: String):
        """person_direction_node의 응시방향 JSON을 파싱해 (x, y, yaw) 목록으로
        보관한다. id 매칭은 하지 않고, publish 시점에 트랙 위치 최근접으로 붙인다."""
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(payload, list):
            return

        facings = []
        for item in payload:
            if not isinstance(item, dict):
                continue
            pos = item.get('bottom_center_map') or {}
            d = item.get('direction') or {}
            try:
                x = float(pos['x'])
                y = float(pos['y'])
                dx = float(d['x'])
                dy = float(d['y'])
            except (KeyError, TypeError, ValueError):
                continue
            if math.hypot(dx, dy) < 1e-6:
                continue
            facings.append((x, y, math.atan2(dy, dx)))

        self._facings = facings
        self._facings_wall = self.get_clock().now().nanoseconds * 1e-9

    def _lookup_facing(self, px, py):
        """트랙 위치 (px, py)에 가장 가까운 응시방향 yaw를 반환. 신선도/거리
        게이트를 통과하지 못하면 None."""
        if not self._facings or self._facings_wall is None:
            return None
        wall = self.get_clock().now().nanoseconds * 1e-9
        if wall - self._facings_wall > self.facing_max_age:
            return None

        best_yaw = None
        best_d2 = self.facing_match_dist ** 2
        for fx, fy, yaw in self._facings:
            d2 = (fx - px) ** 2 + (fy - py) ** 2
            if d2 <= best_d2:
                best_d2 = d2
                best_yaw = yaw
        return best_yaw

    # ------------------------------------------------------------------
    def _infer_size(self, footprint_area: float):
        """footprint_area[m^2]로 박스(l, w, h) 추정. 0이면 사람 기본값."""
        if footprint_area and footprint_area > 0.0:
            side = math.sqrt(footprint_area)
            return (side, side, 1.7)
        return (0.5, 0.5, 1.7)

    # ------------------------------------------------------------------
    def _publish_tracks(self, header):
        out = TrackedTrackArray()
        out.header = header
        if not out.header.frame_id:
            out.header.frame_id = self.map_frame

        for trk in self._tracks:
            if trk.state != CONFIRMED:
                continue
            px, py = trk.position
            # velocity는 정지여도 실제 추정값을 그대로 보낸다. RiskLayer가 이
            # 속도로 물방울↔예측궤적을 연속 블렌딩하기 때문(정지면 자연히 궤적이
            # 꺼지고 물방울이 산다). 예측궤적 폭주 방지는 is_static일 때 궤적을
            # 한 점에 묶는 predicted_trajectory()가 이미 담당하므로 여기서 속도를
            # 0으로 박지 않는다.
            vx, vy = float(trk.velocity[0]), float(trk.velocity[1])
            l, w, _h = trk.size

            obj = TrackedTrack()
            obj.id = int(trk.track_id)
            obj.class_id = getattr(trk, 'obj_class', 'unknown')
            obj.class_confidence = float(getattr(trk, 'confidence', 0.0))
            obj.source = getattr(trk, 'source', 'CCTV')
            obj.position = Point(x=float(px), y=float(py), z=0.0)
            obj.velocity = Vector3(x=vx, y=vy, z=0.0)
            obj.heading = float(trk.heading)
            # 응시방향(facing): 비전 pose 기반. 트랙 위치 최근접 매칭으로 부착.
            # 없으면 has_facing=False → RiskLayer는 기존 원형 코어 사용.
            facing_yaw = self._lookup_facing(px, py)
            if facing_yaw is not None:
                obj.facing = float(facing_yaw)
                obj.has_facing = True
            else:
                obj.facing = 0.0
                obj.has_facing = False
            obj.footprint_area = float(l * w)
            # 안정도: hits / (hits + misses)
            denom = trk.hits + trk.misses
            obj.tracking_stability = float(trk.hits / denom) if denom else 1.0
            obj.predicted_trajectory = [
                Point(x=float(x), y=float(y), z=0.0)
                for x, y in trk.predicted_trajectory()
            ]
            out.tracks.append(obj)

        self._tracks_pub.publish(out)

    # ------------------------------------------------------------------
    def _publish_markers(self, header):
        markers = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        mid = 0
        confirmed = [t for t in self._tracks if t.state == CONFIRMED]

        for trk in confirmed:
            r, g, b, a = COLOR[trk.motion_state]
            px, py = trk.position
            vx, vy = trk.velocity
            l, w, h = trk.size

            box = Marker()
            box.header = header
            box.ns, box.id = 'box', mid
            mid += 1
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(px)
            box.pose.position.y = float(py)
            box.pose.position.z = float(h) / 2.0
            yaw = trk.heading
            box.pose.orientation.z = math.sin(yaw / 2)
            box.pose.orientation.w = math.cos(yaw / 2)
            box.scale.x = float(l)
            box.scale.y = float(w)
            box.scale.z = float(h)
            box.color.r, box.color.g, box.color.b, box.color.a = r, g, b, a
            markers.markers.append(box)

            txt = Marker()
            txt.header = header
            txt.ns, txt.id = 'text', mid
            mid += 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(px)
            txt.pose.position.y = float(py)
            txt.pose.position.z = float(h) + 0.3
            txt.scale.z = 0.25
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f'ID:{trk.track_id} {trk.speed:.2f}m/s [{trk.motion_state}]'
            markers.markers.append(txt)

            if not trk.is_static:
                arr = Marker()
                arr.header = header
                arr.ns, arr.id = 'arrow', mid
                mid += 1
                arr.type = Marker.ARROW
                arr.action = Marker.ADD
                arr.scale.x = 0.05
                arr.scale.y = 0.10
                arr.scale.z = 0.10
                arr.color.r, arr.color.g, arr.color.b, arr.color.a = \
                    1.0, 1.0, 0.0, 0.9
                start = Point(x=float(px), y=float(py), z=float(h) / 2.0)
                end = Point(x=float(px + vx), y=float(py + vy), z=float(h) / 2.0)
                arr.points = [start, end]
                markers.markers.append(arr)

            traj = Marker()
            traj.header = header
            traj.ns, traj.id = 'traj', mid
            mid += 1
            traj.type = Marker.LINE_STRIP
            traj.action = Marker.ADD
            traj.scale.x = 0.04
            traj.color.r, traj.color.g, traj.color.b, traj.color.a = \
                0.3, 0.8, 1.0, 0.7
            traj.points = [
                Point(x=float(x), y=float(y), z=0.0)
                for x, y in trk.predicted_trajectory()
            ]
            markers.markers.append(traj)

        self._marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
