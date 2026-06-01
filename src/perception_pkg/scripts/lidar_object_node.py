#!/usr/bin/env python3
"""LiDAR 포인트클라우드에서 동적 객체를 검출/추적하는 self-contained 노드.

racing_ws의 인지 파이프라인(Patchwork++ 지면제거 → DBSCAN 클러스터링 → KF 추적)을
이 워크스페이스 안에서 재현한다. racing_ws에 의존하지 않으므로 GitHub에서 pull 받은
누구나 그대로 실행할 수 있다. (평탄한 실내 바닥 가정 → 지면제거를 z-밴드로 단순화)

흐름:
  /velodyne_points (PointCloud2, velodyne frame)
    1) z-밴드 + 거리 크롭으로 지면/천장/원거리 제거          (Patchwork++ 대체)
    2) 복셀 다운샘플로 점 수 축소
    3) DBSCAN으로 XY 클러스터링                              (racing_ws dbscan 동일값)
    4) 클러스터 크기 필터로 벽 등 큰 구조물 제외
    5) centroid를 TF로 map frame[m]으로 변환
    6) 등속 칼만필터(상태 [x,y,vx,vy]) + 헝가리안 데이터연관으로 ID/속도[m/s] 추정
       → map frame에서 추적하므로 로봇 ego motion에 강건
  -> /lidar/objects (TrackedObjectArray, class="unknown", source="LiDAR")

출력 메시지 형식은 CCTV(cctv_to_map)와 동일하다. 차이는 class가 "unknown"인 것뿐이며,
fusion 노드가 두 소스를 같은 방식으로 융합한다.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from scipy.optimize import linear_sum_assignment
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from sklearn.cluster import DBSCAN
from tf2_ros import Buffer, TransformListener, TransformException

from perception_msgs.msg import TrackedObject, TrackedObjectArray


class KalmanTrack:
    """등속(constant-velocity) 칼만필터 트랙. map frame에서 동작.

    상태 x = [px, py, vx, vy]  (위치 [m], 속도 [m/s])
    측정 z = [px, py]          (클러스터 센트로이드, map frame [m])

    map frame에서 추적하므로 로봇이 회전/이동해도 객체 상태가 흔들리지 않는다
    (ego motion이 process model을 오염시키지 않음).
    """

    def __init__(self, track_id, x, y, q_accel, r_meas):
        self.id = track_id
        self.x = np.array([x, y, 0.0, 0.0], dtype=float)  # 초기 상태
        # 초기 공분산: 위치는 측정만큼, 속도는 크게(아직 모름)
        self.P = np.diag([r_meas, r_meas, 10.0, 10.0])
        self.q_accel = q_accel          # 가속도 잡음 표준편차 [m/s^2]
        self.R = np.eye(2) * r_meas     # 측정 잡음 공분산 [m^2]
        self.hits = 1
        self.missed = 0

    def predict(self, dt):
        """dt[s] 만큼 등속 모델로 상태/공분산 전파."""
        if dt <= 0.0:
            return
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=float)
        # 이산 백색잡음 가속도 모델 Q
        q = self.q_accel ** 2
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        Q = q * np.array([
            [dt4 / 4, 0, dt3 / 2, 0],
            [0, dt4 / 4, 0, dt3 / 2],
            [dt3 / 2, 0, dt2, 0],
            [0, dt3 / 2, 0, dt2],
        ], dtype=float)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, zx, zy):
        """측정 [zx, zy][m]로 보정."""
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        z = np.array([zx, zy], dtype=float)
        y = z - H @ self.x                       # 잔차
        S = H @ self.P @ H.T + self.R            # 잔차 공분산
        K = self.P @ H.T @ np.linalg.inv(S)      # 칼만 이득
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        self.hits += 1
        self.missed = 0

    @property
    def pos(self):
        return self.x[0], self.x[1]      # [m]

    @property
    def vel(self):
        return self.x[2], self.x[3]      # [m/s]


def quat_to_rotation(x, y, z, w):
    """쿼터니언 → 3x3 회전행렬 (외부 의존성 없이 numpy로 직접 계산)."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


class LidarObjectNode(Node):
    def __init__(self):
        super().__init__("lidar_object")

        self.cloud_topic = self.declare_parameter(
            "cloud_topic", "/velodyne_points"
        ).value
        self.objects_topic = self.declare_parameter(
            "objects_topic", "/lidar/objects"
        ).value
        self.map_frame = self.declare_parameter("map_frame", "map").value

        # 1) 지면제거(z-밴드) + 거리 크롭 — 모두 velodyne frame 기준
        # velodyne는 base_link 위 0.6 m에 수평 장착 → 바닥은 z≈-0.6 m.
        self.min_z = float(self.declare_parameter("min_z", -0.4).value)   # 지면 위 [m]
        self.max_z = float(self.declare_parameter("max_z", 1.5).value)    # 천장/키 한계 [m]
        self.max_range = float(self.declare_parameter("max_range", 8.0).value)  # [m]
        self.min_range = float(self.declare_parameter("min_range", 0.3).value)  # 자기차량 제외 [m]

        # 2) 복셀 다운샘플 한 변 [m]
        self.voxel = float(self.declare_parameter("voxel", 0.05).value)

        # 3) DBSCAN (racing_ws: eps=0.40, min_points=5)
        self.eps = float(self.declare_parameter("eps", 0.40).value)            # [m]
        self.min_samples = int(self.declare_parameter("min_samples", 5).value)

        # 4) 클러스터 필터: 사람 크기는 통과, 벽처럼 큰 건 제외
        self.max_object_size = float(self.declare_parameter("max_object_size", 1.2).value)  # [m]
        self.min_cluster_points = int(self.declare_parameter("min_cluster_points", 5).value)

        # 6) 추적 파라미터 (등속 칼만필터 + 헝가리안 데이터연관)
        self.assoc_threshold = float(self.declare_parameter("assoc_threshold", 0.6).value)  # 게이팅 거리 [m]
        self.q_accel = float(self.declare_parameter("process_noise", 1.0).value)  # 가속도 잡음 [m/s^2]
        self.r_meas = float(self.declare_parameter("meas_noise", 0.05).value)     # 측정 잡음 [m^2]
        self.min_hits = int(self.declare_parameter("min_hits", 2).value)   # 발행 최소 관측수(오탐 억제)
        self.max_coast = int(self.declare_parameter("max_coast", 3).value) # 미관측 시 KF로 유지할 프레임

        self.tracks = []        # list[KalmanTrack]
        self.last_t = None      # 직전 프레임 시각 [s]
        self._next_id = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self.cloud_callback, 5
        )
        self.pub = self.create_publisher(TrackedObjectArray, self.objects_topic, 10)

        self.get_logger().info(
            f"lidar_object started: {self.cloud_topic} -> {self.objects_topic} "
            f"(map frame={self.map_frame}, DBSCAN eps={self.eps}m)"
        )

    def cloud_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9  # [s]

        pts = self._read_xyz(msg)                       # velodyne frame [m]
        if pts.shape[0] == 0:
            return

        pts = self._crop(pts)                           # 지면/천장/거리 크롭
        if pts.shape[0] < self.min_samples:
            return

        pts = self._voxel_downsample(pts)
        centroids = self._cluster(pts)                  # velodyne frame centroids [m]
        if centroids.size == 0:
            self._update_tracks([], now)
            self._publish(now)
            return

        map_centroids = self._to_map(centroids, msg)    # map frame [m]
        if map_centroids is None:
            return

        self._update_tracks(map_centroids, now)
        self._publish(now)

    # ── 1) 읽기 ───────────────────────────────────────────────────────────────
    def _read_xyz(self, msg):
        arr = pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        if arr.dtype.names:  # structured array (Humble)
            return np.column_stack((arr["x"], arr["y"], arr["z"])).astype(np.float32)
        return np.asarray(arr, dtype=np.float32).reshape(-1, 3)

    # ── 1) 크롭 (지면제거 대체) ──────────────────────────────────────────────
    def _crop(self, pts):
        z = pts[:, 2]
        r = np.hypot(pts[:, 0], pts[:, 1])
        mask = (
            (z > self.min_z) & (z < self.max_z)
            & (r > self.min_range) & (r < self.max_range)
        )
        return pts[mask]

    # ── 2) 복셀 다운샘플 ──────────────────────────────────────────────────────
    def _voxel_downsample(self, pts):
        keys = np.floor(pts / self.voxel).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return pts[idx]

    # ── 3+4) DBSCAN 클러스터링 + 크기 필터 ───────────────────────────────────
    def _cluster(self, pts):
        xy = pts[:, :2]
        labels = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit_predict(xy)
        centroids = []
        for lbl in set(labels):
            if lbl == -1:  # 노이즈
                continue
            cluster = xy[labels == lbl]
            if cluster.shape[0] < self.min_cluster_points:
                continue
            size = (cluster.max(axis=0) - cluster.min(axis=0)).max()  # XY 최대 변 [m]
            if size > self.max_object_size:  # 벽 등 큰 구조물 제외
                continue
            centroids.append(cluster.mean(axis=0))
        return np.array(centroids, dtype=np.float32) if centroids else np.empty((0, 2))

    # ── 5) map frame 변환 ─────────────────────────────────────────────────────
    def _to_map(self, centroids, msg):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF {msg.header.frame_id}->{self.map_frame} 실패: {ex}",
                                    throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        rot = quat_to_rotation(q.x, q.y, q.z, q.w)
        # centroid는 XY만 있으므로 z=0(velodyne frame 바닥 투영)으로 변환
        pts3 = np.column_stack(
            (centroids[:, 0], centroids[:, 1], np.zeros(len(centroids)))
        )
        mapped = (rot @ pts3.T).T + np.array([t.x, t.y, t.z])
        return mapped[:, :2]

    # ── 6) 칼만필터 추적 + 헝가리안 데이터연관 ───────────────────────────────
    def _update_tracks(self, detections, now):
        dt = 0.0 if self.last_t is None else (now - self.last_t)
        self.last_t = now

        # (1) 예측: 모든 트랙을 현재 시각으로 등속 전파
        for tr in self.tracks:
            tr.predict(dt)

        n_tr, n_det = len(self.tracks), len(detections)

        # (2) 데이터연관: 헝가리안으로 트랙↔검출 전역 최적 1:1 할당
        assigned_det = set()
        if n_tr > 0 and n_det > 0:
            cost = np.zeros((n_tr, n_det))
            for i, tr in enumerate(self.tracks):
                px, py = tr.pos
                for j, (dx, dy) in enumerate(detections):
                    cost[i, j] = np.hypot(px - dx, py - dy)  # 거리 비용 [m]
            rows, cols = linear_sum_assignment(cost)
            for i, j in zip(rows, cols):
                if cost[i, j] <= self.assoc_threshold:   # 게이팅: 임계 밖이면 무시
                    # (3) 보정: 측정으로 KF update
                    self.tracks[i].update(detections[j][0], detections[j][1])
                    assigned_det.add(j)
                else:
                    self.tracks[i].missed += 1
            # 할당 못 받은 트랙(행) 처리
            matched_rows = {i for i, j in zip(rows, cols)
                            if cost[i, j] <= self.assoc_threshold}
            for i, tr in enumerate(self.tracks):
                if i not in matched_rows:
                    tr.missed += 1
        else:
            for tr in self.tracks:
                tr.missed += 1

        # (4) 매칭 안 된 검출 → 새 트랙 생성
        for j, (dx, dy) in enumerate(detections):
            if j not in assigned_det:
                self.tracks.append(
                    KalmanTrack(self._next_id, dx, dy, self.q_accel, self.r_meas))
                self._next_id += 1

        # (5) 너무 오래 미관측인 트랙 제거
        self.tracks = [t for t in self.tracks if t.missed <= self.max_coast]

    def _publish(self, now):
        out = TrackedObjectArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.map_frame
        for tr in self.tracks:
            # 충분히 확인된 트랙만 발행. missed>0이어도 max_coast 이내면 KF 예측으로 유지
            # → 짧은 가려짐에도 객체가 끊기지 않음 (KF의 핵심 이점).
            if tr.hits < self.min_hits:
                continue
            px, py = tr.pos
            vx, vy = tr.vel
            obj = TrackedObject()
            obj.id = tr.id
            obj.source = "LiDAR"
            obj.class_id = "unknown"     # LiDAR 단독은 종류 모름
            obj.class_confidence = 0.0
            obj.position = Point(x=float(px), y=float(py), z=0.0)   # map frame [m]
            obj.velocity = Vector3(x=float(vx), y=float(vy), z=0.0)  # [m/s]
            denom = tr.hits + tr.missed
            obj.tracking_stability = tr.hits / denom if denom > 0 else 1.0
            out.objects.append(obj)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObjectNode()
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
