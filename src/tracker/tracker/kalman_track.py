import math
import numpy as np

# Track lifecycle
TENTATIVE = 'tentative'
CONFIRMED = 'confirmed'
DELETED   = 'deleted'

# 튜닝 기본값. config/tracker.yaml로 덮어쓸 수 있으며, yaml이 없을 때 fallback.
# KalmanTrack(params=...)로 인스턴스마다 주입된다.
DEFAULTS = {
    'hits_to_confirm': 3,     # consecutive hits → confirmed
    'max_miss': 5,            # consecutive misses → deleted (콜백 기준 보조 한도)
    # 시간 기반 coast: 입력이 끊겨도(YOLO 미검출/bridge 누락) 이 시간 동안 등속
    # 예측으로 트랙을 유지하고, 넘기면 삭제한다.
    'coast_time': 2.0,        # [s]
    # Process noise: small → trust motion model(smooth), large → trust measurement(jumpy)
    'q_pos': 0.01,
    'q_vel': 0.15,
    # Static classification hysteresis. RiskLayer static/moving_speed와 정렬하면
    # 예측궤적이 뻗는 시점과 costmap 궤적 가중 시점이 일치한다.
    'static_enter': 0.12,     # [m/s] below → static
    'static_exit': 0.25,      # [m/s] above → moving
    'heading_min_speed': 0.15,  # [m/s] 이 이상일 때만 heading 갱신
    'predict_horizon': 3.0,   # [s]
    'predict_steps': 15,      # trajectory points
}

# 하위 호환용 모듈 전역(직접 import하는 코드가 있을 수 있어 유지).
HITS_TO_CONFIRM   = DEFAULTS['hits_to_confirm']
MAX_MISS          = DEFAULTS['max_miss']
COAST_TIME        = DEFAULTS['coast_time']
Q_POS             = DEFAULTS['q_pos']
Q_VEL             = DEFAULTS['q_vel']
STATIC_ENTER      = DEFAULTS['static_enter']
STATIC_EXIT       = DEFAULTS['static_exit']
HEADING_MIN_SPEED = DEFAULTS['heading_min_speed']
PREDICT_HORIZON   = DEFAULTS['predict_horizon']
PREDICT_STEPS     = DEFAULTS['predict_steps']

_next_id = 0


def _new_id() -> int:
    global _next_id
    _next_id += 1
    return _next_id


class KalmanTrack:
    """Single object track with constant-velocity Kalman Filter."""

    def __init__(self, z: np.ndarray, R: np.ndarray, obj_class: str, size,
                 params: dict = None):
        # 튜닝값: 주어진 params로 DEFAULTS를 덮어써 인스턴스에 보관.
        p = dict(DEFAULTS)
        if params:
            p.update({k: v for k, v in params.items() if v is not None})
        self._p = p

        self.track_id   = _new_id()
        self.obj_class  = obj_class
        self.size       = size          # (length, width, height)
        self.state      = TENTATIVE
        self.hits       = 1
        self.misses     = 0
        self.age        = 1
        # 마지막 measurement 갱신 이후 누적 미검출 시간 [s]. predict()마다 dt를
        # 더하고 update()에서 0으로 리셋. 시간 기반 coast/삭제 판정에 사용.
        self.time_since_update = 0.0

        # State: [px, py, vx, vy]
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        # Covariance: position from R, velocity unknown → large
        self.P = np.diag([R[0, 0], R[1, 1], 10.0, 10.0])

        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]], dtype=float)

        self._static = True          # start as static
        self._heading = 0.0          # last stable heading [rad]

    # ------------------------------------------------------------------
    def predict(self, dt: float):
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0,  1, 0],
                      [0, 0,  0, 1]], dtype=float)
        # Process noise scaled by dt
        q_pos = self._p['q_pos']
        q_vel = self._p['q_vel']
        Q = np.diag([q_pos * dt, q_pos * dt, q_vel * dt, q_vel * dt])

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.age += 1
        self.time_since_update += dt

    # ------------------------------------------------------------------
    def update(self, z: np.ndarray, R: np.ndarray):
        S = self.H @ self.P @ self.H.T + R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        innov  = z - self.H @ self.x
        self.x = self.x + K @ innov

        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P

        self.hits  += 1
        self.misses = 0
        self.time_since_update = 0.0

        self._update_motion_state()

        if self.state == TENTATIVE and self.hits >= self._p['hits_to_confirm']:
            self.state = CONFIRMED

    # ------------------------------------------------------------------
    def mark_missed(self):
        self.misses += 1
        # 콜백이 들어오는 정상 상황에서의 보조 한도. 시간 기반 삭제(check_coast)와
        # 둘 중 먼저 걸리는 쪽이 트랙을 지운다. COAST_TIME을 충분히 잡았으므로
        # 보통은 시간 기반 쪽이 먼저 작동한다.
        if self.misses > self._p['max_miss'] and \
                self.time_since_update >= self._p['coast_time']:
            self.state = DELETED

    def check_coast(self):
        """입력 콜백 유무와 무관하게 시간만으로 만료된 트랙을 삭제 표시한다.
        domain bridge 누락 등으로 _objects_cb가 한동안 안 불려도, 타이머가
        predict()로 time_since_update를 키워 결국 이 판정으로 정리된다."""
        if self.time_since_update >= self._p['coast_time']:
            self.state = DELETED

    # ------------------------------------------------------------------
    def _update_motion_state(self):
        spd = self.speed
        # Hysteresis: prevents rapid moving/static toggling
        if self._static and spd > self._p['static_exit']:
            self._static = False
        elif (not self._static) and spd < self._p['static_enter']:
            self._static = True

        # Latch heading only when motion is clearly meaningful
        if spd > self._p['heading_min_speed']:
            self._heading = math.atan2(self.x[3], self.x[2])

    # ------------------------------------------------------------------
    @property
    def position(self) -> np.ndarray:
        return self.x[:2]

    @property
    def velocity(self) -> np.ndarray:
        return self.x[2:4]

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.velocity))

    @property
    def is_static(self) -> bool:
        return self._static

    @property
    def motion_state(self) -> str:
        return 'static' if self._static else 'moving'

    @property
    def heading(self) -> float:
        """Last stable heading [rad]; held while static."""
        return self._heading

    # ------------------------------------------------------------------
    def predicted_trajectory(self) -> list:
        """Future (x, y) over configured prediction horizon."""
        horizon = float(self._p['predict_horizon'])
        steps = int(self._p['predict_steps'])
        dt = horizon / steps
        px, py = self.x[0], self.x[1]
        vx, vy = (0.0, 0.0) if self._static else (self.x[2], self.x[3])
        return [(px + vx * (i * dt), py + vy * (i * dt))
                for i in range(1, steps + 1)]

    # ------------------------------------------------------------------
    def innovation_distance_sq(self, z: np.ndarray, R: np.ndarray) -> float:
        """Mahalanobis² distance to measurement z (position-only)."""
        S = self.H @ self.P @ self.H.T + R
        innov = z - self.H @ self.x
        return float(innov @ np.linalg.inv(S) @ innov)
