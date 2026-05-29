# cctv_costmap_layer

CCTV가 감지한 객체 위치를 Nav2 costmap에 장애물로 반영하는 custom costmap layer 플러그인이다.

## 개요

```text
LiDAR 장애물 + CCTV 객체 위치
-> Nav2 costmap (cctv_layer + inflation_layer)
-> global/local 경로 계획
-> Scout Mini 주행
```

CCTV가 감지한 객체 위치를 `/cctv/objects_map` 토픽의 `geometry_msgs/msg/PoseArray`로 수신하고, 각 위치를 lethal obstacle로 표시한다. inflation layer가 obstacle 주변 safety margin을 생성한다.

## 역할

- `/cctv/objects_map` (`geometry_msgs/msg/PoseArray`) 구독
- 각 pose를 costmap frame으로 TF 변환
- `default_radius` 범위 내 cell을 `LETHAL_OBSTACLE`로 설정
- `updateWithMax`로 master costmap에 병합
- global costmap, local costmap 모두 적용 가능

## 패키지 구조

```text
cctv_costmap_layer/
├── include/cctv_costmap_layer/cctv_layer.hpp
├── src/cctv_layer.cpp
├── cctv_costmap_plugins.xml
├── CMakeLists.txt
└── package.xml
```

## 파라미터

```yaml
cctv_layer:
  plugin: "cctv_costmap_layer::CctvLayer"
  enabled: True
  topic: /cctv/objects_map
  default_radius: 0.25
  max_observation_age: 1.0
  transform_tolerance: 0.2
```

| 파라미터 | 설명 |
|----------|------|
| `topic` | CCTV 객체 위치를 수신할 토픽 |
| `default_radius` | pose당 장애물 반경 (미터) |
| `max_observation_age` | 오래된 PoseArray를 버리는 시간 (초) |
| `transform_tolerance` | TF 변환 대기 허용 시간 (초) |

## Nav2 Costmap 설정

`src/bringup_pkg/config/nav2_params_cctv.yaml`

**Global costmap:**
```yaml
plugins: ["static_layer", "obstacle_layer", "cctv_layer", "inflation_layer"]
```

**Local costmap:**
```yaml
plugins: ["voxel_layer", "cctv_layer", "inflation_layer"]
```

## 입출력

| | 토픽 | 타입 |
|--|------|------|
| 입력 | `/cctv/objects_map` | `geometry_msgs/msg/PoseArray` |
| 시각화 출력 | `/cctv/objects_marker` | `visualization_msgs/msg/MarkerArray` |

**PoseArray 형식:**
```yaml
header:
  frame_id: map
poses:
- position: {x: 1.0, y: 2.0, z: 0.0}
  orientation: {w: 1.0}
```

- `position.x/y`: map frame 기준 객체 위치
- `position.z`, `orientation`: 사용하지 않음

## 빌드

워크스페이스 루트에서 실행:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cctv_costmap_layer cctv_object_tools bringup_pkg
source install/setup.bash
```

빌드 확인:
```bash
ros2 pkg prefix cctv_costmap_layer
ros2 pkg executables cctv_object_tools
```

## 실행

### 사전 조건

- Scout Mini CAN 연결
- Velodyne VLP-32C 연결
- 맵 생성 완료
- TF tree: `map -> odom -> base_link -> velodyne`

### 실행 순서

**터미널 1 — Scout base**
```bash
sudo ip link set can0 up type can bitrate 500000
ros2 launch scout_base scout_mini_base.launch.py
```

**터미널 2 — Robot description**
```bash
ros2 launch scout_description scout_base_description.launch.py
```

**터미널 3 — Velodyne 정적 TF**
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0.6 0 0 0 base_link velodyne
```

**터미널 4 — Velodyne 드라이버**
```bash
ros2 launch velodyne velodyne-all-nodes-VLP32C-composed-launch.py
```

**터미널 5 — CCTV layer 포함 Nav2**
```bash
ros2 launch bringup_pkg scout_cctv_nav2.launch.py map:=src/bringup_pkg/maps/scout_mini_map_3.yaml
```

**터미널 6 — CCTV 객체 시뮬레이터**
```bash
ros2 launch cctv_object_tools rviz_click_to_pose_array.launch.py
```

**터미널 7 — RViz2**
```bash
rviz2 -d src/bringup_pkg/rviz/rviz.rviz
```

## 객체 관리

```bash
# 모든 CCTV 장애물 제거
ros2 service call /cctv/clear_objects std_srvs/srv/Empty

# 마지막 장애물 제거
ros2 service call /cctv/remove_last_object std_srvs/srv/Empty
```

## 실제 CCTV 파이프라인 연결

perception 파이프라인이 완성되면 `cctv_object_tools` 대신 아래 조건으로 퍼블리시하면 된다:

```text
토픽:    /cctv/objects_map
타입:    geometry_msgs/msg/PoseArray
frame:   map
주기:    ~5 Hz
```

- 각 객체의 map frame 위치를 `poses[].position.x/y`에 담는다
- 사라진 객체는 다음 PoseArray에서 제외한다
- 빈 PoseArray를 publish하면 모든 CCTV 장애물이 제거된다

## 튜닝

| 파일 | 파라미터 | 효과 |
|------|----------|------|
| `nav2_params_cctv.yaml` | `cctv_layer.default_radius` | 객체당 장애물 크기 |
| `nav2_params_cctv.yaml` | `global_costmap.inflation_layer.inflation_radius` | global 경로 회피 여유 |
| `nav2_params_cctv.yaml` | `local_costmap.inflation_layer.inflation_radius` | local 근거리 회피 여유 |
| `nav2_params_cctv.yaml` | `cctv_layer.max_observation_age` | 오래된 객체 제거 시간 |

## 문제 해결

**`/cctv/objects_map` 토픽이 없을 때**
- `cctv_object_tools` 노드 실행 여부 확인
- RViz `Publish Point` 도구 선택 및 Fixed Frame이 `map`인지 확인

**costmap에 장애물이 안 보일 때**
- `nav2_params_cctv.yaml`의 costmap plugins 목록에 `cctv_layer` 포함 여부 확인
- `ros2 topic echo /cctv/objects_map --once` 확인

**TF 경고 발생 시**
- RViz `2D Pose Estimate`로 초기 위치 지정 여부 확인
- `PoseArray.header.frame_id`가 `map`인지 확인

**경로가 바뀌지 않을 때**
- global path 위 또는 근처를 클릭
- `nav2_params_cctv.yaml`에서 `inflation_radius` 증가
- Nav2 goal을 다시 지정해 재경로 계획 유도

## 현재 한계

- 객체 class 구분 없이 동일 크기 장애물로 처리
- confidence, velocity 미반영
- PoseArray에 객체별 radius 담기 불가
- 다중 CCTV 객체 fusion 미구현

## 향후 확장 방향

1. custom 메시지 도입 (`object_id`, `class`, `confidence`, `velocity`, `radius`)
2. class별 cost 차등 적용 (사람 / 박스 / 지게차)
3. 동적 객체 risk (velocity 기반 heading cost, 예측 위치)
4. 다중 CCTV fusion 및 중복 객체 제거
