# Test.md — CCTV + LiDAR 객체를 SLAM 맵 위에 올리기

이 문서는 **T자 교차로에서 로봇이 진입하기 전에 CCTV로 사각지대를 미리 보는**
데모를 처음부터 끝까지 재현하는 안내서입니다. 명령어 → 얻는 값 → 어디에 넣는지 →
왜 하는지 순서로 정리했습니다.

전체 흐름:

```
0. 빌드
1. 로봇/센서 구동 (scout_base + velodyne)  ── 기존 그대로
2. SLAM 맵 따기                     ── 기존 그대로
3. Nav2 주행                        ── 기존 그대로
4. 캘리브레이션 (CCTV 픽셀 ↔ map 미터)  ← 새 작업, 핵심
5. CCTV 객체를 맵에 올리기            ← 새 노드 cctv_to_map
6. LiDAR 객체를 맵에 올리기           ← 새 노드 lidar_object (racing_ws 모방, self-contained)
7. Fusion (두 센서 통합 레이어)        ← 새 노드 fusion
8. FOV / 사각지대 시각화              ← 새 노드 cctv_fov
9. 데모 시나리오
```

> 건드리지 않는 것: `yolo_detector_node`, `cam_to_map`(기존 이미지 디버그용),
> `scout_*`/`velodyne`(third_party), SLAM/Nav2 launch. 새 기능은 모두 **추가된 노드**입니다.

---

## 0. 빌드

워크스페이스 루트(`~/capstone-project`)에서:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select perception_msgs perception_pkg cctv_costmap_layer
source install/setup.bash
```

> **왜:** `perception_msgs`는 객체 정보(class, 속도, 예측경로)를 담는 메시지 패키지라
> `perception_pkg`보다 먼저 빌드돼야 합니다. colcon이 순서를 자동 처리합니다.

---

## 1. 로봇 / 센서 구동 (scout_base + velodyne — 기존 그대로)

하드웨어 드라이버는 `third_party/`의 scout_ros2·velodyne 패키지를 그대로 씁니다.
터미널을 나눠서:

```bash
# CAN + base
sudo ip link set can0 up type can bitrate 500000
ros2 launch scout_base scout_mini_base.launch.py
ros2 launch scout_description scout_base_description.launch.py

# base_link -> velodyne 정적 TF
ros2 run tf2_ros static_transform_publisher 0 0 0.6 0 0 0 base_link velodyne

# Velodyne 드라이버
ros2 launch velodyne velodyne-all-nodes-VLP32C-composed-launch.py
```

확인:

```bash
ros2 topic hz /velodyne_points   # ~10Hz
ros2 topic echo /scan --once     # 2D scan (SLAM 입력) 나오는지
```

> **왜:** `odom->base_link->velodyne` TF와 `/scan`이 있어야 SLAM과 Nav2가 돕니다.
> 이 단계는 모든 작업의 토대입니다.

---

## 2. SLAM 맵 따기 (기존 그대로)

```bash
# SLAM 시작 (map->odom 발행 시작)
ros2 launch bringup_pkg scout_slam.launch.py

# 로봇을 텔레오퍼레이션으로 교차로 전체를 천천히 2~3회 왕복
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

맵이 충분히 그려지면 저장:

```bash
ros2 run nav2_map_server map_saver_cli -f src/bringup_pkg/maps/intersection
```

얻는 것: `intersection.pgm`, `intersection.yaml`

> **왜:** 모든 객체(CCTV·LiDAR)는 이 `map` 좌표계 위에 올라갑니다. 맵의 `origin`과
> `resolution`이 좌표 기준이 됩니다. 교차로 구석·통로 끝까지 꼼꼼히 돌아야 나중에
> 캘리브레이션 마커 좌표를 읽을 때 빈 곳이 없습니다.

---

## 3. Nav2 주행 (기존 그대로)

SLAM을 끄고 저장된 맵으로 주행합니다.

```bash
ros2 launch bringup_pkg scout_cctv_nav2.launch.py \
  map:=src/bringup_pkg/maps/intersection.yaml
```

RViz2:
1. `Fixed Frame` = `map`
2. `2D Pose Estimate`로 로봇 초기 위치 지정 (AMCL 수렴)
3. `Nav2 Goal`로 목표 지정

> **왜:** `scout_cctv_nav2.launch.py`는 Nav2에 **CctvLayer**(CCTV 객체를 장애물로
> 올리는 커스텀 costmap 레이어)가 포함된 설정으로 뜹니다. 5~7단계에서 우리가 올린
> CCTV 객체가 여기 costmap에 반영되어 로봇이 우회합니다.

---

## 4. 캘리브레이션 — CCTV 픽셀 ↔ map 미터 (핵심 작업)

이 단계가 정확도를 좌우합니다. **바닥이 평평하므로** 카메라 이미지 픽셀과 바닥
위 실제 좌표는 단 하나의 3×3 행렬(homography)로 연결됩니다. 그 행렬을 구하려면
"이 픽셀 = 저 map 좌표" 대응쌍이 카메라당 8개 이상 필요합니다.

### 4-1. 바닥에 마커 붙이기

검은/밝은 테이프로 X 표식 8~12개를 교차로 바닥에 붙입니다.
- 일직선 금지(분산 배치), 간격 0.5m 이상, 카메라 화면 구석까지 포함.

### 4-2. 각 마커의 map 좌표(미터) 읽기 — 로봇으로 측정

로봇 중심(`base_link`)을 마커 위에 정확히 올린 뒤:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

출력의 `Translation: [x, y, z]`에서 **x, y가 그 마커의 map 좌표(미터)**입니다.
마커마다 기록하세요. (Nav2가 떠 있어 AMCL이 `map->base_link`를 알고 있어야 합니다.)

> **왜 RViz 클릭 대신 로봇으로?** AMCL이 추정한 로봇 위치는 맵 해상도(5cm)
> 수준으로 정확합니다. 마우스 클릭은 픽셀 단위 오차가 커서, 로봇을 직접 올리는
> 편이 homography 품질을 크게 높입니다. **모두 map frame입니다.**

### 4-3. 각 마커의 픽셀 좌표 읽기 — pixel_picker

카메라 영상에서 같은 마커를 클릭해 픽셀을 읽습니다.

```bash
ros2 run perception_pkg pixel_picker_node --ros-args -p image_topic:=/camera2/image_raw
```

창에서 마커를 4-2와 **같은 순서로** 클릭 → 터미널에 `#1 pixel = [u, v]` 출력.
`q`를 누르면 `image_points: [...]` 한 줄로 정리되어 출력됩니다.

### 4-4. 값을 config에 넣기

[config/cctv_to_map.yaml](config/cctv_to_map.yaml)을 엽니다. 카메라2 기준:

- `image_points`: 4-3에서 출력된 픽셀 리스트 붙여넣기
- `map_points`: 4-2에서 읽은 미터 좌표를 **같은 순서**로 입력

```yaml
cctv_to_map:
  ros__parameters:
    image_points: [441.0,366.0, 583.0,200.0, ...]   # 4-3 픽셀
    map_points:   [0.50,0.30,   1.20,0.90,   ...]    # 4-2 미터 (map frame)
```

> **왜:** 이 두 리스트로 `cv2.findHomography`가 "픽셀→미터" 변환식을 만듭니다.
> 순서/개수가 어긋나면 변환이 틀어지니 반드시 1:1로 맞추세요. 카메라가 2대면
> 카메라3용으로 이 블록을 복제해 별도 yaml로 저장합니다(아래 7-3 참고).

---

## 5. CCTV 객체를 맵에 올리기

YOLO + homography 변환 노드를 함께 띄웁니다.

```bash
ros2 launch perception_pkg cctv_to_map.launch.py
```

확인:

```bash
# 미터 단위 PoseArray (Nav2 CctvLayer + RViz가 사용)
ros2 topic echo /cctv/objects_map --once
# class/속도 등 풍부한 정보 (fusion이 사용)
ros2 topic echo /cctv/objects --once
```

RViz에서 `PoseArray`(`/cctv/objects_map`) 디스플레이를 추가하면, 카메라에 잡힌
사람이 맵 위 점으로 찍힙니다. 3단계 Nav2가 떠 있으면 그 점이 costmap 장애물이
되어 로봇이 우회합니다.

> **왜:** 기존 `cam_to_map`은 디버그용으로 맵 *이미지*에 점을 찍을 뿐 미터 좌표를
> 내보내지 않아, 지금까지 RViz 수동 클릭으로 대체하고 있었습니다. `cctv_to_map`은
> 그 클릭을 없애고 **YOLO 탐지 → map 미터 좌표 PoseArray**를 자동 발행합니다.
> 이로써 CCTV→costmap 경로가 완전 자동화됩니다.

> **정확도 점검:** 마커 위에 사람을 세우고 `/cctv/objects_map` 좌표가 4-2에서 읽은
> 값과 얼마나 차이 나는지 보세요. 오차(m)를 여러 번(20회+) 기록해 두면 7단계의
> `match_threshold`를 정하는 근거가 됩니다.

### 5-1. 팀원의 CCTV 추적기 연동 (중요)

팀원이 YOLO 바운딩박스를 추적하는 노드를 만든다면, **출력을
`vision_msgs/Detection2DArray`로 내고 각 `Detection2D.id`에 추적 ID(문자열)를
채우면** 됩니다. 그 토픽을 `cctv_to_map`의 `detections_topic`으로 지정합니다.

```
yolo_detector → /yolo/detections (id 비어있음)
      ↓
[팀원 추적기]  → /yolo/tracks (Detection2D.id = "1","2",... 채움)
      ↓
cctv_to_map (detections_topic:=/yolo/tracks)
```

```bash
ros2 run perception_pkg cctv_to_map --ros-args \
  --params-file install/perception_pkg/share/perception_pkg/config/cctv_to_map.yaml \
  -p detections_topic:=/yolo/tracks
```

> **왜 이게 되나:** `cctv_to_map`은 `Detection2D.id`를 그대로 보존하고, 그 ID로 map
> 위치를 시간 차분해 **map frame 속도[m/s]**까지 계산합니다. 덕분에 LiDAR가 못 보는
> 사각지대의 사람도 CCTV만으로 속도·예측 경로를 가집니다(교차로 시나리오 핵심).
> 추적기가 아직 없어 `id`가 비어 있으면 위치는 정상 동작하고 속도만 0이 됩니다
> (점진적 통합 가능).

---

## 6. LiDAR 객체를 맵에 올리기 (self-contained, racing_ws 불필요)

racing_ws의 인지 흐름(지면제거 → DBSCAN 클러스터링 → 추적)을 **이 워크스페이스
안에 모방해 넣은** `lidar_object` 노드를 씁니다. GitHub에서 pull 받은 누구나
racing_ws 없이 그대로 실행됩니다. (평탄한 실내 가정 → 지면제거를 z-밴드로 단순화)

```bash
ros2 run perception_pkg lidar_object --ros-args --params-file \
  install/perception_pkg/share/perception_pkg/config/lidar_object.yaml
```

확인:

```bash
ros2 topic echo /lidar/objects --once   # TrackedObjectArray, class="unknown", 속도 포함
```

RViz/Foxglove에서 보려면 fusion 마커(`/perception/markers`)에 파란색으로 표시됩니다.

내부 동작(노드 docstring 참고):
1. `/velodyne_points`에서 z-밴드 + 거리로 지면/천장/원거리 제거
2. 복셀 다운샘플 → DBSCAN(eps=0.40, min_samples=5, racing_ws와 동일값)
3. 클러스터 크기로 벽 등 큰 구조물 제외 (`max_object_size`)
4. centroid를 TF로 map frame[m] 변환
5. 등속 칼만필터(상태 [x,y,vx,vy]) + 헝가리안 데이터연관으로 ID 유지·속도[m/s] 추정
   (map frame 추적이라 로봇 이동/회전에 강건, 짧은 가려짐은 KF 예측으로 유지)

> **왜 모방인가:** racing_ws의 GPU DBSCAN/Patchwork++를 그대로 가져오면 외부
> 워크스페이스·CUDA에 묶여 "pull 받으면 안 도는" 코드가 됩니다. 평탄한 실내에서는
> 지면제거가 z-밴드로 충분하고, DBSCAN은 동일 파라미터로 CPU에서 돌므로, 검증된
> 알고리즘을 **의존성 없이** 재현했습니다.

> **출력 형식 통일:** LiDAR도 CCTV와 **똑같은 `TrackedObjectArray`**를 냅니다. 차이는
> `class_id="unknown"` 뿐입니다. 그래서 fusion이 두 소스를 동일하게 다룹니다.
> (팀원이 다른 추적 방식을 쓰더라도 이 메시지 형식만 맞추면 그대로 융합됩니다.)

> **튜닝:** 벽이 객체로 잡히면 `max_object_size`를 줄이고, 사람이 안 잡히면 `eps`나
> `max_range`를 키우세요. 값은 [config/lidar_object.yaml](config/lidar_object.yaml).

---

## 7. Fusion — 두 센서를 하나의 레이어로

### 7-1. 실행

```bash
ros2 run perception_pkg fusion --ros-args --params-file \
  install/perception_pkg/share/perception_pkg/config/fusion.yaml
```

(또는 전체 데모 launch: `ros2 launch perception_pkg perception_demo.launch.py`)

> ⚠️ **토픽 충돌 주의 (`/cctv/objects_map`):** 이 PoseArray를 **cctv_to_map과 fusion이
> 둘 다 발행**합니다. costmap(CctvLayer)이 받는 토픽이라 동시에 쓰면 서로 덮어씁니다.
> - 5절처럼 **fusion 없이 단독 테스트**: cctv_to_map이 직접 발행 → 정상
> - **fusion까지 함께 구동**: cctv_to_map의 PoseArray를 다른 토픽으로 돌려 fusion만
>   `/cctv/objects_map`을 담당하게 합니다(통합본이 costmap에 들어가도록).
>   ```bash
>   # fusion 동시 구동 시 cctv_to_map은 PoseArray를 사용 안 하는 토픽으로
>   ros2 run perception_pkg cctv_to_map --ros-args \
>     --params-file install/perception_pkg/share/perception_pkg/config/cctv_to_map.yaml \
>     -p objects_topic:=/cctv/cam2/objects_map
>   ```
> (지금은 디버깅 편의를 위해 기본값을 단독 테스트 기준으로 둡니다. 추후 launch에서
> 분리 예정.)

출력:

```bash
ros2 topic echo /perception/tracked_objects --once  # 통합 레이어 (id/class/속도/예측)
ros2 topic echo /cctv/objects_map --once            # costmap용 PoseArray (재발행)
# RViz: /perception/markers (MarkerArray) 추가
```

마커 색: **초록=FUSED, 파랑=LiDAR-only, 주황=CCTV-only**. 각 객체 위에
`#id class [source] 속도` 텍스트와 예측 경로(LINE_STRIP)가 표시됩니다.

### 7-2. match_threshold 정하기

같은 객체로 묶는 거리 기준(미터)입니다. [config/fusion.yaml](config/fusion.yaml)
`match_threshold`. 5단계에서 측정한 CCTV-LiDAR 위치 오차의 95% 지점을 시작값으로
쓰세요(보통 0.5~1.0m, 기본 0.6).

> **왜:** 너무 작으면 같은 사람을 둘로(CCTV+LiDAR 따로) 표시하고, 너무 크면 다른
> 두 객체를 하나로 합칩니다. 실측 오차 기반으로 정해야 합니다.

### 7-3. 카메라 2대로 확장

카메라3용 YOLO + cctv_to_map을 하나 더 띄우되 토픽을 분리합니다:

```bash
# 카메라3 cctv_to_map은 별도 config(cctv_to_map_cam3.yaml)에서
#   image_topic(yolo): /camera3/image_raw
#   objects_topic:      /cctv/cam3/objects_map
#   objects_full_topic: /cctv/cam3/objects
```

그리고 fusion.yaml:

```yaml
cctv_topics: ["/cctv/objects", "/cctv/cam3/objects"]
```

> **왜:** 두 카메라가 같은 토픽에 쓰면 서로 덮어씁니다. 카메라별 토픽으로 분리하면
> fusion이 둘을 받아 **CCTV간 병합** 후 LiDAR와 합칩니다.

---

## 8. FOV / 사각지대 시각화

### 8-1. 카메라 위치·방향 측정

- 위치(x, y): 로봇을 카메라 렌즈 바로 아래 바닥에 두고 `tf2_echo map base_link`
- 높이(z): 줄자
- yaw(수평 방향): 측정이 어려우면 추정값으로 두거나, homography 기반 추정값 사용
- hfov: C922 ≈ 56°, C930e ≈ 67° (이미 알고 있는 값)

[config/cctv_fov.yaml](config/cctv_fov.yaml)의 `cameras`에
`[x, y, yaw_deg, hfov_deg, range_m, color_id]`로 입력.

### 8-2. 실행

```bash
ros2 run perception_pkg cctv_fov --ros-args --params-file \
  install/perception_pkg/share/perception_pkg/config/cctv_fov.yaml
```

RViz에 `/cctv/fov_markers`(MarkerArray) 추가 → 카메라 커버 영역이 반투명 부채꼴로
표시됩니다. 부채꼴 **밖**이 곧 CCTV가 못 보는 사각지대입니다.

> **왜:** 데모에서 "로봇이 못 보는 영역을 CCTV가 본다"를 시각적으로 보여줍니다.
> 부채꼴과 실제 카메라 화면을 비교하면 yaw/hfov 값을 검증할 수 있습니다.

---

## 9. 데모 시나리오 (검증)

1. 로봇이 직선 통로에서 교차로로 자율주행(Nav2 Goal).
2. 실험자가 횡단 통로(로봇 사각지대)에서 걸어다님 → **CCTV만** 감지.
   - `/perception/tracked_objects`: `class=person, source=CCTV`, 예측경로 표시
   - Nav2 costmap에 장애물로 올라가 로봇이 교차로 진입 전에 감속/정지
3. 로봇이 교차로에 접근해 LiDAR가 같은 사람을 봄 → `source=FUSED`
   (위치/속도는 LiDAR, class는 CCTV).
4. 사람이 다시 LiDAR 사각지대로 가면 → `source=CCTV`로 전환.

> **이것이 프로젝트의 핵심 결과:** 로봇이 **직접 보기 전에** 인프라(CCTV)가 준
> 정보로 교차로 위험을 회피합니다.

---

## 10. 기존 layer 코드(CctvLayer) 분석 & 활용

`cctv_costmap_layer` 패키지에 이미 만들어진 **Nav2 costmap 플러그인**이 있습니다.
이것이 "객체를 경로계획에 반영"하는 핵심이며, 우리는 이걸 **그대로 재사용**합니다.

동작 ([cctv_layer.cpp](../cctv_costmap_layer/src/cctv_layer.cpp)):
- `/cctv/objects_map` (PoseArray, map frame [m]) 구독
- 각 pose 주변 `default_radius`[m]를 `LETHAL_OBSTACLE`로 칠함 (`markObject`)
- `max_observation_age`[s] 지난 pose는 자동 제거 → 객체가 사라지면 장애물도 사라짐
- `nav2_params_cctv.yaml`에서 local/global costmap의 plugin으로 등록됨
  (global `default_radius` 0.35 m, local 0.25 m)

연결 방식:
```
fusion → /cctv/objects_map (통합된 모든 객체를 PoseArray로 재발행)
            ↓
       CctvLayer (기존 코드) → costmap LETHAL → Nav2 경로 우회
```

> **왜 이렇게:** fusion이 CCTV·LiDAR·FUSED 객체 전부를 `/cctv/objects_map`으로 다시
> 내보내므로, **기존 CctvLayer 코드를 한 줄도 안 고치고** 통합 레이어가 그대로
> costmap에 반영됩니다. 특히 CCTV-only(사각지대) 객체도 장애물이 되어, 로봇이 직접
> 못 보는 위험을 경로계획이 미리 피합니다.

> 참고: LiDAR-only 객체는 Nav2 기본 `obstacle_layer`(/scan)에도 이미 잡히므로
> CctvLayer를 통한 표시는 중복이지만 무해합니다. 사각지대 CCTV 객체를 올리는 것이
> 이 레이어의 진짜 가치입니다.

---

## 11. Foxglove로 SLAM 맵 + 레이어 시각화

발표용으로 RViz보다 깔끔합니다. `foxglove_bridge`가 이미 설치돼 있습니다.

### 11-1. 브릿지 실행

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
# 기본 포트 ws://localhost:8765 로 ROS 토픽을 WebSocket으로 노출
```

### 11-2. Foxglove Studio 접속

- 데스크톱 앱(권장) 또는 브라우저 `https://app.foxglove.dev`
- `Open connection` → `Foxglove WebSocket` → `ws://localhost:8765`
  (원격 PC면 localhost 대신 로봇 IP)

### 11-3. 3D 패널 구성 (맵 위에 레이어 쌓기)

`3D` 패널을 추가하고 다음 토픽을 켭니다:

| 토픽 | 타입 | 보이는 것 |
| --- | --- | --- |
| `/map` | OccupancyGrid | **SLAM 맵** (바닥 평면) |
| `/perception/markers` | MarkerArray | 통합 객체(원기둥)+라벨+예측경로 |
| `/cctv/fov_markers` | MarkerArray | CCTV FOV 부채꼴/사각지대 |
| `/tf`, `/tf_static` | TF | 로봇·센서 좌표축 |
| `/scan` 또는 `/velodyne_points` | LaserScan/PointCloud2 | 라이브 센서 |
| `/local_costmap/costmap` | OccupancyGrid | costmap 반영 확인(선택) |

- `3D` 패널 설정에서 `Fixed frame`(또는 `Display frame`)을 **`map`**으로.
- `/map`이 안 보이면 Nav2/SLAM에서 map_server가 `/map`을 발행 중인지 확인:
  `ros2 topic echo /map --once | head`

### 11-4. 카메라 화면 같이 띄우기

`Image` 패널 추가 → `/yolo/annotated_image` 선택 → YOLO 바운딩박스가 그려진
CCTV 영상이 3D 맵 옆에 나란히 표시됩니다.

### 11-5. 레이아웃 저장

상단 레이아웃 메뉴에서 저장하면 다음 발표 때 그대로 불러옵니다. 팀 공유도 가능.

> **왜 Foxglove:** OccupancyGrid 맵 + 우리 객체 레이어(MarkerArray) + FOV + 카메라
> 영상을 **한 화면**에 묶어 보여줄 수 있어, "맵 위에 무엇이 어떻게 쌓이는지"를
> 디버그가 아니라 발표 품질로 전달합니다.

---

## 토픽 요약

| 토픽 | 타입 | 발행 노드 | 용도 |
| --- | --- | --- | --- |
| `/yolo/detections` | Detection2DArray | yolo_detector_node (기존) | YOLO 탐지 |
| `/cctv/objects_map` | PoseArray | cctv_to_map → (fusion 재발행) | Nav2 CctvLayer 입력 |
| `/cctv/objects` | TrackedObjectArray | cctv_to_map | class 포함 CCTV 객체 |
| `/lidar/objects` | TrackedObjectArray | lidar_object | LiDAR 객체(class=unknown, 속도) |
| `/perception/tracked_objects` | TrackedObjectArray | fusion | **통합 레이어** |
| `/perception/markers` | MarkerArray | fusion | RViz/Foxglove 시각화 |
| `/cctv/fov_markers` | MarkerArray | cctv_fov | FOV/사각지대 |

## 측정해서 채워야 할 값 체크리스트

| 파일 | 항목 | 출처 |
| --- | --- | --- |
| `config/cctv_to_map.yaml` | `image_points` | pixel_picker (4-3) |
| `config/cctv_to_map.yaml` | `map_points` (미터) | tf2_echo map base_link (4-2) |
| `config/lidar_object.yaml` | `max_object_size`, `eps` 등 | 현장 튜닝 (6) |
| `config/fusion.yaml` | `match_threshold` | CCTV-LiDAR 오차 측정 (7-2) |
| `config/cctv_fov.yaml` | `cameras` | 카메라 위치/yaw/hfov (8-1) |
