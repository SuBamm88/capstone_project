# CCTV PoseArray Costmap Layer

## 한 줄 요약

CCTV가 발견한 객체 위치를 `/cctv/objects_map` 토픽의 `geometry_msgs/msg/PoseArray`로 받고, custom Nav2 costmap layer가 해당 위치를 장애물로 표시한 뒤, 기존 inflation layer가 이를 부풀려 global/local path planning에 반영한다.

## 쉽게 이해하기

기본 Nav2 주행은 다음 정보만 사용한다.

```text
저장된 지도 + 로봇 LiDAR 장애물
-> Nav2 costmap
-> 경로 계획
-> 주행
```

이번 CCTV 데모는 여기에 CCTV 객체 위치를 하나 더 추가한다.

```text
저장된 지도 + 로봇 LiDAR 장애물 + CCTV 객체 위치
-> Nav2 costmap
-> 경로 계획
-> 주행
```

즉, 로봇 onboard sensor로 아직 보이지 않는 객체라도 CCTV가 map 좌표계 위치를 알려주면 Nav2가 그 위치를 장애물처럼 보고 경로를 바꾸는 것을 보여주는 데모이다.

중간발표 단계에서는 risk, class, confidence, velocity 같은 복잡한 위험도 계산은 하지 않는다. 모든 CCTV 객체 pose를 동일한 장애물로 보고, Nav2 inflation layer가 주변 safety margin을 만든다.

## 데모 목표

중간발표까지의 목표는 다음이다.

1. CCTV 객체 위치가 map frame 좌표로 들어왔다고 가정한다.
2. `/cctv/objects_map` 토픽으로 객체 위치 목록을 받는다.
3. custom costmap layer가 해당 위치를 Nav2 costmap에 장애물로 올린다.
4. global costmap과 local costmap 모두 CCTV layer를 사용한다.
5. inflation layer가 CCTV obstacle 주변을 부풀린다.
6. Nav2 planner/controller가 CCTV obstacle을 반영해 경로를 변경하거나 회피한다.

발표에서 보여줄 핵심 메시지는 다음이다.

```text
로봇 자체 센서로 보이지 않는 객체의 위치를 CCTV가 먼저 알려주면,
그 정보가 Nav2 costmap에 반영되어 global/local 경로 계획에 영향을 줄 수 있다.
```

## 계획한 내용

처음 계획은 다음 구조였다.

```text
CCTV detection / homography / TF 변환 파트
-> /cctv/objects_map
-> custom Nav2 costmap layer
-> inflation layer
-> global/local path planning
-> Scout Mini 경로 추종
```

초기 계획에서는 custom msg도 고려했다. 예를 들어 object id, class, confidence, covariance, velocity, radius를 담는 별도 메시지를 만들 수 있었다.

하지만 데모 단계에서는 팀원들이 쉽게 연결하고, ROS2 기본 도구로 확인하기 쉽도록 custom msg를 만들지 않기로 했다. 최종 계획은 공식 ROS2 메시지인 `geometry_msgs/msg/PoseArray`를 사용하는 것으로 정리했다.

최종 계획의 핵심 결정은 다음이다.

- 입력 토픽은 `/cctv/objects_map`
- 메시지 타입은 `geometry_msgs/msg/PoseArray`
- `header.frame_id`는 기본 `map`
- `poses[].position.x`, `poses[].position.y`를 객체 위치로 사용
- `poses[].orientation`은 데모에서는 사용하지 않음
- 모든 pose는 동일한 obstacle로 처리
- 객체 크기는 msg가 아니라 layer 파라미터 `default_radius`로 설정
- global/local costmap 모두 CCTV layer를 inflation layer 앞에 추가
- 1\~3번 파트가 완성되기 전에는 RViz 클릭으로 fake CCTV 객체를 만든다
- 1\~3번 파트가 완성되면 같은 `/cctv/objects_map` PoseArray만 publish하면 바로 연결한다

## 구현한 내용

이번 구현으로 새 패키지 2개와 Nav2 CCTV 설정이 추가되었다.

### 1. `cctv_costmap_layer`

경로:

```text
src/cctv_costmap_layer
```

역할:

- Nav2 custom costmap layer plugin
- `/cctv/objects_map` 구독
- `geometry_msgs/msg/PoseArray` 수신
- 각 pose를 costmap frame으로 TF 변환
- 각 pose 주변을 원형 lethal obstacle로 표시
- 기존 Nav2 inflation layer가 그 obstacle을 inflation

주요 파일:

```text
src/cctv_costmap_layer/include/cctv_costmap_layer/cctv_layer.hpp
src/cctv_costmap_layer/src/cctv_layer.cpp
src/cctv_costmap_layer/cctv_costmap_plugins.xml
src/cctv_costmap_layer/CMakeLists.txt
src/cctv_costmap_layer/package.xml
```

layer 동작 방식:

```text
PoseArray 수신
-> 각 pose의 frame 확인
-> global/local costmap frame으로 TF 변환
-> default_radius 안의 cell을 LETHAL_OBSTACLE로 set
-> updateWithMax로 master costmap에 병합
-> inflation layer가 주변 cost 생성
```

현재 기본 파라미터:

```yaml
cctv_layer:
  plugin: "cctv_costmap_layer::CctvLayer"
  enabled: True
  topic: /cctv/objects_map
  default_radius: 0.25
  max_observation_age: 1.0
  transform_tolerance: 0.2
```

파라미터 의미:

- `topic`: CCTV 객체 위치를 받을 토픽
- `default_radius`: 각 pose를 장애물로 찍을 반경
- `max_observation_age`: 오래된 PoseArray를 버리는 시간
- `transform_tolerance`: TF 변환 대기 허용 시간

### 2. `cctv_object_tools`

경로:

```text
src/cctv_object_tools
```

역할:

- 1\~3번 CCTV 인지/변환 파트가 완성되기 전, RViz 클릭으로 fake CCTV 객체를 만든다.
- RViz `Publish Point`가 publish하는 `/clicked_point`를 받아 pose 목록으로 누적한다.
- 누적 pose 목록을 `/cctv/objects_map`의 `PoseArray`로 주기 publish한다.
- RViz 확인용 marker도 publish한다.
- clear / undo service를 제공한다.

주요 파일:

```text
src/cctv_object_tools/cctv_object_tools/rviz_click_to_pose_array.py
src/cctv_object_tools/launch/rviz_click_to_pose_array.launch.py
src/cctv_object_tools/setup.py
src/cctv_object_tools/package.xml
```

입출력:

```text
입력:
  /clicked_point
  geometry_msgs/msg/PointStamped

출력:
  /cctv/objects_map
  geometry_msgs/msg/PoseArray

시각화:
  /cctv/objects_marker
  visualization_msgs/msg/MarkerArray

서비스:
  /cctv/clear_objects
  /cctv/remove_last_object
```

### 3. `scout_mini_bringup` CCTV Nav2 설정

추가 파일:

```text
src/scout_mini_bringup/config/nav2_params_cctv.yaml
src/scout_mini_bringup/launch/scout_cctv_nav2.launch.py
```

기존 `nav2_params.yaml`은 기본 Nav2 주행용으로 유지했다. CCTV 데모는 별도 파일인 `nav2_params_cctv.yaml`을 사용한다.

global costmap 구성:

```yaml
plugins: ["static_layer", "obstacle_layer", "cctv_layer", "inflation_layer"]
```

의미:

- `static_layer`: 저장된 map
- `obstacle_layer`: 로봇 LiDAR 기반 장애물
- `cctv_layer`: CCTV PoseArray 기반 장애물
- `inflation_layer`: static / obstacle / CCTV obstacle 주변 inflation

local costmap 구성:

```yaml
plugins: ["voxel_layer", "cctv_layer", "inflation_layer"]
```

의미:

- `voxel_layer`: 로봇 LiDAR 기반 local obstacle
- `cctv_layer`: CCTV PoseArray 기반 obstacle
- `inflation_layer`: local costmap에서 obstacle 주변 inflation

## 사용 메시지 형식

토픽:

```text
/cctv/objects_map
```

타입:

```text
geometry_msgs/msg/PoseArray
```

예시:

```yaml
header:
  frame_id: map
poses:
- position:
    x: 1.0
    y: 2.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

데모 단계 해석:

- `position.x`: map frame 기준 객체 x 좌표
- `position.y`: map frame 기준 객체 y 좌표
- `position.z`: 사용하지 않음
- `orientation`: 사용하지 않음
- 여러 객체는 `poses` 배열에 여러 pose로 넣는다

주의:

- PoseArray는 object id, class, confidence, radius를 담지 않는다.
- 따라서 중간발표 데모에서는 모든 객체를 같은 크기의 obstacle로 처리한다.
- 객체 크기와 safety margin은 `default_radius`와 inflation radius로 조정한다.

## 실행 전 준비

아래 명령 예시는 워크스페이스 루트에서 실행한다고 가정한다.

```bash
cd /home/youngwoo/capstone/workspace/scout_ws
```

필요 조건:

- SCOUT MINI와 CAN 연결 완료
- Velodyne VLP-32C 연결 완료
- 기존 map 생성 완료
- `/scan` 토픽 정상 publish
- TF tree 정상 구성

정상 TF 구조:

```text
map -> odom -> base_link -> velodyne
```

역할:

- `map -> odom`: AMCL
- `odom -> base_link`: scout\_base
- `base_link -> velodyne`: static TF

## 빌드

처음 한 번 빌드한다.

```bash
cd /home/youngwoo/capstone/workspace/scout_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cctv_costmap_layer cctv_object_tools scout_mini_bringup
source install/setup.bash
```

빌드 확인:

```bash
ros2 pkg prefix cctv_costmap_layer
ros2 pkg executables cctv_object_tools
```

`cctv_object_tools rviz_click_to_pose_array`가 보이면 RViz 클릭 도구 노드가 설치된 것이다.

## 실제 Scout CCTV 데모 실행 순서

### 1. CAN 인터페이스 up + Scout base 실행

터미널 1:

```bash
sudo ip link set can0 up type can bitrate 500000
ros2 launch scout_base scout_mini_base.launch.py
```

설명:

- Scout Mini base driver 실행
- `/odom` publish
- `odom -> base_link` TF publish

### 2. Robot description 실행

터미널 2:

```bash
source /opt/ros/humble/setup.bash
source /home/youngwoo/capstone/workspace/scout_ws/install/setup.bash
ros2 launch scout_description scout_base_description.launch.py
```

설명:

- URDF 기반 robot state publish
- RViz에서 로봇 모델 확인 가능

### 3. Velodyne 정적 TF 실행

터미널 3:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0.6 0 0 0 base_link velodyne
```

설명:

- `base_link -> velodyne` TF publish

### 4. Velodyne 드라이버 실행

터미널 4:

```bash
ros2 launch velodyne velodyne-all-nodes-VLP32C-composed-launch.py
```

설명:

- Velodyne driver 실행
- pointcloud / laserscan 관련 노드 실행
- Nav2 obstacle layer가 사용할 `/scan` 준비

확인:

```bash
ros2 topic list | grep scan
ros2 topic echo /scan --once
```

### 5. CCTV layer 포함 Nav2 실행

터미널 5:

```bash
ros2 launch scout_mini_bringup scout_cctv_nav2.launch.py map:=src/scout_mini_bringup/maps/scout_mini_map_3.yaml
```

설명:

- 저장된 map 로드
- AMCL 실행
- Nav2 planner/controller 실행
- global/local costmap에 `cctv_layer` 포함

기본 Nav2와 차이:

```text
기본:
  ros2 launch scout_mini_bringup scout_nav2.launch.py ...

CCTV 데모:
  ros2 launch scout_mini_bringup scout_cctv_nav2.launch.py ...
```

### 6. RViz 클릭 기반 CCTV 객체 publisher 실행

터미널 6:

```bash
cd /home/youngwoo/capstone/workspace/scout_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cctv_object_tools rviz_click_to_pose_array.launch.py
```

설명:

- RViz `Publish Point` 클릭을 fake CCTV 객체로 사용
- 클릭된 점들을 `/cctv/objects_map` PoseArray로 publish
- `/cctv/objects_marker`로 시각화 marker publish

### 7. RViz2 실행 및 조작

터미널 7:

```bash
source /opt/ros/humble/setup.bash
source /home/youngwoo/capstone/workspace/scout_ws/install/setup.bash
rviz2
```

RViz 설정:

1. `Fixed Frame`을 `map`으로 설정한다.
2. `Map`, `LaserScan`, `TF`, `RobotModel`, `Path`, `Costmap` display를 켠다.
3. `2D Pose Estimate`로 로봇 초기 위치를 지정한다.
4. AMCL 위치 추정이 안정될 때까지 기다린다.
5. `Nav2 Goal` 또는 `2D Goal Pose`로 목표 지점을 준다.
6. `Publish Point` 도구를 선택한다.
7. 로봇 경로 위나 사각지대 역할을 하는 위치를 클릭한다.

클릭 후 기대 동작:

```text
RViz 클릭
-> /clicked_point publish
-> rviz_click_to_pose_array 노드가 pose 누적
-> /cctv/objects_map PoseArray publish
-> cctv_layer가 costmap에 obstacle 표시
-> inflation_layer가 obstacle 주변 cost 생성
-> Nav2 경로가 변경되거나 local planner가 회피
```

## 데모 중 객체 삭제

모든 CCTV demo 객체 삭제:

```bash
ros2 service call /cctv/clear_objects std_srvs/srv/Empty
```

마지막으로 클릭한 객체 하나 삭제:

```bash
ros2 service call /cctv/remove_last_object std_srvs/srv/Empty
```

삭제 후 기대 동작:

```text
/cctv/objects_map이 빈 PoseArray로 publish
-> cctv_layer obstacle 제거
-> costmap에서 CCTV obstacle 사라짐
-> 경로가 기존 경로로 복구 가능
```

## 토픽 확인 명령어

CCTV PoseArray 확인:

```bash
ros2 topic echo /cctv/objects_map
```

CCTV marker 확인:

```bash
ros2 topic echo /cctv/objects_marker --once
```

토픽 주기 확인:

```bash
ros2 topic hz /cctv/objects_map
```

Nav2 costmap 토픽 확인:

```bash
ros2 topic list | grep costmap
```

TF 확인:

```bash
ros2 run tf2_tools view_frames
```

특정 TF 확인:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

## 수동 PoseArray publish 예시

RViz 클릭 노드 없이 직접 테스트할 수도 있다.

```bash
ros2 topic pub --once /cctv/objects_map geometry_msgs/msg/PoseArray "{
  header: {frame_id: 'map'},
  poses: [
    {
      position: {x: 1.0, y: 2.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  ]
}"
```

여러 객체 publish 예시:

```bash
ros2 topic pub /cctv/objects_map geometry_msgs/msg/PoseArray "{
  header: {frame_id: 'map'},
  poses: [
    {
      position: {x: 1.0, y: 2.0, z: 0.0},
      orientation: {w: 1.0}
    },
    {
      position: {x: 1.5, y: 2.5, z: 0.0},
      orientation: {w: 1.0}
    }
  ]
}" -r 5
```

## 실제 1\~3번 파트와 연결하는 방법

팀원 파트가 완성되면 RViz 클릭 노드는 끄고, 팀원 노드가 아래 토픽만 publish하면 된다.

```text
topic: /cctv/objects_map
type: geometry_msgs/msg/PoseArray
frame_id: map
```

팀원 노드 출력 조건:

- 각 객체의 bbox 하단 중심점 등을 homography로 map 좌표계로 변환한다.
- 변환된 객체 위치를 `poses[].position.x/y`에 넣는다.
- `header.frame_id`는 `map`으로 둔다.
- 여러 CCTV에서 온 객체는 하나의 PoseArray에 모아도 되고, 같은 토픽으로 계속 갱신 publish해도 된다.
- publish rate는 데모 기준 5 Hz 정도면 충분하다.

중요:

- 현재 layer는 최신 PoseArray 하나를 현재 CCTV 객체 목록으로 본다.
- 따라서 객체가 사라졌으면 다음 PoseArray에서 해당 pose를 빼야 한다.
- 빈 PoseArray를 publish하면 CCTV obstacle이 제거된다.

## 발표 시연 시나리오

### 시나리오 1: CCTV 객체 없음

1. Nav2 goal을 준다.
2. 로봇이 기본 경로를 생성한다.
3. 이 경로를 baseline으로 보여준다.

핵심:

```text
기본 Nav2는 저장 map과 로봇 LiDAR 기반 costmap만 사용한다.
```

### 시나리오 2: CCTV 객체 추가

1. 로봇 경로 위나 사각지대 위치를 RViz `Publish Point`로 클릭한다.
2. `/cctv/objects_map`에 pose가 publish된다.
3. costmap에 CCTV obstacle이 생긴다.
4. inflation이 생긴다.
5. Nav2 경로가 바뀌거나 local planner가 회피한다.

핵심:

```text
로봇이 직접 보기 전이라도 CCTV 정보가 costmap에 들어가면 경로 계획에 반영된다.
```

### 시나리오 3: CCTV 객체 제거

1. `/cctv/clear_objects` service를 호출한다.
2. CCTV obstacle이 costmap에서 사라진다.
3. 경로가 원래처럼 복구되는지 보여준다.

핵심:

```text
CCTV 정보는 동적으로 들어오고 사라질 수 있으며, Nav2 costmap도 이에 맞춰 갱신된다.
```

## 파라미터 조정 가이드

### CCTV obstacle 자체 크기 조정

파일:

```text
src/scout_mini_bringup/config/nav2_params_cctv.yaml
```

항목:

```yaml
cctv_layer:
  default_radius: 0.25
```

값을 키우면 클릭한 객체 자체가 더 큰 장애물로 찍힌다.

### global inflation 크기 조정

항목:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: 0.55
```

값을 키우면 global planner가 CCTV obstacle 주변을 더 넓게 피한다.

### local inflation 크기 조정

항목:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: 0.20
```

값을 키우면 local planner가 근거리에서 더 보수적으로 회피한다.

### 오래된 CCTV 정보 제거 시간 조정

항목:

```yaml
cctv_layer:
  max_observation_age: 1.0
```

값이 너무 작으면 publish가 잠깐 끊겼을 때 obstacle이 사라진다. 값이 너무 크면 오래된 CCTV 객체가 오래 남는다.

## 문제 해결

### `/cctv/objects_map`이 안 보일 때

확인:

```bash
ros2 topic list | grep cctv
ros2 topic echo /cctv/objects_map --once
```

가능한 원인:

- `rviz_click_to_pose_array` 노드를 실행하지 않았다.
- RViz에서 `Publish Point`를 사용하지 않았다.
- RViz Fixed Frame이 `map`이 아니다.

### RViz 클릭했는데 객체가 추가되지 않을 때

확인:

```bash
ros2 topic echo /clicked_point --once
```

가능한 원인:

- RViz `Publish Point` 도구가 선택되지 않았다.
- 클릭한 point의 frame이 `map`이 아니다.
- `rviz_click_to_pose_array` 노드는 기본적으로 `map` frame 클릭만 받는다.

### Costmap에 CCTV obstacle이 안 보일 때

확인:

```bash
ros2 topic echo /cctv/objects_map --once
ros2 topic list | grep costmap
```

확인할 설정:

```text
src/scout_mini_bringup/config/nav2_params_cctv.yaml
```

global/local costmap plugin 목록에 `cctv_layer`가 있어야 한다.

```yaml
plugins: ["static_layer", "obstacle_layer", "cctv_layer", "inflation_layer"]
plugins: ["voxel_layer", "cctv_layer", "inflation_layer"]
```

### TF transform 경고가 날 때

가능한 원인:

- `map -> odom` TF가 없다.
- AMCL이 아직 초기 pose를 받지 못했다.
- PoseArray `header.frame_id`가 잘못됐다.

확인:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

해결:

- RViz에서 `2D Pose Estimate`로 초기 위치를 준다.
- PoseArray의 `header.frame_id`를 `map`으로 맞춘다.

### 경로가 안 바뀔 때

가능한 원인:

- 클릭한 위치가 실제 global path와 멀다.
- inflation radius가 너무 작다.
- planner가 아직 재계획하지 않았다.
- local costmap window 밖에 찍었다.

대응:

- global path 바로 위를 클릭한다.
- `global_costmap` inflation radius를 키운다.
- goal을 다시 찍어 global planning을 유도한다.
- local 회피를 보고 싶으면 로봇 근처 local costmap window 안에 찍는다.

## 현재 구현 한계

중간발표 데모를 위해 의도적으로 단순화한 부분이다.

- 객체 class를 구분하지 않는다.
- 사람, 박스, 지게차를 모두 같은 obstacle로 본다.
- confidence를 사용하지 않는다.
- velocity / heading을 사용하지 않는다.
- 객체별 radius를 PoseArray에 담을 수 없다.
- 여러 CCTV의 중복 객체 fusion은 하지 않는다.
- risk score나 Gaussian risk map은 아직 구현하지 않았다.
- 현재 목표는 risk-aware navigation 완성본이 아니라 CCTV 위치 정보가 Nav2 costmap에 반영될 수 있음을 보이는 것이다.

## 이후 확장 방향

중간발표 이후에는 다음 방향으로 확장할 수 있다.

1. custom msg 도입
   - object id
   - class id
   - confidence
   - covariance
   - velocity
   - object radius
2. class별 cost 차등 적용
   - person: 넓은 inflation
   - box: 일반 obstacle
   - forklift: 진행 방향 risk
   - unknown: 보수적 risk
3. 동적 객체 risk
   - velocity 기반 heading 방향 cost
   - 예측 위치 표시
   - time decay
4. 여러 CCTV fusion
   - 같은 객체 중복 제거
   - camera id 기반 source 관리
   - timestamp 기반 stale object 제거
5. 발표용 시각화 강화
   - CCTV 객체 marker
   - CCTV obstacle layer only view
   - CCTV 사용 전/후 path 비교
   - costmap 변화 영상 저장

## 최종 데모 체크리스트

실제 시연 전 다음을 확인한다.

- Scout base driver 실행
- `/odom` publish 확인
- `base_link -> velodyne` TF 확인
- Velodyne 실행
- `/scan` publish 확인
- CCTV Nav2 launch 실행
- AMCL 초기 pose 지정
- `map -> odom -> base_link` TF 확인
- RViz click publisher 실행
- `/cctv/objects_map` publish 확인
- `/cctv/objects_marker` 표시 확인
- global/local costmap에 CCTV obstacle 표시 확인
- CCTV obstacle 추가 전/후 path 변화 확인
- `/cctv/clear_objects`로 obstacle 제거 확인

## 가장 짧은 실행 명령 요약

빌드:

```bash
cd /home/youngwoo/capstone/workspace/scout_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cctv_costmap_layer cctv_object_tools scout_mini_bringup
source install/setup.bash
```

CCTV Nav2:

```bash
ros2 launch scout_mini_bringup scout_cctv_nav2.launch.py map:=src/scout_mini_bringup/maps/scout_mini_map.yaml
```

RViz 클릭 publisher:

```bash
ros2 launch cctv_object_tools rviz_click_to_pose_array.launch.py
```

객체 삭제:

```bash
ros2 service call /cctv/clear_objects std_srvs/srv/Empty
```
