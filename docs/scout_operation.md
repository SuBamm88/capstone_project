## 실행 전

- SCOUT MINI와 CAN 연결 완료
- Velodyne VLP-32C 연결 완료

아래 명령 예시는 워크스페이스 루트에서 실행한다고 가정한다.


### 1. CAN 인터페이스 up + SCOUT MINI base 실행

```bash
sudo ip link set can0 up type can bitrate 500000
ros2 launch scout_base scout_mini_base.launch.py
```
```bash
ros2 launch scout_description scout_base_description.launch.py
```

```bash
ros2 launch scout_description scout_base_description.launch.py
```

설명:

- SCOUT MINI base driver 실행
- `/odom` publish
- `odom -> base_link` TF publish

### 2. Velodyne 정적 TF 실행

`base_link -> velodyne`


```bash
ros2 run tf2_ros static_transform_publisher 0 0 0.6 0 0 0 base_link velodyne
```

### 3. Velodyne 드라이버 및 포인트클라우드 실행

```bash
ros2 launch velodyne velodyne-all-nodes-VLP32C-composed-launch.py
```

설명:

- Velodyne 드라이버 실행
- PointCloud 관련 노드 실행

## Mapping

맵 작성 단계에서는 `slam_toolbox`를 사용한다.

```bash
ros2 launch bringup_pkg scout_slam.launch.py
```

설명:

- `/scan` 기반으로 맵 생성
- SLAM 중에는 `slam_toolbox`가 `map -> odom`을 publish

맵이 충분히 생성되면 다른 터미널에서 저장:

```bash
ros2 run nav2_map_server map_saver_cli -f src/bringup_pkg/maps/scout_mini_map
```

저장 결과:

- `src/bringup_pkg/maps/scout_mini_map.pgm`
- `src/bringup_pkg/maps/scout_mini_map.yaml`

## Nav2

맵 저장 후 SLAM은 종료하고, 저장된 맵을 사용해 Nav2를 실행한다.

```bash
  ros2 launch bringup_pkg scout_cctv_nav2.launch.py

```

설명:

- 저장된 맵 로드
- `map_server`, `amcl`, `navigation` 실행

## RViz2 

1. RViz2 실행
2. `Fixed Frame`을 `map`으로 설정
3. `2D Pose Estimate`로 현재 위치와 방향 지정
4. 위치 추정이 안정되면 `Nav2 Goal` 또는 `2D Goal Pose`로 목표 지정

주의:

- `map -> odom`은 static TF로 직접 고정하지 않는다.
- Nav2 단계에서는 `AMCL`이 `map -> odom`을 publish해야 한다.

## TF tree

정상 동작 기준 TF 구조:

```text
map -> odom -> base_link -> velodyne
```

역할:

- `map -> odom`: SLAM 또는 AMCL
- `odom -> base_link`: scout_base
- `base_link -> velodyne`: static TF
