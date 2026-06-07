# bringup_pkg

캡스톤 프로젝트의 상위 소프트웨어 스택(SLAM, Nav2)을 위한 launch 및 설정 패키지다.

`scout_base`, `scout_description`, `velodyne_*` 등 로봇 하드웨어 및 저수준 드라이버 패키지는 포함하지 않으며 별도로 실행해야 한다.

## Directory Structure

* `launch/` — SLAM / Nav2 실행 파일
* `config/` — SLAM / Nav2 설정 파일
* `rviz/` — RViz 설정
* `maps/` — 저장된 지도

## Dependencies

본 패키지는 다음 패키지들과 함께 사용된다.

* `planning_pkg`
* `cctv_costmap_layer`
* `nav2_bringup`
* `slam_toolbox`

---

## Mode: Baseline vs Proposed

본 캡스톤은 **표준 Nav2(Baseline)** 와 **CCTV Perception + Custom BT(Proposed)** 를 비교하는 실험 구조로 설계되었다.

실험의 공정성(fair comparison)을 위해 주행 성능에 영향을 주는 주요 Nav2 파라미터는 동일하게 유지하였다.

두 설정의 차이는 다음 두 가지뿐이다.

* CCTV perception 정보 사용 여부
* Custom Behavior Tree 적용 여부

| Mode     | Launch                      | Params                         | Description                 |
| -------- | --------------------------- | ------------------------------ | --------------------------- |
| Mapping  | `scout_slam.launch.py`      | `config/slam_toolbox.yaml`     | 지도 생성                       |
| Baseline | `scout_nav2.launch.py`      | `config/nav2_params.yaml`      | 표준 Nav2 사용                  |
| Proposed | `scout_cctv_nav2.launch.py` | `config/nav2_params_cctv.yaml` | CCTV Costmap + Custom BT 사용 |

### Run

```bash
# Mapping
ros2 launch bringup_pkg scout_slam.launch.py

# Baseline
ros2 launch bringup_pkg scout_nav2.launch.py

# Proposed
ros2 launch bringup_pkg scout_cctv_nav2.launch.py
```

> **Note**
>
> `robot_radius`, `max_speed_xy`, `max_velocity`, `use_astar`,
> `trans_stopped_velocity` 등 주행 관련 파라미터는 Baseline과 Proposed에서 동일하게 유지하였다.

---

## Proposed Mode Architecture

`scout_cctv_nav2.launch.py`는 다음 구성 요소를 함께 실행한다.

### CCTV Costmap Layer

`nav2_params_cctv.yaml`을 통해 `cctv_layer`가 활성화된다.

### Custom Behavior Tree

`default_nav_to_pose_bt_xml`을 런타임에

```text
planning_pkg/behavior_trees/navigate_cctv_risk.xml
```

로 치환하여 사용한다.

### Path Risk State Publisher

`planning_pkg`의 `path_risk_state_publisher` 노드를 실행한다.

입력:

```text
/perception/tracked_objects
/plan
```

출력:

```text
/planning/path_risk_state
```

BT는 `/planning/path_risk_state`의 `dynamic_path_occupied` 값을 읽어 경로 유지(wait) 또는 일반 Navigation 수행 여부를 결정한다.

자세한 BT 구조는 Custom Behavior Tree 파일의 주석을 참고한다.

## Debug Topics

```bash
ros2 topic echo /planning/path_risk_state
ros2 topic echo /plan
ros2 topic echo /perception/tracked_objects
```

---

## Build

```bash
colcon build --symlink-install \
  --packages-select \
  planning_pkg \
  bringup_pkg \
  cctv_costmap_layer

source install/setup.bash
```