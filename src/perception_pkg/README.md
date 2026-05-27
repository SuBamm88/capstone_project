# perception_pkg

CCTV 이미지 위 YOLO bbox를 ArUco 기반 캘리브레이션 상수로 `map` 좌표에 투영하는
ROS 2 Humble 패키지입니다. 런타임 입력은 `vision_msgs/msg/Detection2DArray`이고,
출력은 `geometry_msgs/msg/PoseArray`입니다.

## Build

워크스페이스 루트에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select perception_pkg --symlink-install
source install/setup.bash
```

`yolo_detector_node`는 Python `ultralytics` 패키지를 사용합니다. 환경에 없다면
워크스페이스 빌드 전에 설치해야 합니다.

## Calibration

### 1. image -> floor Homography

`config/aruco_image_homography.yaml`에서 CCTV 이미지 경로, ArUco ID, 각 ID의
floor 좌표를 수정합니다. 기본 좌표는 0.4 m x 0.3 m 사각형입니다.

```bash
ros2 launch perception_pkg calibrate_image_to_floor.launch.py
```

성공하면 `config/image_to_floor.yaml`에 `image_to_floor_homography`가 저장됩니다.

### 2. floor -> map TF

`config/floor_to_map_calibrator.yaml`에서 같은 ArUco ID의 floor 좌표와 SLAM map에서
측정한 map 좌표를 대응시킵니다. 이 파일은 ArUco를 이미지에서 다시 검출하지 않고,
마커 ID를 기준으로 두 좌표계의 점 대응을 계산합니다.

```bash
ros2 launch perception_pkg calibrate_floor_to_map.launch.py
```

성공하면 `config/floor_to_map_tf.yaml`에 `map -> floor` static TF 값이 저장됩니다.

카메라, 마커, SLAM map이 모두 고정이면 이 값은 다시 구할 필요가 없습니다. 카메라가
움직이거나, 마커가 이동하거나, SLAM map을 새로 만들면 다시 캘리브레이션해야 합니다.

## Run

USB camera input can be started from the vendored `usb_cam` package:

```bash
ros2 launch usb_cam camera.launch.py
```

`cctv_to_map_pipeline.launch.py`는 `yolo_detector_node`,
`floor_to_map_tf_broadcaster_node`, `yolo_to_map_projector_node`를 함께 실행합니다.
YOLO 모델은 기본적으로 `perception_pkg/resource/best.pt`를 사용하며, 경로는
`config/yolo_detector.yaml`의 `model_path`에서 수정합니다.

```bash
ros2 launch perception_pkg cctv_to_map_pipeline.launch.py
```

## Topics

| Topic | Type | Direction |
| --- | --- | --- |
| `/yolo/detections` | `vision_msgs/msg/Detection2DArray` | `yolo_to_map_projector_node` input |
| `/cctv/objects_floor` | `geometry_msgs/msg/PoseArray` | debug floor output |
| `/cctv/objects_map` | `geometry_msgs/msg/PoseArray` | final map output |
| `/cctv/topview_image` | `sensor_msgs/msg/Image` | image homography debug output |

`yolo_to_map_projector_node`는 bbox 바닥 중앙점을 계산합니다. 기본값은
`vision_msgs` 표준처럼 bbox `center.position.x/y`가 중심이라고 보고
`u = center.x`, `v = center.y + size_y / 2`를 사용합니다. YOLO 쪽에서 x/y를
좌상단으로 넣고 있다면 `config/yolo_to_map_projector.yaml`의
`bbox_xy_origin`을 `"top_left"`로 바꾸면 `u = x + width / 2`, `v = y + height`를
사용합니다.

## Parameters

기본 파라미터는 `config/` 아래 YAML 파일에서 수정합니다.

- `aruco_image_homography.yaml`: ArUco 이미지 검출과 image -> floor H 저장
- `image_to_floor.yaml`: 런타임 image -> floor Homography 상수
- `floor_to_map_calibrator.yaml`: ArUco ID별 floor/map 좌표 대응
- `floor_to_map_tf.yaml`: 런타임 `map -> floor` static TF 상수
- `yolo_detector.yaml`: YOLO 모델, 입력 이미지 토픽, detection 출력 토픽
- `yolo_to_map_projector.yaml`: YOLO 입력, class filter, 출력 토픽
