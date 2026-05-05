# perception_pkg

CCTV 인지 흐름 중 3번, 5번 노드를 포함한 ROS 2 Humble C++ 패키지입니다.
커스텀 메시지는 사용하지 않고 표준 메시지 타입으로 토픽을 연결합니다.

## Build

워크스페이스 루트에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select perception_pkg
source install/setup.bash
```

## Run

두 노드를 함께 실행합니다.

```bash
ros2 launch perception_pkg cctv_projection_pipeline.launch.py
```

노드를 따로 실행할 수도 있습니다.

```bash
ros2 run perception_pkg ground_contact_extractor_node
ros2 run perception_pkg floor_to_map_node
```

## Topics

| Topic | Type | Direction |
| --- | --- | --- |
| `/yolo/detections` | `vision_msgs/msg/Detection2DArray` | `ground_contact_extractor_node` input |
| `/cctv/footpoints_pixel` | `geometry_msgs/msg/PoseArray` | `ground_contact_extractor_node` output |
| `/cctv/objects_floor` | `geometry_msgs/msg/PoseArray` | `floor_to_map_node` input |
| `/cctv/objects_map` | `geometry_msgs/msg/PoseArray` | `floor_to_map_node` output |

`/cctv/footpoints_pixel`은 `PoseArray`의 `position.x`를 pixel `u`,
`position.y`를 pixel `v`로 사용합니다.

## Parameters

기본 파라미터는 `config/` 아래 YAML 파일에서 수정합니다.

- `ground_contact_extractor.yaml`: 입력/출력 토픽, 대상 class, 최소 confidence
- `floor_to_map.yaml`: 입력/출력 토픽, map frame, `tx`, `ty`, `yaw`, homography

현재 4번 `image_to_floor_node`는 아직 없으므로 `/cctv/objects_floor`는 다른 노드가
발행해야 합니다.
