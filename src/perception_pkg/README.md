# perception_pkg

ROS 2 Humble package for running YOLO detection on CCTV camera images and
projecting detections onto a SLAM map image with a homography.

The detector loads `resource/best.pt` from the installed `perception_pkg` share
directory by default. The homography node loads
`resource/scout_mini_map_3.pgm` the same way, so runtime paths do not depend on
a user-specific home directory.

## Build

Run from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select perception_pkg --symlink-install
source install/setup.bash
```

The node requires the Python `ultralytics` package at runtime.

## Run

```bash
ros2 launch perception_pkg cctv_perception.launch.py
```

This starts:

| Node | Input | Output |
| --- | --- | --- |
| `yolo_detector_node` | `/camera2/image_raw` | `/yolo/detections`, `/yolo/annotated_image` |
| `cam_to_map` | `/yolo/detections` | `/cctv/slam_map` |

The logical order is camera image -> YOLO -> homography projection. In launch
they can start together because `cam_to_map` simply waits until YOLO
detections arrive.

## Parameters

Default parameters are in `config/yolo_detector.yaml`.

| Parameter | Default |
| --- | --- |
| `model_path` | `resource/best.pt` |
| `image_topic` | `/camera2/image_raw` |
| `annotated_topic` | `/yolo/annotated_image` |
| `detections_topic` | `/yolo/detections` |
| `confidence` | `0.5` |
| `publish_annotated` | `true` |

## Topics

| Topic | Type | Direction |
| --- | --- | --- |
| `/camera2/image_raw` | `sensor_msgs/msg/Image` | input |
| `/yolo/detections` | `vision_msgs/msg/Detection2DArray` | output |
| `/yolo/annotated_image` | `sensor_msgs/msg/Image` | output |
| `/cctv/slam_map` | `sensor_msgs/msg/Image` | output |

## Imported Homography Code

The original `homography_pkg` contained several executables:

| Original executable | Role |
| --- | --- |
| `cam_to_map` | Runtime node used here. It subscribes to YOLO detections, projects bbox bottom centers to the SLAM map image, and publishes an annotated map. |
| `cctv_projection` | Similar projection flow, but treats the homography target as metric map coordinates before converting to map-image pixels. Useful if the point pairs are maintained in meters. |
| `bev` | Debug/visualization node that warps the camera image into a bird's-eye-view image using four hand-picked points. It uses OpenCV GUI windows. |
| `want_to_know_pixel` | Debug helper for clicking on a PGM map image and reading pixel coordinates/values. It also uses OpenCV GUI windows. |

Only the production path, YOLO plus `cam_to_map`, is started by
`cctv_perception.launch.py`. The GUI/debug helpers are not required for normal
runtime.
