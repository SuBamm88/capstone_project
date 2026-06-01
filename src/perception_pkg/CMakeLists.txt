cmake_minimum_required(VERSION 3.8)
project(perception_pkg)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/cam_to_map_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cam_to_map
)

install(PROGRAMS
  scripts/yolo_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_detector_node
)

install(PROGRAMS
  scripts/cctv_pose_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cctv_pose_node
)

install(PROGRAMS
  scripts/cctv_to_map_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cctv_to_map
)

install(PROGRAMS
  scripts/pixel_picker_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pixel_picker_node
)

install(PROGRAMS
  scripts/cctv_fov_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cctv_fov
)

install(PROGRAMS
  scripts/fusion_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fusion
)

install(PROGRAMS
  scripts/lidar_object_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME lidar_object
)

install(DIRECTORY
  config
  launch
  resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
