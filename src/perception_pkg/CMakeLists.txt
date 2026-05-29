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

install(DIRECTORY
  config
  launch
  resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
