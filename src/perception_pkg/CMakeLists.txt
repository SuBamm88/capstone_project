cmake_minimum_required(VERSION 3.8)
project(perception_pkg)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/aruco_floor_to_map_calibrator.py
  scripts/aruco_image_homography_node.py
  scripts/floor_to_map_tf_broadcaster_node.py
  scripts/yolo_detector_node.py
  scripts/yolo_to_map_projector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
