cmake_minimum_required(VERSION 3.8)
project(perception_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)

add_executable(ground_contact_extractor_node
  src/ground_contact_extractor_node.cpp
)
ament_target_dependencies(ground_contact_extractor_node
  geometry_msgs
  rclcpp
  vision_msgs
)

add_executable(floor_to_map_node
  src/floor_to_map_node.cpp
)
ament_target_dependencies(floor_to_map_node
  geometry_msgs
  rclcpp
)

install(TARGETS
  ground_contact_extractor_node
  floor_to_map_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
