cmake_minimum_required(VERSION 3.8)
project(planning_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 의존성 ─────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── 메시지 생성 ────────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PathRiskState.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# 메시지 타입 지원 타겟 (다른 타겟에서 planning_pkg 메시지를 쓸 수 있게 연결)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ── PathRiskStatePublisher 실행 파일 ───────────────────────────────────────
add_executable(path_risk_state_publisher
  src/path_risk_state_publisher.cpp
)
target_include_directories(path_risk_state_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(path_risk_state_publisher
  rclcpp std_msgs nav_msgs geometry_msgs perception_msgs
)
target_link_libraries(path_risk_state_publisher "${cpp_typesupport_target}")

# ── BT Condition 플러그인 공유 라이브러리 ─────────────────────────────────
# bt_navigator가 plugin_lib_names에서 이 라이브러리 이름(planning_bt_nodes)을 로드함
add_library(planning_bt_nodes SHARED
  src/risk_state_condition_base.cpp
  src/plugins/condition/dynamic_path_occupied.cpp
  src/plugins/nav2_tree_nodes.cpp
)
target_include_directories(planning_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(planning_bt_nodes
  rclcpp behaviortree_cpp_v3 pluginlib
)
target_link_libraries(planning_bt_nodes "${cpp_typesupport_target}")

# ── 설치 ──────────────────────────────────────────────────────────────────
install(TARGETS path_risk_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS planning_bt_nodes
  DESTINATION lib  # bt_navigator가 lib/ 아래에서 플러그인을 탐색함
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
