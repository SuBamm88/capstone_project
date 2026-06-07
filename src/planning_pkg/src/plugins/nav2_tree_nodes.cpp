#include <behaviortree_cpp_v3/bt_factory.h>

#include "planning_pkg/plugins/condition/dynamic_path_occupied.hpp"

// bt_navigator가 plugin_lib_names에서 이 라이브러리를 로드할 때 이 함수를 호출함.
// 두 번째 인자의 문자열이 BT XML에서 사용하는 태그명이 된다.
BT_REGISTER_NODES(factory)
{
  using namespace planning_pkg::bt_nodes;
  factory.registerNodeType<IsDynamicPathOccupiedCondition>("IsDynamicPathOccupied");
}
