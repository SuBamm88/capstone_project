#include "planning_pkg/plugins/condition/dynamic_path_occupied.hpp"

namespace planning_pkg::bt_nodes
{

IsDynamicPathOccupiedCondition::IsDynamicPathOccupiedCondition(
  const std::string & name, const BT::NodeConfiguration & conf)
: RiskStateConditionBase(name, conf) {}

BT::NodeStatus IsDynamicPathOccupiedCondition::tick()
{
  if (!hasValidState()) {
    return BT::NodeStatus::FAILURE;
  }
  return getLastState().dynamic_path_occupied
    ? BT::NodeStatus::SUCCESS
    : BT::NodeStatus::FAILURE;
}

}  // namespace planning_pkg::bt_nodes
