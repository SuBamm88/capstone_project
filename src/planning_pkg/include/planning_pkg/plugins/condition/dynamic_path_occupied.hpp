#pragma once
#include "planning_pkg/risk_state_condition_base.hpp"

namespace planning_pkg::bt_nodes
{

// 동적 객체가 현재 경로를 점유하거나 단기 충돌이 예측되는 경우 SUCCESS.
// PathRiskStatePublisher의 dynamic_path_occupied 필드를 읽는다.
// SUCCESS: DynamicPathOccupied 브랜치 활성 → replan 억제, SpeedPlanner 감속
// FAILURE: NormalNavigation 브랜치 활성 → 1Hz replan 재개
class IsDynamicPathOccupiedCondition : public RiskStateConditionBase
{
public:
  IsDynamicPathOccupiedCondition(const std::string & name, const BT::NodeConfiguration & conf);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts() { return {}; }
};

}  // namespace planning_pkg::bt_nodes
