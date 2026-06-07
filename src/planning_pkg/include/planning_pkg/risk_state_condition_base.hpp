#pragma once

#include <mutex>
#include <string>

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include "planning_pkg/msg/path_risk_state.hpp"

namespace planning_pkg::bt_nodes
{

// 모든 Risk State 기반 조건 노드의 공통 베이스.
// /planning/path_risk_state를 구독하고, 마지막 메시지를 스레드-세이프하게 캐싱한다.
// 파생 클래스는 tick()만 구현하면 된다.
class RiskStateConditionBase : public BT::ConditionNode
{
public:
  RiskStateConditionBase(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts() { return {}; }

protected:
  // tick() 에서 호출: 마지막으로 수신한 상태를 복사해서 반환
  planning_pkg::msg::PathRiskState getLastState() const;

  // 메시지를 아직 한 번도 수신하지 않았으면 false
  bool hasValidState() const;

  rclcpp::Node::SharedPtr node_;

private:
  rclcpp::Subscription<planning_pkg::msg::PathRiskState>::SharedPtr sub_;

  mutable std::mutex mutex_;
  planning_pkg::msg::PathRiskState::SharedPtr last_state_;
};

}  // namespace planning_pkg::bt_nodes
