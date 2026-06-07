#include "planning_pkg/risk_state_condition_base.hpp"

namespace planning_pkg::bt_nodes
{

RiskStateConditionBase::RiskStateConditionBase(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
  // bt_navigator가 blackboard에 자신의 rclcpp::Node를 "node" 키로 넣어둠
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  sub_ = node_->create_subscription<planning_pkg::msg::PathRiskState>(
    "/planning/path_risk_state",
    rclcpp::QoS(rclcpp::KeepLast(1)),
    [this](planning_pkg::msg::PathRiskState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      last_state_ = msg;
    });
}

planning_pkg::msg::PathRiskState RiskStateConditionBase::getLastState() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!last_state_) {
    return planning_pkg::msg::PathRiskState{};
  }
  return *last_state_;
}

bool RiskStateConditionBase::hasValidState() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_state_ != nullptr;
}

}  // namespace planning_pkg::bt_nodes
