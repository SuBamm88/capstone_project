#include "cctv_costmap_layer/cctv_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace cctv_costmap_layer
{

void CctvLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock parent lifecycle node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(topic_));
  declareParameter("default_radius", rclcpp::ParameterValue(default_radius_));
  declareParameter("max_observation_age", rclcpp::ParameterValue(max_observation_age_));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(transform_tolerance_));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".default_radius", default_radius_);
  node->get_parameter(name_ + ".max_observation_age", max_observation_age_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);

  matchSize();
  setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
  resetMaps();

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&CctvLayer::poseArrayCallback, this, std::placeholders::_1),
    options);

  current_ = true;
  RCLCPP_INFO(
    logger_,
    "CCTV costmap layer '%s' subscribed to '%s' with radius %.2f m",
    name_.c_str(), topic_.c_str(), default_radius_);
}

void CctvLayer::poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  const rclcpp::Time received_time = node->now();
  std::vector<ObservedPose> new_poses;
  new_poses.reserve(msg->poses.size());

  std::string frame_id = msg->header.frame_id;
  if (frame_id.empty()) {
    frame_id = layered_costmap_->getGlobalFrameID();
  }

  builtin_interfaces::msg::Time stamp = msg->header.stamp;
  if (rclcpp::Time(stamp).nanoseconds() == 0) {
    stamp = received_time;
  }

  for (const auto & pose : msg->poses) {
    ObservedPose observed;
    observed.pose.header.frame_id = frame_id;
    observed.pose.header.stamp = stamp;
    observed.pose.pose = pose;
    observed.received_time = received_time;
    new_poses.push_back(observed);
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    poses_ = std::move(new_poses);
  }

  current_ = true;
}

bool CctvLayer::transformPose(
  const geometry_msgs::msg::PoseStamped & input,
  geometry_msgs::msg::PoseStamped & output)
{
  const std::string target_frame = layered_costmap_->getGlobalFrameID();
  if (input.header.frame_id == target_frame) {
    output = input;
    return true;
  }

  try {
    output = tf_->transform(
      input, target_frame,
      tf2::durationFromSec(transform_tolerance_));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "Failed to transform CCTV pose from '%s' to '%s': %s",
      input.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

void CctvLayer::includeObjectBounds(
  double x, double y, double radius,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  touch(x - radius, y - radius, min_x, min_y, max_x, max_y);
  touch(x + radius, y + radius, min_x, min_y, max_x, max_y);
}

void CctvLayer::markObject(double x, double y, double radius)
{
  if (radius <= 0.0) {
    return;
  }

  int min_i;
  int min_j;
  int max_i;
  int max_j;
  worldToMapEnforceBounds(x - radius, y - radius, min_i, min_j);
  worldToMapEnforceBounds(x + radius, y + radius, max_i, max_j);

  if (min_i > max_i) {
    std::swap(min_i, max_i);
  }
  if (min_j > max_j) {
    std::swap(min_j, max_j);
  }

  for (int j = min_j; j <= max_j; ++j) {
    for (int i = min_i; i <= max_i; ++i) {
      double wx;
      double wy;
      mapToWorld(static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);
      if (std::hypot(wx - x, wy - y) <= radius) {
        setCost(
          static_cast<unsigned int>(i),
          static_cast<unsigned int>(j),
          nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

void CctvLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    current_ = false;
    return;
  }

  resetMaps();

  const rclcpp::Time now = node->now();
  std::vector<ObservedPose> poses_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    poses_copy = poses_;
  }

  double active_min_x = std::numeric_limits<double>::max();
  double active_min_y = std::numeric_limits<double>::max();
  double active_max_x = std::numeric_limits<double>::lowest();
  double active_max_y = std::numeric_limits<double>::lowest();
  bool has_active_bounds = false;
  bool all_transforms_ok = true;

  for (const auto & observed : poses_copy) {
    if (
      max_observation_age_ > 0.0 &&
      (now - observed.received_time).seconds() > max_observation_age_)
    {
      continue;
    }

    geometry_msgs::msg::PoseStamped transformed;
    if (!transformPose(observed.pose, transformed)) {
      all_transforms_ok = false;
      continue;
    }

    const double x = transformed.pose.position.x;
    const double y = transformed.pose.position.y;
    markObject(x, y, default_radius_);
    includeObjectBounds(x, y, default_radius_, min_x, min_y, max_x, max_y);
    includeObjectBounds(
      x, y, default_radius_, &active_min_x, &active_min_y, &active_max_x, &active_max_y);
    has_active_bounds = true;
  }

  if (has_last_bounds_) {
    touch(last_min_x_, last_min_y_, min_x, min_y, max_x, max_y);
    touch(last_max_x_, last_max_y_, min_x, min_y, max_x, max_y);
  }

  has_last_bounds_ = has_active_bounds;
  if (has_active_bounds) {
    last_min_x_ = active_min_x;
    last_min_y_ = active_min_y;
    last_max_x_ = active_max_x;
    last_max_y_ = active_max_y;
  }

  current_ = all_transforms_ok;
}

void CctvLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void CctvLayer::reset()
{
  resetMaps();
  current_ = true;
}

bool CctvLayer::isClearable()
{
  return false;
}

}  // namespace cctv_costmap_layer

PLUGINLIB_EXPORT_CLASS(cctv_costmap_layer::CctvLayer, nav2_costmap_2d::Layer)
