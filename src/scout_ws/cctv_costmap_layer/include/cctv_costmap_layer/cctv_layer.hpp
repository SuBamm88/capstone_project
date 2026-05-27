#ifndef CCTV_COSTMAP_LAYER__CCTV_LAYER_HPP_
#define CCTV_COSTMAP_LAYER__CCTV_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cctv_costmap_layer
{

class CctvLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  CctvLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override;

private:
  struct ObservedPose
  {
    geometry_msgs::msg::PoseStamped pose;
    rclcpp::Time received_time;
  };

  void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  bool transformPose(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output);
  void markObject(double x, double y, double radius);
  void includeObjectBounds(
    double x, double y, double radius,
    double * min_x, double * min_y, double * max_x, double * max_y);

  std::mutex mutex_;
  std::vector<ObservedPose> poses_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;

  std::string topic_{"/cctv/objects_map"};
  double default_radius_{0.25};
  double max_observation_age_{1.0};
  double transform_tolerance_{0.2};

  bool has_last_bounds_{false};
  double last_min_x_{0.0};
  double last_min_y_{0.0};
  double last_max_x_{0.0};
  double last_max_y_{0.0};
};

}  // namespace cctv_costmap_layer

#endif  // CCTV_COSTMAP_LAYER__CCTV_LAYER_HPP_
