#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"

class FloorToMapNode : public rclcpp::Node
{
public:
  FloorToMapNode()
  : Node("floor_to_map_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cctv/objects_floor");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cctv/objects_map");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    tx_ = declare_parameter<double>("tx", 0.0);
    ty_ = declare_parameter<double>("ty", 0.0);
    yaw_ = declare_parameter<double>("yaw", 0.0);
    use_homography_ = declare_parameter<bool>("use_homography", false);
    homography_ = parseHomography(
      declare_parameter<std::vector<double>>(
        "homography",
        {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}));

    publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(output_topic_, 10);
    subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      input_topic_,
      10,
      std::bind(&FloorToMapNode::onObjectsFloor, this, std::placeholders::_1));
  }

private:
  void onObjectsFloor(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    // PoseArray is enough here because this node only transforms coordinates.
    geometry_msgs::msg::PoseArray output;
    output.header = msg->header;
    output.header.frame_id = map_frame_;

    for (const auto & pose : msg->poses) {
      geometry_msgs::msg::Pose mapped = pose;
      const auto [x_map, y_map] = transformXY(pose.position.x, pose.position.y);
      mapped.position.x = x_map;
      mapped.position.y = y_map;
      if (!use_homography_) {
        mapped.orientation = rotateYaw(pose.orientation, yaw_);
      }
      output.poses.push_back(mapped);
    }

    publisher_->publish(output);
  }

  std::pair<double, double> transformXY(double x_floor, double y_floor) const
  {
    if (use_homography_) {
      return transformHomography(x_floor, y_floor);
    }

    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    const double x_map = cos_yaw * x_floor - sin_yaw * y_floor + tx_;
    const double y_map = sin_yaw * x_floor + cos_yaw * y_floor + ty_;
    return {x_map, y_map};
  }

  std::pair<double, double> transformHomography(double x_floor, double y_floor) const
  {
    const double x_map =
      homography_[0] * x_floor + homography_[1] * y_floor + homography_[2];
    const double y_map =
      homography_[3] * x_floor + homography_[4] * y_floor + homography_[5];
    const double w_map =
      homography_[6] * x_floor + homography_[7] * y_floor + homography_[8];

    if (std::abs(w_map) < 1.0e-12) {
      return {x_map, y_map};
    }
    return {x_map / w_map, y_map / w_map};
  }

  std::vector<double> parseHomography(const std::vector<double> & values) const
  {
    if (values.size() == 9) {
      return values;
    }

    RCLCPP_WARN(get_logger(), "homography must contain 9 values; using identity");
    return {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  }

  geometry_msgs::msg::Quaternion rotateYaw(
    const geometry_msgs::msg::Quaternion & orientation,
    double yaw_delta) const
  {
    const double yaw = yawFromQuaternion(orientation) + yaw_delta;
    geometry_msgs::msg::Quaternion rotated;
    rotated.z = std::sin(yaw / 2.0);
    rotated.w = std::cos(yaw / 2.0);
    return rotated;
  }

  double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q) const
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string map_frame_;
  double tx_;
  double ty_;
  double yaw_;
  bool use_homography_;
  std::vector<double> homography_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FloorToMapNode>());
  rclcpp::shutdown();
  return 0;
}
