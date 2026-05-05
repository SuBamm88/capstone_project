#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

class GroundContactExtractorNode : public rclcpp::Node
{
public:
  GroundContactExtractorNode()
  : Node("ground_contact_extractor_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/yolo/detections");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/cctv/footpoints_pixel");
    target_classes_ =
      declare_parameter<std::vector<std::string>>("target_classes", {"person"});
    min_confidence_ = declare_parameter<double>("min_confidence", 0.4);
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "");

    publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(output_topic_, 10);
    subscription_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      input_topic_,
      10,
      std::bind(&GroundContactExtractorNode::onDetections, this, std::placeholders::_1));
  }

private:
  void onDetections(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    // PoseArray keeps this node on standard ROS messages; x/y carry pixel u/v.
    geometry_msgs::msg::PoseArray output;
    output.header = msg->header;
    if (!output_frame_id_.empty()) {
      output.header.frame_id = output_frame_id_;
    }

    for (const auto & detection : msg->detections) {
      const auto [class_id, score] = bestHypothesis(detection.results);
      if (!target_classes_.empty() && !containsTargetClass(class_id)) {
        continue;
      }
      if (score < min_confidence_) {
        continue;
      }

      const auto & bbox = detection.bbox;
      if (bbox.size_x <= 0.0 || bbox.size_y <= 0.0) {
        continue;
      }

      geometry_msgs::msg::Pose pose;
      pose.position.x = bbox.center.position.x;
      pose.position.y = bbox.center.position.y + bbox.size_y / 2.0;
      pose.orientation.w = 1.0;
      output.poses.push_back(pose);
    }

    publisher_->publish(output);
  }

  std::pair<std::string, double> bestHypothesis(
    const std::vector<vision_msgs::msg::ObjectHypothesisWithPose> & results) const
  {
    std::string best_class_id;
    double best_score = -1.0;

    for (const auto & result : results) {
      if (result.hypothesis.score > best_score) {
        best_class_id = result.hypothesis.class_id;
        best_score = result.hypothesis.score;
      }
    }

    if (best_score < 0.0) {
      return {"", 0.0};
    }
    return {best_class_id, best_score};
  }

  bool containsTargetClass(const std::string & class_id) const
  {
    return std::find(target_classes_.begin(), target_classes_.end(), class_id) !=
           target_classes_.end();
  }

  std::string input_topic_;
  std::string output_topic_;
  std::vector<std::string> target_classes_;
  double min_confidence_;
  std::string output_frame_id_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundContactExtractorNode>());
  rclcpp::shutdown();
  return 0;
}
