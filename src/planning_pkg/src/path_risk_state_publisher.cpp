#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <perception_msgs/msg/tracked_object_array.hpp>
#include "planning_pkg/msg/path_risk_state.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace planning_pkg
{

// YOLO에서 나오는 class_id 문자열
constexpr const char * CLASS_PERSON = "person";

class PathRiskStatePublisher : public rclcpp::Node
{
public:
  PathRiskStatePublisher()
  : Node("path_risk_state_publisher"),
    path_was_occupied_(false)
  {
    declare_parameter("tracked_objects_topic", "/perception/tracked_objects");
    declare_parameter("global_path_topic",     "/plan");
    declare_parameter("path_risk_state_topic", "/planning/path_risk_state");

    declare_parameter("lookahead_dist",           3.0);
    declare_parameter("near_path_threshold",      1.5);
    declare_parameter("block_duration_threshold", 5.0);
    declare_parameter("publish_rate",             10.0);

    loadParameters();

    objects_sub_ = create_subscription<perception_msgs::msg::TrackedObjectArray>(
      objects_topic_, 10,
      [this](perception_msgs::msg::TrackedObjectArray::SharedPtr msg) {
        last_objects_ = msg;
      });

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
      [this](nav_msgs::msg::Path::SharedPtr msg) {
        last_path_ = msg;
      });

    risk_pub_ = create_publisher<planning_pkg::msg::PathRiskState>(risk_topic_, 10);

    double rate = get_parameter("publish_rate").as_double();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&PathRiskStatePublisher::evaluate, this));

    RCLCPP_INFO(get_logger(),
      "PathRiskStatePublisher started | objects: %s | path: %s | out: %s",
      objects_topic_.c_str(), path_topic_.c_str(), risk_topic_.c_str());
  }

private:
  // ── 파라미터 로드 ──────────────────────────────────────────────────────
  void loadParameters()
  {
    objects_topic_ = get_parameter("tracked_objects_topic").as_string();
    path_topic_    = get_parameter("global_path_topic").as_string();
    risk_topic_    = get_parameter("path_risk_state_topic").as_string();

    lookahead_dist_           = get_parameter("lookahead_dist").as_double();
    near_path_threshold_      = get_parameter("near_path_threshold").as_double();
    block_duration_threshold_ = get_parameter("block_duration_threshold").as_double();
  }

  // ── 경로 앞쪽 lookahead_dist 범위의 포인트 추출 ────────────────────────
  std::vector<geometry_msgs::msg::Point> extractLookaheadPoints(
    const nav_msgs::msg::Path & path)
  {
    std::vector<geometry_msgs::msg::Point> pts;
    if (path.poses.empty()) return pts;

    pts.push_back(path.poses.front().pose.position);
    double accumulated = 0.0;

    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & prev = path.poses[i - 1].pose.position;
      const auto & curr = path.poses[i].pose.position;
      accumulated += std::hypot(curr.x - prev.x, curr.y - prev.y);
      pts.push_back(curr);
      if (accumulated >= lookahead_dist_) break;
    }
    return pts;
  }

  // ── 점 p에서 선분 ab까지의 최단 거리 ──────────────────────────────────
  double distPointToSegment(
    const geometry_msgs::msg::Point & p,
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b)
  {
    double dx = b.x - a.x, dy = b.y - a.y;
    double len2 = dx * dx + dy * dy;
    if (len2 < 1e-9) {
      return std::hypot(p.x - a.x, p.y - a.y);
    }
    double t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0);
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
  }

  // ── 임의 위치에서 경로까지의 최단 거리 ────────────────────────────────
  double minDistToPath(
    const geometry_msgs::msg::Point & pos,
    const std::vector<geometry_msgs::msg::Point> & pts)
  {
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 1; i < pts.size(); ++i) {
      min_dist = std::min(min_dist, distPointToSegment(pos, pts[i - 1], pts[i]));
    }
    return min_dist;
  }

  // ── 객체가 perception 기준으로 "움직이는 중"인가 ──────────────────────
  // fusion_node가 속도 0.1m/s 이상일 때만 predicted_trajectory를 채워주므로
  // 이 필드의 존재 여부가 곧 perception의 "동적 객체" 판정과 같다.
  bool isDynamicObject(const perception_msgs::msg::TrackedObject & obj)
  {
    return !obj.predicted_trajectory.empty();
  }

  // ── 이 객체에 대해 "기다리는" 것이 합리적인가 ─────────────────────────
  // 사람은 정지해 있어도 다음 순간의 행동을 예측할 수 없으므로 항상 대기 대상.
  // 그 외 객체는 실제로 움직이고 있을 때만 대기 대상(정지 박스는 우회가 합리적).
  bool shouldWaitFor(const perception_msgs::msg::TrackedObject & obj)
  {
    return obj.class_id == CLASS_PERSON || isDynamicObject(obj);
  }

  // ── 예측 궤적이 lookahead 경로 근처를 지나가는지 ──────────────────────
  bool predictsPathCrossing(
    const perception_msgs::msg::TrackedObject & obj,
    const std::vector<geometry_msgs::msg::Point> & path_pts)
  {
    for (const auto & pt : obj.predicted_trajectory) {
      if (minDistToPath(pt, path_pts) < near_path_threshold_) {
        return true;
      }
    }
    return false;
  }

  // ── 메인 평가 루프 (publish_rate Hz) ──────────────────────────────────
  void evaluate()
  {
    auto state = planning_pkg::msg::PathRiskState();
    state.header.stamp    = now();
    state.header.frame_id = "map";
    state.closest_obstacle_dist = -1.0f;

    if (!last_path_ || last_path_->poses.empty() || !last_objects_) {
      state.debug_message = "waiting for /plan or /perception/tracked_objects";
      risk_pub_->publish(state);
      return;
    }

    auto path_pts = extractLookaheadPoints(*last_path_);
    if (path_pts.size() < 2) {
      state.debug_message = "path too short";
      risk_pub_->publish(state);
      return;
    }

    double closest_dist = std::numeric_limits<double>::max();
    bool   affects_path = false;

    for (const auto & obj : last_objects_->objects) {
      geometry_msgs::msg::Point pos;
      pos.x = obj.position.x;
      pos.y = obj.position.y;
      pos.z = 0.0;

      double dist = minDistToPath(pos, path_pts);
      closest_dist = std::min(closest_dist, dist);

      if (!shouldWaitFor(obj)) {
        continue;
      }

      bool near_path = dist < near_path_threshold_;
      bool predicts_crossing = isDynamicObject(obj) && predictsPathCrossing(obj, path_pts);

      if (near_path || predicts_crossing) {
        affects_path = true;
      }
    }

    if (closest_dist < std::numeric_limits<double>::max()) {
      state.closest_obstacle_dist = static_cast<float>(closest_dist);
    }

    // ── 경로 점유 지속 시간 추적 ────────────────────────────────────────
    double occupancy_sec = 0.0;
    if (affects_path) {
      if (!path_was_occupied_) {
        occupancy_start_time_ = now();
        path_was_occupied_ = true;
      }
      occupancy_sec = (now() - occupancy_start_time_).seconds();
    } else {
      path_was_occupied_ = false;
    }
    state.path_occupancy_duration = static_cast<float>(occupancy_sec);

    // ── BT dynamic_path_occupied 플래그 ─────────────────────────────────
    // 점유 중이면서 아직 block_duration_threshold를 넘지 않았을 때만 "대기 가치 있음"
    state.dynamic_path_occupied = affects_path && occupancy_sec < block_duration_threshold_;

    if (state.dynamic_path_occupied) {
      state.debug_message = "WAIT: dynamic object affecting path (" +
        std::to_string(occupancy_sec) + "s)";
    } else if (affects_path) {
      state.debug_message = "REPLAN: path occupied too long (" +
        std::to_string(occupancy_sec) + "s), rerouting";
    } else {
      state.debug_message = "OK";
    }

    risk_pub_->publish(state);
  }

  // ── 멤버 변수 ──────────────────────────────────────────────────────────
  rclcpp::Subscription<perception_msgs::msg::TrackedObjectArray>::SharedPtr objects_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<planning_pkg::msg::PathRiskState>::SharedPtr risk_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  perception_msgs::msg::TrackedObjectArray::SharedPtr last_objects_;
  nav_msgs::msg::Path::SharedPtr last_path_;

  bool path_was_occupied_;
  rclcpp::Time occupancy_start_time_;

  std::string objects_topic_, path_topic_, risk_topic_;
  double lookahead_dist_, near_path_threshold_;
  double block_duration_threshold_;
};

}  // namespace planning_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<planning_pkg::PathRiskStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
