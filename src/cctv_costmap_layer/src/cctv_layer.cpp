#include "cctv_costmap_layer/cctv_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
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
  declareParameter("base_radius", rclcpp::ParameterValue(base_radius_));
  declareParameter("inflation_radius", rclcpp::ParameterValue(inflation_radius_));
  declareParameter("cost_scaling_factor", rclcpp::ParameterValue(cost_scaling_factor_));
  declareParameter("prediction_decay", rclcpp::ParameterValue(prediction_decay_));
  declareParameter("min_stability", rclcpp::ParameterValue(min_stability_));
  declareParameter("max_observation_age", rclcpp::ParameterValue(max_observation_age_));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(transform_tolerance_));
  declareParameter("teardrop_enabled", rclcpp::ParameterValue(teardrop_enabled_));
  declareParameter("sigma_front", rclcpp::ParameterValue(sigma_front_));
  declareParameter("sigma_side", rclcpp::ParameterValue(sigma_side_));
  declareParameter("sigma_back", rclcpp::ParameterValue(sigma_back_));
  declareParameter("static_speed", rclcpp::ParameterValue(static_speed_));
  declareParameter("moving_speed", rclcpp::ParameterValue(moving_speed_));
  declareParameter("block_behind_walls", rclcpp::ParameterValue(block_behind_walls_));
  declareParameter("wall_cost_thresh", rclcpp::ParameterValue(wall_cost_thresh_));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".base_radius", base_radius_);
  node->get_parameter(name_ + ".inflation_radius", inflation_radius_);
  node->get_parameter(name_ + ".cost_scaling_factor", cost_scaling_factor_);
  node->get_parameter(name_ + ".prediction_decay", prediction_decay_);
  node->get_parameter(name_ + ".min_stability", min_stability_);
  node->get_parameter(name_ + ".max_observation_age", max_observation_age_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(name_ + ".teardrop_enabled", teardrop_enabled_);
  node->get_parameter(name_ + ".sigma_front", sigma_front_);
  node->get_parameter(name_ + ".sigma_side", sigma_side_);
  node->get_parameter(name_ + ".sigma_back", sigma_back_);
  node->get_parameter(name_ + ".static_speed", static_speed_);
  node->get_parameter(name_ + ".moving_speed", moving_speed_);
  node->get_parameter(name_ + ".block_behind_walls", block_behind_walls_);
  node->get_parameter(name_ + ".wall_cost_thresh", wall_cost_thresh_);

  loadClassParams();

  matchSize();
  setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
  resetMaps();

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  tracks_sub_ = node->create_subscription<perception_msgs::msg::TrackedTrackArray>(
    topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&CctvLayer::tracksCallback, this, std::placeholders::_1),
    options);

  current_ = true;
  RCLCPP_INFO(
    logger_,
    "CCTV costmap layer '%s' subscribed to '%s' "
    "(base_radius %.2f m, inflation_radius %.2f m, %zu class overrides)",
    name_.c_str(), topic_.c_str(), base_radius_, inflation_radius_,
    class_params_.size());
}

void CctvLayer::loadClassParams()
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  // class_names를 선언/조회. 각 class에 대해 "<class>.<key>" 평탄 파라미터를 읽어
  // class_params_에 채운다. 누락 key는 레이어 전역 기본값으로 fallback.
  declareParameter("class_names", rclcpp::ParameterValue(std::vector<std::string>{}));
  std::vector<std::string> class_names;
  node->get_parameter(name_ + ".class_names", class_names);

  for (const auto & cls : class_names) {
    ClassParams cp{base_radius_, inflation_radius_, cost_scaling_factor_};

    const std::string base = cls + ".core_radius";
    const std::string infl = cls + ".inflation_radius";
    const std::string scal = cls + ".cost_scaling_factor";
    declareParameter(base, rclcpp::ParameterValue(cp.core_radius));
    declareParameter(infl, rclcpp::ParameterValue(cp.inflation_radius));
    declareParameter(scal, rclcpp::ParameterValue(cp.cost_scaling_factor));
    node->get_parameter(name_ + "." + base, cp.core_radius);
    node->get_parameter(name_ + "." + infl, cp.inflation_radius);
    node->get_parameter(name_ + "." + scal, cp.cost_scaling_factor);

    class_params_[cls] = cp;
    RCLCPP_INFO(
      logger_, "  class '%s': core=%.2f inflation=%.2f scale=%.2f",
      cls.c_str(), cp.core_radius, cp.inflation_radius, cp.cost_scaling_factor);
  }
}

void CctvLayer::tracksCallback(
  const perception_msgs::msg::TrackedTrackArray::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  // 트랙은 보통 map frame. 위험 점은 원본 좌표 그대로 저장하고, costmap 전역
  // frame(local=odom일 수 있음)으로의 변환은 updateBounds 시점에 최신 tf로 수행.
  std::string src_frame = msg->header.frame_id;
  if (src_frame.empty()) {
    src_frame = layered_costmap_->getGlobalFrameID();
  }

  std::vector<RiskPoint> new_points;
  std::vector<PredPoint> new_pred;

  for (const auto & obj : msg->tracks) {
    if (obj.tracking_stability < min_stability_) {
      continue;
    }

    // class별 override 조회 (없으면 레이어 전역 기본값).
    double cls_base_radius = base_radius_;
    double cls_inflation = inflation_radius_;
    double cls_cost_scale = cost_scaling_factor_;
    auto it = class_params_.find(obj.class_id);
    if (it != class_params_.end()) {
      cls_base_radius = it->second.core_radius;
      cls_inflation = it->second.inflation_radius;
      cls_cost_scale = it->second.cost_scaling_factor;
    }

    // footprint_area로부터 코어 반경 추정 (없으면 class/전역 기본 반경)
    double core_radius = cls_base_radius;
    if (obj.footprint_area > 0.0) {
      core_radius = std::max(cls_base_radius, std::sqrt(obj.footprint_area) * 0.5);
    }

    // 안정도가 낮을수록 코어 비용을 낮춘다 (오탐 보호).
    const double stability = std::clamp<double>(obj.tracking_stability, 0.0, 1.0);
    const double core_cost =
      static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE) * stability;

    // 속도 블렌딩 가중: w_motion=0(정지)이면 물방울이, 1(이동)이면 궤적이 지배.
    // 경계에서 코스트가 툭 바뀌지 않도록 정지/이동 사이를 선형 보간한다.
    const double speed = std::hypot(obj.velocity.x, obj.velocity.y);
    double w_motion = 0.0;
    if (moving_speed_ > static_speed_) {
      w_motion = std::clamp<double>(
        (speed - static_speed_) / (moving_speed_ - static_speed_), 0.0, 1.0);
    } else {
      w_motion = (speed >= moving_speed_) ? 1.0 : 0.0;
    }

    // 1) 현재 위치 코어.
    //    응시방향(facing)이 있으면 (1-w_motion) 가중으로 물방울(비대칭)을,
    //    동시에 w_motion 가중으로 등방 원형을 함께 둔다. max 합성이므로 둘이
    //    겹쳐도 큰 쪽이 남는다. facing 없으면 기존처럼 등방 원형만.
    const bool use_teardrop =
      teardrop_enabled_ && obj.has_facing && (w_motion < 1.0);
    if (use_teardrop) {
      const double td_cost = core_cost * (1.0 - w_motion);
      if (td_cost >= 1.0) {
        RiskPoint tp{obj.position.x, obj.position.y, core_radius, td_cost};
        tp.inflation = cls_inflation;
        tp.cost_scale = cls_cost_scale;
        tp.teardrop = true;
        tp.facing = obj.facing;
        new_points.push_back(tp);
      }
    }
    // 등방 코어: facing 없으면 항상, 있으면 이동 성분(w_motion)만큼.
    const double iso_cost =
      obj.has_facing ? core_cost * w_motion : core_cost;
    if (!use_teardrop || iso_cost >= 1.0) {
      RiskPoint cp{obj.position.x, obj.position.y, core_radius, iso_cost};
      cp.inflation = cls_inflation;
      cp.cost_scale = cls_cost_scale;
      new_points.push_back(cp);
    }

    // 2) 예측 궤적 = LETHAL. risk_points_(자체 grid)에 칠하지 않고 pred_points_로
    //    따로 모아, updateCosts에서 정적 벽 raycast 체크 후 통과한 것만 master에
    //    직접 마킹한다(벽 너머 예측 제거). 소속 트랙 위치를 ray 시작점으로 보관.
    const std::size_t n = obj.predicted_trajectory.size();
    for (std::size_t k = 0; k < n; ++k) {
      const auto & pt = obj.predicted_trajectory[k];
      new_pred.push_back(
        {pt.x, pt.y, core_radius, obj.position.x, obj.position.y});
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    risk_points_ = std::move(new_points);
    pred_points_ = std::move(new_pred);
    track_frame_ = src_frame;
    last_msg_time_ = node->now();
    have_msg_ = true;
  }

  current_ = true;
}

bool CctvLayer::transformPoint(
  const std::string & src_frame, double in_x, double in_y,
  double & out_x, double & out_y)
{
  const std::string target = layered_costmap_->getGlobalFrameID();
  if (src_frame.empty() || src_frame == target) {
    out_x = in_x;
    out_y = in_y;
    return true;
  }

  geometry_msgs::msg::PoseStamped in;
  in.header.frame_id = src_frame;
  in.header.stamp = rclcpp::Time(0);  // 최신 가용 tf 사용
  in.pose.position.x = in_x;
  in.pose.position.y = in_y;
  in.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped out;
  try {
    out = tf_->transform(in, target, tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "CctvLayer: failed to transform '%s' -> '%s': %s",
      src_frame.c_str(), target.c_str(), ex.what());
    return false;
  }
  out_x = out.pose.position.x;
  out_y = out.pose.position.y;
  return true;
}

void CctvLayer::includePointBounds(
  double x, double y, double reach,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  // reach = 코어+후광을 포함한 전체 마킹 반경(호출부에서 점별로 계산해 전달).
  touch(x - reach, y - reach, min_x, min_y, max_x, max_y);
  touch(x + reach, y + reach, min_x, min_y, max_x, max_y);
}

void CctvLayer::markRiskPoint(const RiskPoint & p)
{
  // 점별 inflation/감쇠. class별로 다를 수 있으며, 음수면 레이어 기본값.
  const double inflation = (p.inflation >= 0.0) ? p.inflation : inflation_radius_;
  const double cost_scale = (p.cost_scale >= 0.0) ? p.cost_scale : cost_scaling_factor_;

  // 마킹 반경: 등방은 코어+inflation, 물방울은 가장 긴 축(전방 3σ)까지 덮어야
  // 앞쪽 꼬리가 잘리지 않는다.
  double total_radius;
  double cos_f = 1.0;
  double sin_f = 0.0;
  if (p.teardrop) {
    total_radius = p.radius + 3.0 * std::max({sigma_front_, sigma_side_, sigma_back_});
    cos_f = std::cos(p.facing);
    sin_f = std::sin(p.facing);
  } else {
    total_radius = p.radius + inflation;
  }
  if (total_radius <= 0.0) {
    return;
  }

  int min_i;
  int min_j;
  int max_i;
  int max_j;
  worldToMapEnforceBounds(p.x - total_radius, p.y - total_radius, min_i, min_j);
  worldToMapEnforceBounds(p.x + total_radius, p.y + total_radius, max_i, max_j);

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

      double cell_cost;
      if (p.teardrop) {
        // 사람 좌표계로 회전: x'=전후축(+앞), y'=좌우축
        const double dx = wx - p.x;
        const double dy = wy - p.y;
        const double x_along = dx * cos_f + dy * sin_f;
        const double y_side = -dx * sin_f + dy * cos_f;
        // 코어(반경 안)는 최대 비용 유지, 그 밖은 비대칭 가우시안 감쇠.
        const double core_d = std::hypot(dx, dy);
        if (core_d <= p.radius) {
          cell_cost = p.cost;
        } else {
          // 전방/후방에서 along축 σ를 다르게 → 앞으로 긴 물방울
          const double sigma_along =
            (x_along >= 0.0) ? sigma_front_ : sigma_back_;
          const double e =
            (x_along * x_along) / (2.0 * sigma_along * sigma_along) +
            (y_side * y_side) / (2.0 * sigma_side_ * sigma_side_);
          cell_cost = p.cost * std::exp(-e);
        }
      } else {
        const double dist = std::hypot(wx - p.x, wy - p.y);
        if (dist > total_radius) {
          continue;
        }
        if (dist <= p.radius) {
          // 코어 안: 최대 비용
          cell_cost = p.cost;
        } else {
          // 후광: 코어 경계에서 가우시안 감쇠 (점별 cost_scale 사용)
          const double d = dist - p.radius;
          const double factor = std::exp(-cost_scale * d);
          cell_cost = p.cost * factor;
        }
      }

      if (cell_cost < 1.0) {
        continue;
      }

      const unsigned char uc = static_cast<unsigned char>(
        std::min<double>(cell_cost, nav2_costmap_2d::LETHAL_OBSTACLE));

      const unsigned int ui = static_cast<unsigned int>(i);
      const unsigned int uj = static_cast<unsigned int>(j);
      const unsigned char old = getCost(ui, uj);
      // 여러 위험 점이 겹치면 최대값 유지
      if (old == nav2_costmap_2d::NO_INFORMATION || uc > old) {
        setCost(ui, uj, uc);
      }
    }
  }
}

void CctvLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
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

  // rolling window costmap(local)에서는 레이어 맵 원점이 로봇을 따라가야 한다.
  // 그렇지 않으면 setCost 좌표가 master grid와 어긋나 비용이 사라진다.
  if (layered_costmap_->isRolling()) {
    const double new_origin_x = robot_x - getSizeInMetersX() / 2.0;
    const double new_origin_y = robot_y - getSizeInMetersY() / 2.0;
    updateOrigin(new_origin_x, new_origin_y);
  }

  resetMaps();

  std::vector<RiskPoint> points_copy;
  std::vector<PredPoint> pred_copy;
  std::string src_frame;
  bool fresh = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (have_msg_) {
      const double age = (node->now() - last_msg_time_).seconds();
      fresh = (max_observation_age_ <= 0.0) || (age <= max_observation_age_);
    }
    if (fresh) {
      points_copy = risk_points_;
      pred_copy = pred_points_;
      src_frame = track_frame_;
    }
  }
  pred_points_world_.clear();

  double active_min_x = std::numeric_limits<double>::max();
  double active_min_y = std::numeric_limits<double>::max();
  double active_max_x = std::numeric_limits<double>::lowest();
  double active_max_y = std::numeric_limits<double>::lowest();
  bool has_active = false;

  for (const auto & p : points_copy) {
    // 트랙 frame(map) -> costmap 전역 frame(map 또는 odom)으로 변환
    RiskPoint tp = p;
    if (!transformPoint(src_frame, p.x, p.y, tp.x, tp.y)) {
      continue;  // 변환 실패한 점은 건너뜀
    }
    markRiskPoint(tp);
    // 전체 마킹 반경(reach) = 코어 + 후광. 물방울은 전방 3σ까지 뻗고, 등방은
    // 점별 inflation(class별)만큼 뻗는다. bounds가 좁으면 꼬리가 잘려 잔상이
    // 안 지워지므로 markRiskPoint와 동일 기준으로 계산해 넘긴다.
    double reach;
    if (tp.teardrop) {
      reach = tp.radius + 3.0 * std::max({sigma_front_, sigma_side_, sigma_back_});
    } else {
      const double infl = (tp.inflation >= 0.0) ? tp.inflation : inflation_radius_;
      reach = tp.radius + infl;
    }
    includePointBounds(tp.x, tp.y, reach, min_x, min_y, max_x, max_y);
    includePointBounds(
      tp.x, tp.y, reach,
      &active_min_x, &active_min_y, &active_max_x, &active_max_y);
    has_active = true;
  }

  // 예측 점: costmap frame으로 변환해 pred_points_world_에 보관(updateCosts에서
  // 벽 체크 후 master에 직접 마킹). 점과 src(ray 시작점) 둘 다 변환해야 한다.
  for (const auto & pp : pred_copy) {
    PredPoint wp = pp;
    if (!transformPoint(src_frame, pp.x, pp.y, wp.x, wp.y)) {
      continue;
    }
    if (!transformPoint(src_frame, pp.src_x, pp.src_y, wp.src_x, wp.src_y)) {
      continue;
    }
    pred_points_world_.push_back(wp);
    const double reach = wp.radius;  // 예측은 후광 없이 코어만 LETHAL
    includePointBounds(wp.x, wp.y, reach, min_x, min_y, max_x, max_y);
    includePointBounds(
      wp.x, wp.y, reach,
      &active_min_x, &active_min_y, &active_max_x, &active_max_y);
    has_active = true;
  }

  // 이전 프레임에 칠했던 영역도 bounds에 포함해야 다음에 지워진다.
  if (has_last_bounds_) {
    touch(last_min_x_, last_min_y_, min_x, min_y, max_x, max_y);
    touch(last_max_x_, last_max_y_, min_x, min_y, max_x, max_y);
  }

  has_last_bounds_ = has_active;
  if (has_active) {
    last_min_x_ = active_min_x;
    last_min_y_ = active_min_y;
    last_max_x_ = active_max_x;
    last_max_y_ = active_max_y;
  }
}

void CctvLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  // 예측 점은 현재 객체 core를 master에 합성하기 전에 벽 통과 여부를 판정한다.
  // 합성 후 검사하면 ray가 사람 자신의 lethal core를 벽으로 오인해 전부 잘릴 수 있다.
  std::vector<PredPoint> visible_pred_points;
  visible_pred_points.reserve(pred_points_world_.size());
  for (const auto & pp : pred_points_world_) {
    if (block_behind_walls_ &&
        rayBlockedByWall(master_grid, pp.src_x, pp.src_y, pp.x, pp.y))
    {
      continue;
    }
    visible_pred_points.push_back(pp);
  }

  // 1) 코어/물방울(자체 grid)을 master에 max 합성.
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);

  // 2) 벽 체크를 통과한 예측 점을 master에 직접 LETHAL 마킹한다.
  for (const auto & pp : visible_pred_points) {
    // 예측 코어 반경만큼 LETHAL로 칠한다(중심 1셀이 아니라 원).
    const double r = std::max(pp.radius, master_grid.getResolution());
    int ci, cj, lo_i, lo_j, hi_i, hi_j;
    unsigned int u;
    unsigned int v;
    if (!master_grid.worldToMap(pp.x, pp.y, u, v)) {
      continue;
    }
    ci = static_cast<int>(u);
    cj = static_cast<int>(v);
    const int cells = static_cast<int>(r / master_grid.getResolution());
    lo_i = std::max(min_i, ci - cells);
    hi_i = std::min(max_i, ci + cells);
    lo_j = std::max(min_j, cj - cells);
    hi_j = std::min(max_j, cj + cells);
    for (int j = lo_j; j <= hi_j; ++j) {
      for (int i = lo_i; i <= hi_i; ++i) {
        double wx;
        double wy;
        master_grid.mapToWorld(
          static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);
        if (std::hypot(wx - pp.x, wy - pp.y) > r) {
          continue;
        }
        master_grid.setCost(
          static_cast<unsigned int>(i), static_cast<unsigned int>(j),
          nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

bool CctvLayer::rayBlockedByWall(
  nav2_costmap_2d::Costmap2D & master_grid,
  double sx, double sy, double ex, double ey) const
{
  unsigned int s_mx;
  unsigned int s_my;
  unsigned int e_mx;
  unsigned int e_my;
  if (!master_grid.worldToMap(sx, sy, s_mx, s_my) ||
      !master_grid.worldToMap(ex, ey, e_mx, e_my))
  {
    return false;  // 좌표가 맵 밖이면 자르지 않음(보수적으로 표시 유지)
  }

  // Bresenham으로 시작→끝 셀을 훑는다. 시작점 부근(사람이 벽 옆) 오탐 방지를
  // 위해 첫 margin 셀은 벽 검사를 면제한다.
  int x0 = static_cast<int>(s_mx);
  int y0 = static_cast<int>(s_my);
  const int x1 = static_cast<int>(e_mx);
  const int y1 = static_cast<int>(e_my);

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx_step = (x0 < x1) ? 1 : -1;
  int sy_step = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  const int margin = 2;  // 시작점 부근 면제 셀 수
  int steps = 0;
  while (true) {
    if (x0 == x1 && y0 == y1) {
      break;
    }
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx_step;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy_step;
    }
    ++steps;
    if (steps <= margin) {
      continue;
    }
    // 끝 셀(예측점 자신)은 검사 안 함 — 그 셀이 벽이면 어차피 위에서 안 칠해짐.
    if (x0 == x1 && y0 == y1) {
      break;
    }
    const unsigned char c = master_grid.getCost(
      static_cast<unsigned int>(x0), static_cast<unsigned int>(y0));
    if (c >= static_cast<unsigned char>(wall_cost_thresh_) &&
        c != nav2_costmap_2d::NO_INFORMATION)
    {
      return true;  // 벽에 막힘
    }
  }
  return false;
}

void CctvLayer::reset()
{
  resetMaps();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    risk_points_.clear();
    pred_points_.clear();
    have_msg_ = false;
  }
  pred_points_world_.clear();
  current_ = true;
}

bool CctvLayer::isClearable()
{
  return false;
}

}  // namespace cctv_costmap_layer

PLUGINLIB_EXPORT_CLASS(cctv_costmap_layer::CctvLayer, nav2_costmap_2d::Layer)
