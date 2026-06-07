#ifndef CCTV_COSTMAP_LAYER__CCTV_LAYER_HPP_
#define CCTV_COSTMAP_LAYER__CCTV_LAYER_HPP_

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "perception_msgs/msg/tracked_track_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cctv_costmap_layer
{

// CctvLayer: CCTV 추적기(/cctv/tracks)의 객체 위치 + 예측 궤적을 받아
// "똑똑한" 위험 비용을 costmap에 칠하는 Nav2 레이어. (옛 RiskLayer를 발전시킨 것)
//
//   1) 객체 현재 위치는 강한(LETHAL) 코어로 마킹
//   2) 예측 궤적(predicted_trajectory)을 따라 미래 점들을 시간 감쇠 비용으로 마킹
//      → 가까운 미래는 강하게, 먼 미래는 약하게 (객체가 갈 곳을 미리 비움)
//   3) 각 점 주변은 가우시안으로 부드럽게 inflation
//   4) tracking_stability가 낮으면 전체 비용을 약화 (오탐 보호)
//   5) 비전 pose 응시방향(facing)이 있으면 현재 위치 코어를 등방 원이 아니라
//      "물방울"(asymmetric Gaussian: 앞으로 길고 뒤로 짧음)로 칠한다. 정지한
//      사람이 바라보는 방향의 공간을 더 비워 사회적으로 우회하게 한다.
//      (Kirby 2010 asymmetric Gaussian personal space)
//      속도가 빠를수록 궤적 비용이, 느릴수록 물방울 비용이 지배하도록 블렌딩.
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
  struct RiskPoint
  {
    double x;        // map frame [m]
    double y;        // map frame [m]
    double radius;   // 코어 반경 [m]
    double cost;     // 코어 비용 [0..254]
    // 점별 inflation/감쇠 (class별로 다를 수 있음). 0 이하면 레이어 기본값 사용.
    double inflation{-1.0};   // 코어 밖 후광 반경 [m] (<0 이면 inflation_radius_)
    double cost_scale{-1.0};  // 가우시안 감쇠 계수 (<0 이면 cost_scaling_factor_)
    // 물방울(비대칭) 마킹용. teardrop=false면 기존 등방 원형으로 칠한다.
    bool teardrop{false};   // true면 facing 방향 비대칭 가우시안
    double facing{0.0};     // 응시 방향 yaw [rad] (teardrop일 때만 유효)
  };

  // class별 inflation/감쇠/코어반경 override. class_id로 lookup해 점에 주입한다.
  struct ClassParams
  {
    double core_radius;
    double inflation_radius;
    double cost_scaling_factor;
  };

  // 예측 궤적 점. 벽 자르기를 위해 소속 트랙 위치(src)를 함께 보관한다.
  struct PredPoint
  {
    double x;        // 예측 점 [m]
    double y;
    double radius;   // 코어 반경 [m]
    double src_x;    // 소속 트랙 현재 위치(ray 시작점) [m]
    double src_y;
  };

  void tracksCallback(const perception_msgs::msg::TrackedTrackArray::SharedPtr msg);

  // class_names + "<class>.core_radius/inflation_radius/cost_scaling_factor"
  // 평탄 파라미터를 읽어 class_params_ map을 채운다.
  void loadClassParams();

  // (sx,sy)→(ex,ey) 직선상에 master_grid 비용이 wall_cost_thresh_ 이상인 셀이
  // 있으면 true(벽에 막힘). 시작점 부근 margin 셀은 면제.
  bool rayBlockedByWall(
    nav2_costmap_2d::Costmap2D & master_grid,
    double sx, double sy, double ex, double ey) const;

  // 한 위험 점(코어 + 가우시안 후광)을 staging 비용맵에 누적 마킹.
  // p.teardrop이면 facing 방향 비대칭 가우시안(물방울), 아니면 등방 원형.
  void markRiskPoint(const RiskPoint & p);
  void includePointBounds(
    double x, double y, double radius,
    double * min_x, double * min_y, double * max_x, double * max_y);
  // 트랙 frame(보통 map)의 (x,y)를 costmap 전역 frame으로 변환.
  // 변환 실패 시 false (해당 점은 건너뜀).
  bool transformPoint(
    const std::string & src_frame, double in_x, double in_y,
    double & out_x, double & out_y);

  std::mutex mutex_;
  // 메시지 원본 frame 그대로 보관한 위험 점 (변환은 updateBounds에서 수행)
  std::vector<RiskPoint> risk_points_;
  // 예측 궤적 점(원본 frame). 자체 grid에 안 칠하고 updateCosts에서 벽 체크 후
  // master에 직접 마킹한다.
  std::vector<PredPoint> pred_points_;
  // updateBounds에서 costmap frame으로 변환해 둔 예측 점(updateCosts가 사용).
  std::vector<PredPoint> pred_points_world_;
  std::string track_frame_;
  rclcpp::Time last_msg_time_;
  bool have_msg_{false};

  rclcpp::Subscription<perception_msgs::msg::TrackedTrackArray>::SharedPtr tracks_sub_;

  // 파라미터
  std::string topic_{"/cctv/tracks"};
  double base_radius_{0.35};           // footprint 정보 없을 때 코어 반경 [m]
  double inflation_radius_{0.8};       // 코어 밖 가우시안 후광 반경 [m]
  double cost_scaling_factor_{3.0};    // 가우시안 감쇠 계수 (클수록 빨리 감쇠)
  double prediction_decay_{0.6};       // 예측 점 비용 감쇠 (먼 미래일수록 약화)
  double min_stability_{0.3};          // 이 안정도 미만이면 트랙 무시
  double max_observation_age_{1.0};    // 메시지 신선도 [s]
  double transform_tolerance_{0.2};

  // ── 물방울(asymmetric Gaussian personal space) 파라미터 ──
  bool teardrop_enabled_{true};        // facing 있는 트랙에 물방울 비용 적용
  double sigma_front_{1.0};            // 응시 전방 분산 σ [m] (길게)
  double sigma_side_{0.45};            // 좌우 분산 σ [m]
  double sigma_back_{0.35};            // 후방 분산 σ [m] (짧게)
  // 속도 블렌딩: v<=static이면 물방울 100%, v>=moving이면 궤적 100%.
  double static_speed_{0.08};          // [m/s] 이 속도 이하 = 정지로 간주
  double moving_speed_{0.20};          // [m/s] 이 속도 이상 = 이동으로 간주

  // ── class별 inflation 차등 ──
  // class_id → {core_radius, inflation_radius, cost_scaling_factor}.
  // 비어 있거나 미등록 class면 위 전역 기본값(base_radius_/inflation_radius_/
  // cost_scaling_factor_)을 사용한다.
  std::map<std::string, ClassParams> class_params_;

  // ── 예측궤적 벽 자르기 ──
  bool block_behind_walls_{true};   // 예측이 정적 벽 너머면 잘라낸다
  int wall_cost_thresh_{253};       // master 비용이 이 값 이상이면 벽으로 간주
                                    // (253=INSCRIBED, 254=LETHAL)

  bool has_last_bounds_{false};
  double last_min_x_{0.0};
  double last_min_y_{0.0};
  double last_max_x_{0.0};
  double last_max_y_{0.0};
};

}  // namespace cctv_costmap_layer

#endif  // CCTV_COSTMAP_LAYER__CCTV_LAYER_HPP_
