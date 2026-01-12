/**
 * @file speed_obstacle_preprocess.h
 * @author liu.tianrui1 (liu.tianrui1@byd.com)
 * @brief
 * @version 0.1
 * @date 2025-03-30
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include "speed_gaming_common.h"
#include "simulator/speed_idm_base.h"
#include "simulator/speed_idm_common.h"

namespace st::planning {
enum class GamingCostType { kEgo = 1, kObj = 2 };

class GamingLaneChangeEvaluator {
 public:
  GamingLaneChangeEvaluator(const SpeedGamingParams *speed_gaming_params)
      : speed_gaming_params_(speed_gaming_params){};
  virtual ~GamingLaneChangeEvaluator() = default;

  void Init(const double obj_length, const double obj_width,
            const double obj_offset, const double ego_length,
            const double ego_width, const double ego_offset,
            const PiecewiseLinearFunction<double> &ego_speed_limit_plf,
            const PiecewiseLinearFunction<double> &obj_speed_limit_plf,
            const ObjectType obj_type,
            const GamingSimResult &gaming_sim_result);

  double LaneChangeSafetyEvaluate(bool &is_collision_result);
  const std::string &GetDebugMessage() const { return debug_; }

  void ClearDebugMessage() { debug_.clear(); }

 private:
  std::string debug_;
  GamingSimResult gaming_sim_result_;
  double obj_length_ = 0.0;
  double obj_width_ = 0.0;
  double obj_offset_ = 0.0;
  double ego_length_ = 0.0;
  double ego_width_ = 0.0;
  double ego_offset_ = 0.0;
  ObjectType obj_type_ = ObjectType::OT_VEHICLE;
  double end_ego_traj_s_ = 0.0;
  PiecewiseLinearFunction<double> ego_speed_limit_plf_;
  PiecewiseLinearFunction<double> obj_speed_limit_plf_;
  const SpeedGamingParams *speed_gaming_params_;

  // weight
  const double w_ego_jerk_smoothness_ = 10.0;
  const double w_obj_jerk_smoothness_ = 5.0;
  const double w_ego_acc_smoothness_ = 2.0;
  const double w_obj_acc_smoothness_ = 1.0;
  const double w_ego_speed_efficiency_ = 200.0;
  const double w_obj_speed_efficiency_ = 50.0;
  const double w_ego_travel_dis_efficiency_ = 200.0;
  const double w_obj_travel_dis_efficiency_ = 50.0;
  const double w_safety_ = 300.0;
  const double w_ego_break_to_stop_ = 200.0;
  const double w_obj_break_to_stop_ = 100.0;
  const double w_both_break_to_stop_ = 150.0;

  const std::vector<double> w_index_ = {
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

  double GetWeight(size_t t);

  double CalculateStaticSafetyCost(bool &is_collision_result);
  double CalculateMinBoxDistance(const size_t i);
  int CheckBreakToStop(GamingCostType cost_type);

  double CalculateJerkSmoothness(GamingCostType cost_type);
  double CalculateAccSmoothness(GamingCostType cost_type);

  double CalculateTravelEfficiency(GamingCostType cost_type);
  double CalculateSpeedEfficiency(GamingCostType cost_type);
  double CalculatePointToSegmentDistance(const std::pair<double, double> &point,
                                         const std::pair<double, double> &start,
                                         const std::pair<double, double> &end);
};

}  // namespace st::planning
