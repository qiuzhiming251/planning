/***********************************************************
 * @file     alternative_idm_simulator.cc
 * @author   tianrui.liu
 * @date     2025.03.15
 * @brief    The implementation of alternative-idm simulator
 * @version  1.0
 ***********************************************************/

#pragma once
#include "alternative_gaming/speed_gaming/speed_gaming_common.h"
#include "speed_cross_idm.h"
#include "speed_merge_idm.h"
#ifdef ALTERNATIVE_GAMING_UNIT_TEST
#include <json/json.h>
#endif
namespace st::planning {
enum class AidmScene { kMergeIn = 1, kCross = 2 };

struct AidmConfig {
  AidmConfig() {}
  AidmConfig(double _cooperation_grade, double _update_coeff,
             double _ego_yield_coeff, double _time_step, int _iter_time)
      : cooperation_grade(_cooperation_grade),
        update_coeff(_update_coeff),
        ego_yield_coeff(_ego_yield_coeff),
        time_step(_time_step),
        iter_time(_iter_time) {}
  double cooperation_grade = 1.0;
  double update_coeff = 0.5;
  double ego_yield_coeff = 0.5;
  double time_step = 0.2;
  int iter_time = 10;
};

struct AlternativeSimulatorInfo {
  GamingConflictZone riskfield_conflict_zone_in_ego_view;
  GamingConflictZone riskfield_conflict_zone_in_agent_view;
  PiecewiseLinearFunction<double> ego_speed_limit_plf;    // SV speed limit
  PiecewiseLinearFunction<double> agent_speed_limit_plf;  // SV speed limit
  double ego_cross_angle;
  double agent_cross_angle;
  TrajectoryPoint ego_init_point;
  TrajectoryPoint agent_init_point;
  SpeedVector upper_speed_data;
  SpeedVector normal_yield_speed_data;
  DiscretizedPath av_path;
  DiscretizedPath agent_path;
  double ideal_idm_dist;
};

class AlternativeIDMSimulator {
 public:
  AlternativeIDMSimulator(const AidmScene &aidm_scene,
                          const SpeedGamingParams *speed_gaming_params)
      : aidm_scene_(aidm_scene), speed_gaming_params_(speed_gaming_params) {
    if (aidm_scene == AidmScene::kCross) {
      speed_idm_base_ = std::make_shared<SpeedCrossIdm>(speed_gaming_params_);
    } else {
      speed_idm_base_ = std::make_shared<SpeedMergeIdm>(speed_gaming_params_);
    }
  }

  void SetConfig(const AidmConfig &aidm_config) { aidm_config_ = aidm_config; }

  void RunEgoPassSimulation(
      const AlternativeSimulatorInfo &alter_simulator_info,
      const PlannerObject &gaming_object, GamingSimResult *gaming_sim_result,
      IdmSimulateState *simulate_state = nullptr) const;

  void RunEgoYieldSimulation(
      const AlternativeSimulatorInfo &alter_simulator_info,
      const PlannerObject &gaming_object, GamingSimResult *gaming_sim_result,
      IdmSimulateState *simulate_state = nullptr) const;

  const AidmConfig &GetConfig() const { return aidm_config_; }

  static int frame_index;

 private:
  // Simulation of pass fixing agent2
  bool PassSimulation(const TrajectoryPoint &init_point,
                      const DiscretizedPath &av_path,
                      const SpeedVector &ego_upper_speed_data,
                      const SpeedVector &ego_normal_speed_data,
                      GamingConflictZone &risk_field_conflict_zone_agent_1,
                      SpeedVector &agent_2_speed_profile,
                      GamingConflictZone &risk_field_conflict_zone_agent_2,
                      double ego_cross_angle,
                      const PiecewiseLinearFunction<double> &speed_limit_plf,
                      SpeedVector *agent_1_speed_profile,
                      IdmSimulateState *simulate_state) const;
  // Simulation of yield fixing agent2
  bool YieldSimulation(const TrajectoryPoint &init_point,
                       const DiscretizedPath &av_path,
                       GamingConflictZone &risk_field_conflict_zone_agent_1,
                       SpeedVector &agent_2_speed_profile,
                       GamingConflictZone &risk_field_conflict_zone_agent_2,
                       const PiecewiseLinearFunction<double> &speed_limit_plf,
                       SpeedVector *agent_1_speed_profile,
                       const PlannerObject &gaming_object,
                       IdmSimulateState *simulate_state) const;
  bool CalPassSpeedProfile(
      const TrajectoryPoint &agent1_init_point, const DiscretizedPath &av_path,
      const GamingConflictZone &riskfield_conflict_zone_agent_1,
      double ego_cross_angle,
      const PiecewiseLinearFunction<double> &speed_limit_plf,
      const SpeedVector &ego_upper_speed_data,
      const SpeedVector &ego_normal_yield_speed_data,
      const SpeedVector &agent2_decel_speed_profile,
      const SpeedVector &origin_speed_data, SpeedVector *pass_speed_data,
      IdmSimulateState *simulate_state) const;
  bool CalYieldSpeedProfile(
      const TrajectoryPoint &init_point, const DiscretizedPath &av_path,
      const GamingConflictZone &riskfield_conflict_zone,
      const SpeedVector &agent_speed_profile,
      const PiecewiseLinearFunction<double> &speed_limit_plf,
      const SpeedVector &origin_speed_data, SpeedVector *yield_speed_data,

      const PlannerObject &gaming_object,
      IdmSimulateState *simulate_state) const;
  void ReverseEgoAgentInfo(GamingConflictZone *riskfiled_conflict_zone) const;

 private:
  std::shared_ptr<SpeedGamingIdmBase> speed_idm_base_;
  AidmConfig aidm_config_;
  AidmScene aidm_scene_;
  const SpeedGamingParams *speed_gaming_params_;

#ifdef ALTERNATIVE_GAMING_UNIT_TEST
  mutable Json::Value g_root_;
#endif
};

}  // namespace st::planning