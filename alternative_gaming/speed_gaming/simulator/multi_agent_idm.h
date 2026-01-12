/**
 * @file multi_agent_idm.h
 * @author liu.tianrui (liu.tianrui1@byd.com)
 * @brief gaming result post process
 * @version 0.1
 * @date 2025-04-19
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include "../speed_obstacle_processor.h"
#include "alternative_idm_simulator.h"
namespace st::planning {

struct IDMConfigPlf {
  PiecewiseLinearFunction<double> k0_plf;
  PiecewiseLinearFunction<double> d0_plf;
  //    PiecewiseLinearFunction<double> speed_k0_plf;
  //    PiecewiseLinearFunction<double> speed_d0_plf;
};

struct LastInfos {
  double last_s;
  double last_v;
  double last_a;
  double last_delta_theta;
};
class MultiAgentIdm {
 public:
  MultiAgentIdm(const SpeedGamingParams *speed_gaming_params)
      : speed_gaming_params_(speed_gaming_params){};

  ~MultiAgentIdm() = default;

  SpeedVector CalcYieldTrajSpeedDataOfMultiAgent(
      const TrajectoryPoint &init_point, const DiscretizedPath &av_path,
      const SpeedVector &upper_speed_data, const IDMConfigPlf &idm_config_plf,
      const IDMConfigPlf &follow_idm_config_plf,
      const PiecewiseLinearFunction<double> &speed_limit_plf,
      const SpeedIdmUpdateParams &idm_param,
      const std::unordered_map<std::string, GamingSimResult>
          &ego_yield_sim_results,
      const std::unordered_map<std::string, GamingSimResult>
          &ego_follow_sim_results = {});

  double CalcMergeYieldAcc(const SpeedIdmState &idm_state,
                           const GamingSimResult &yield_conflict_zone,
                           const int index,
                           const SpeedIdmUpdateParams &idm_param, double &beta,
                           const DiscretizedPath &av_path,
                           LastInfos &last_info);

  double CalcCrossYieldAcc(SpeedIdmState &idm_state,
                           const GamingSimResult &yield_conflict_zone,
                           double simu_time,
                           const SpeedIdmUpdateParams &idm_param,
                           const IDMConfigPlf &idm_config_plf, double &beta);
  double CalcMergeOvertakeAcc(const SpeedIdmState &idm_state,
                              const GamingSimResult &yield_conflict_zone,
                              double simu_time,
                              const SpeedIdmUpdateParams &idm_param,
                              double &beta);

  double CalcCrossOvertakeAcc(const SpeedIdmState &idm_state,
                              const GamingSimResult &yield_conflict_zone,
                              double simu_time,
                              const SpeedIdmUpdateParams &idm_param,
                              double &beta);

  double CalcUpperSpeedBoundAcc(double t, const SpeedIdmState &cur_state,
                                const SpeedVector &upper_speed_bound);
  double CalcNormalSpeedTrackAcc(double t, const SpeedIdmState &cur_state,
                                 const SpeedVector &upper_speed_bound);
  SpeedVector CalcTrajSpeedDataOfMultiAgent(
      const TrajectoryPoint &init_point, const DiscretizedPath &av_path,
      const SpeedVector &upper_speed_data, const SpeedVector &normal_speed_data,
      const PiecewiseLinearFunction<double> &speed_limit_plf,
      const SpeedIdmUpdateParams &idm_param,
      const std::unordered_map<std::string, GamingSimResult>
          &ego_overtake_sim_results);

 private:
  SpeedVector upper_speed_data_;
  SpeedVector normal_speed_data_;
  const SpeedGamingParams *speed_gaming_params_;
};

}  // namespace st::planning