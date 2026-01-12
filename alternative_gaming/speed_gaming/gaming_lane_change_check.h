#pragma once
#include "plan_common/timer.h"
#include "plan_common/planning_macros.h"
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <optional>
#include "simulator/alternative_idm_simulator.h"
#include "simulator/multi_agent_idm.h"
#include "gaming_lane_change_evaluator.h"
#include "speed_obstacle_processor.h"
#include "gaming_util.h"
namespace st::planning {
struct LaneChangeSafetyGamingInput {
  const SpacetimeTrajectoryManager *traj_mgr = nullptr;
  const DiscretizedPath *ego_path = nullptr;
  const double plan_start_v = 0.0;
  const double plan_start_a = 0.0;
  const double plan_start_j = 0.0;
  const DrivePassage *drive_passage = nullptr;
  const double speed_limit = 0.0;
  int plan_id;
};

class LaneChangeSafetyChecker {
 public:
  LaneChangeSafetyChecker(const VehicleGeometryParamsProto *vehicle_geo_params,
                          const VehicleDriveParamsProto *vehicle_drive_params)
      : vehicle_geo_params_(vehicle_geo_params),
        vehicle_drive_params_(vehicle_drive_params) {}
  ~LaneChangeSafetyChecker() = default;
  absl::StatusOr<std::string> Execute(const LaneChangeSafetyGamingInput &input,
                                      const std::string &check_traj_id);

  SpeedVector GenerateEgoSpeedProfile(
      const LaneChangeSafetyGamingInput
          &input);  // 所有障碍车判断OK，最后调用此函数输出reference

 private:
  absl::Status LaneChangeSafetyCheck(
      const DiscretizedPath &av_path, const TrajectoryPoint &ego_init_point,
      const SpacetimeTrajectoryManager &speed_traj_manager,
      const GamingConflictZoneInfo &conflict_zone_info,
      const PiecewiseLinearFunction<double> &speed_limit_plf,
      const SpeedVector &upper_speed_data,
      const SpeedVector &normal_yield_speed_data, AidmScene aidm_scene,
      GamingSimResult *gaming_sim_result);

 private:
  const VehicleGeometryParamsProto *vehicle_geo_params_ = nullptr;
  const VehicleDriveParamsProto *vehicle_drive_params_ = nullptr;
  bool first_trigger_ = false;
  std::shared_ptr<SpeedObstacleProcessor> speed_obs_processor_ =
      std::make_shared<SpeedObstacleProcessor>();
  SpeedGamingParams speed_gaming_params_;
  std::unique_ptr<MultiAgentIdm> multi_agent_idm_;
  std::unordered_map<std::string, GamingSimResult> ego_follow_sim_results_;
  std::unordered_map<std::string, GamingSimResult> ego_yield_sim_results_;
  std::unordered_map<std::string, GamingSimResult> ego_overtake_sim_results_;
  SpeedVector upper_speed_data_ = {};
  TrajectoryPoint ego_init_point_;
  PiecewiseLinearFunction<double> speed_limit_plf_;
  IDMConfigPlf aggressive_idm_config_plf_ = {
      PiecewiseLinearFunction<double>({0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
                                      {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3}),
      PiecewiseLinearFunction<double>(
          {0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
          {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0})};
  IDMConfigPlf normal_idm_config_plf_ = {
      PiecewiseLinearFunction<double>({0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
                                      {0.5, 0.5, 0.5, 0.6, 0.8, 1.0, 1.0}),
      PiecewiseLinearFunction<double>(
          {0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
          {1.2, 1.2, 1.5, 2.0, 2.0, 2.0, 2.0, 2.0})};
  IDMConfigPlf follow_idm_config_plf_ = {
      PiecewiseLinearFunction<double>({0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
                                      {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}),
      PiecewiseLinearFunction<double>(
          {0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
          {3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0})};
};
}  // namespace st::planning