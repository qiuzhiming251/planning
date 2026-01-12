/**
 * @file speed_gaming_decider.h
 * @author zhang.xing (zhang.xing43@byd.com)
 * @brief
 * @version 0.1
 * @date 2025-03-19
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include "plan_common/timer.h"
#include "plan_common/planning_macros.h"
#include "simulator/alternative_idm_simulator.h"
#include "simulator/multi_agent_idm.h"
#include "speed_gaming_evaluator.h"
#include "speed_obstacle_processor.h"
#include "gaming_util.h"
#include "modules/cnoa_pnc/planning/proto/gaming.pb.h"

#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
namespace st::planning {
struct SpeedGamingInput {
  const SpacetimeTrajectoryManager *traj_mgr = nullptr;
  const DiscretizedPath *ego_path = nullptr;
  const double plan_start_v = 0.0;
  const double plan_start_a = 0.0;
  const double plan_start_j = 0.0;
  const DrivePassage *drive_passage = nullptr;
  const SpeedLimitProvider *speed_limit_provider = nullptr;
  const std::vector<StBoundaryWithDecision> *st_boundaries_with_decision =
      nullptr;
  const std::unordered_map<std::string, SpacetimeObjectTrajectory>
      *processed_st_objects = nullptr;
  int plan_id;
  // gaming lc obs id
  const absl::flat_hash_set<std::string> gaming_lc_obs_set;
  const SpeedGamingResultProto *last_speed_gaming_result = nullptr;
};

struct SpeedGamingOutput {
  // std::vector<StBoundaryWithDecision> st_boundaries_with_decision_gaming;
  std::unordered_map<std::string, SpacetimeObjectTrajectory>
      gaming_processed_st_objects;  //只存储yield的修正预测轨迹给纵向DP
  std::unordered_map<std::string, RiskObjInfo>
      gaming_risk_field_zoneinfos;  //存储风险场障碍物信息
  std::unordered_map<std::string, StBoundaryProto::DecisionType>
      gaming_decision;  //存储所有接入障碍物的决策结果
  SpeedVector gaming_ego_speed_profile;
  SpeedGamingResultProto speed_gaming_result;
};

class SpeedGamingDecider {
 public:
  SpeedGamingDecider(const VehicleGeometryParamsProto *vehicle_geo_params,
                     const VehicleDriveParamsProto *vehicle_drive_params)
      : vehicle_geo_params_(vehicle_geo_params),
        vehicle_drive_params_(vehicle_drive_params) {}
  ~SpeedGamingDecider() = default;

  /**
   * @brief 外部调用主函数
   *
   */
  absl::StatusOr<SpeedGamingOutput> Execute(const SpeedGamingInput &input);

  //暴露给DP之后的解冲突接口
  absl::StatusOr<SpeedGamingOutput> ConflictResolutionAfterDP(
      const SpeedGamingInput &input,
      std::unordered_map<std::string, StBoundaryProto::DecisionType>
          &single_gaming_decision);

  //城区分支single-agent-only标志位
  void SetIsSingleAgentGaming(bool is_single_agent_gaming) {
    is_single_agent_gaming_ = is_single_agent_gaming;
  };
  bool GetIsSingleAgentGaming() { return is_single_agent_gaming_; };

 private:
#ifdef ALTERNATIVE_GAMING_UNIT_TEST
 public:
#endif
  void RunCrossGaming(
      const DiscretizedPath &av_path, const TrajectoryPoint &ego_init_point,
      const SpacetimeTrajectoryManager &speed_traj_manager,
      const std::unordered_map<std::string, SpacetimeObjectTrajectory>
          &processed_st_objects,
      const GamingConflictZoneInfo &conflict_zone_info,
      const PiecewiseLinearFunction<double> &speed_limit_plf,
      const SpeedVector &upper_speed_data,
      const SpeedVector &normal_yield_speed_data,
      GamingSimResult *gaming_sim_result);

  void RunAgentMergeEgoGaming(
      const DiscretizedPath &av_path, const TrajectoryPoint &ego_init_point,
      const SpacetimeTrajectoryManager &speed_traj_manager,
      const std::unordered_map<std::string, SpacetimeObjectTrajectory>
          &processed_st_objects,
      const GamingConflictZoneInfo &conflict_zone_info,
      const PiecewiseLinearFunction<double> &speed_limit_plf,
      const SpeedVector &upper_speed_data,
      const SpeedVector &normal_yield_speed_data,
      GamingSimResult *gaming_sim_result);

  void RunEgoMergeAgentGaming(
      const DiscretizedPath &av_path, const TrajectoryPoint &ego_init_point,
      const SpacetimeTrajectoryManager &speed_traj_manager,
      const std::unordered_map<std::string, SpacetimeObjectTrajectory>
          &processed_st_objects,
      const GamingConflictZoneInfo &conflict_zone_info,
      const PiecewiseLinearFunction<double> &speed_limit_plf,
      const SpeedVector &upper_speed_data,
      const SpeedVector &normal_yield_speed_data,
      GamingSimResult *gaming_sim_result);

  void RunGaming(
      const DiscretizedPath &av_path, const TrajectoryPoint &ego_init_point,
      const SpacetimeTrajectoryManager &speed_traj_manager,
      const std::unordered_map<std::string, SpacetimeObjectTrajectory>
          &processed_st_objects,
      const GamingConflictZoneInfo &conflict_zone_info,
      const PiecewiseLinearFunction<double> &speed_limit_plf,
      const SpeedVector &upper_speed_data,
      const SpeedVector &normal_yield_speed_data, AidmScene aidm_scene,
      GamingSimResult *gaming_sim_result);

  std::unordered_set<std::string> RunSimpleSpeedGamingDecider(
      const std::vector<const StBoundary *> &st_boundarys,
      const std::unordered_map<std::string, GamingConflictZoneInfo>
          &risk_field_conflict_zones,
      const std::optional<double> &traffic_light_stop_fence_dist,
      const SpeedVector &ego_upper_speed_data) const;

  std::optional<std::vector<std::string>> ConflictResolution(
      const SpeedGamingInput &input, const SpeedIdmUpdateParams &idm_param,
      const SpeedVector &normal_yield_speed_data, std::string &group_name,
      std::string &debug_info, SpeedVector &upper_speed_data);

 private:
  const VehicleGeometryParamsProto *vehicle_geo_params_;
  const VehicleDriveParamsProto *vehicle_drive_params_;
  std::shared_ptr<SpeedObstacleProcessor> speed_obs_processor_ =
      std::make_shared<SpeedObstacleProcessor>();
  std::unordered_map<std::string, GamingConflictZoneInfo> conflict_zone_infos_;
  SpeedGamingParams
      speed_gaming_params_;  // TODO by
                             // zx：待参数proto文件配置好后，需改成在构造函数中初始化
  //以下变量成员化，方便DP之后解冲突直接使用，不用再次生成
  IDMConfigPlf aggressive_idm_config_plf_ = {
      PiecewiseLinearFunction<double>({0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
                                      {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3}),
      PiecewiseLinearFunction<double>({0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
                                      {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0})
      //   PiecewiseLinearFunction<double>({0.0, 5.0, 10.0}, {0.0, 0.0, 0.0}),
      //   PiecewiseLinearFunction<double>({0.0, 5.0, 10.0}, {0.0, 0.0, 0.0})
  };
  IDMConfigPlf normal_idm_config_plf_ = {
      PiecewiseLinearFunction<double>({0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
                                      {0.5, 0.5, 0.5, 0.6, 0.8, 1.0, 1.0}),
      PiecewiseLinearFunction<double>({0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
                                      {1.2, 1.2, 1.5, 2.0, 2.0, 2.0, 2.0, 2.0})
      // PiecewiseLinearFunction<double>({0.0, 5.0, 10.0}, {0.0, 0.0, 2.0}),
      // PiecewiseLinearFunction<double>({0.0, 5.0, 10.0}, {0.0, 0.0, 1.0})
  };
  IDMConfigPlf follow_idm_config_plf_ = {
      PiecewiseLinearFunction<double>({0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
                                      {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}),
      PiecewiseLinearFunction<double>({0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
                                      {3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0})
      // PiecewiseLinearFunction<double>({0.0, 5.0, 10.0}, {0.0, 0.0, 2.0}),
      // PiecewiseLinearFunction<double>({0.0, 5.0, 10.0}, {0.0, 0.0, 1.0})
  };
  TrajectoryPoint ego_init_point_;
  PiecewiseLinearFunction<double> speed_limit_plf_;
  std::unique_ptr<MultiAgentIdm> multi_agent_idm_;
  SpeedVector follow_upper_speed_data_;  //刷follow障碍物生成的upperbound
  std::unordered_map<std::string, GamingSimResult> ego_follow_sim_results_;
  std::unordered_map<std::string, GamingSimResult> ego_yield_sim_results_;
  std::unordered_map<std::string, GamingSimResult> ego_overtake_sim_results_;
  bool is_single_agent_gaming_ = false;  //是否单agent博弈
  bool is_risk_field_enabled_ = false;   //是否启用风险场
  std::unordered_map<std::string, ObjectCooperationInfo>
      object_cooperation_maps_;
  IntersectionTurnType intersection_turn_type_;
#ifdef ALTERNATIVE_GAMING_UNIT_TEST
  Json::Value g_root_;
#endif
};
}  // namespace st::planning
