/**
 * @file speed_gaming_common.h
 * @author zhang.xing (zhang.xing43@byd.com)
 * @brief
 * @version 0.1
 * @date 2025-03-19
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "plan_common/log_data.h"
#include "plan_common/speed/st_speed/speed_limit.h"
#include "plan_common/speed/st_speed/speed_limit_provider.h"
#include "plan_common/util/status_macros.h"
#include "predictor/prediction_util.h"
#include "simulator/speed_idm_common.h"
namespace st::planning {
enum LonGamingDecisionType {
  kNone = 0,
  kYield = 1,
  kOvertake = 2,
};

struct GamingSimResult {
  const PlannerObject *planner_object = nullptr;
  SpeedVector ego_speed_data{};
  SpeedVector obj_speed_data{};
  std::vector<TrajectoryPoint> ego_traj = {};
  std::vector<TrajectoryPoint> obj_traj = {};
  LonGamingDecisionType ego_lon_type = LonGamingDecisionType::kNone;
  InteractionType interaction_type = InteractionType::KNone;
  GamingConflictZone sim_conflict_zone_in_ego_view{};
  GamingConflictZone sim_conflict_zone_in_agent_view{};
  std::string debug_info = "";
};

class SpeedGamingCommon {
 public:
  SpeedGamingCommon() = default;
  ~SpeedGamingCommon() = default;

  static SpeedVector ConstructConstAccelSpeedProfile(
      const TrajectoryPoint &init_point, double accel, double speed_limit,
      const PiecewiseLinearFunction<double> &speed_limit_plf, double timestep,
      double time_horizon);

  static DiscretizedPath GeneratePathBasedTraj(
      const SpacetimeObjectTrajectory *traj, double max_s, bool extend = true);

  static double CalcEgoCrossAngle(
      const DiscretizedPath &av_path,
      const SpacetimeObjectTrajectory *agent_pred_traj, double agent_cutin_time,
      double ego_cutout_s);

  static bool GetPredictedPointByTimeFromTrajectory(
      const SpacetimeObjectTrajectory *PredictedTraj, double time,
      prediction::PredictedTrajectoryPoint *traj_point);

  static SpacetimeObjectTrajectory genThetaValidTraj(
      const SpacetimeObjectTrajectory &st_traj);

  static double CalcAgentCrossAngle(
      const DiscretizedPath &av_path,
      const SpacetimeObjectTrajectory *agent_pred_traj,
      double agent_cutout_time, double ego_cutin_s);

  static PiecewiseLinearFunction<double> GetObjCurveSpeedLimit(
      const SpacetimeObjectTrajectory *pred_traj);

  static double CalKappaByThreePoints(const Vec2d &p1, const Vec2d &p2,
                                      const Vec2d &p3);

  static double CalCurveSpeedlimit(const double k);

  static double TableInterpolate(
      const std::vector<std::pair<double, double>> &table, double x);

  static SpeedVector ConvertPredTrajToSpeedData(
      const SpacetimeObjectTrajectory *pred_traj, double time_step,
      double time_horizon);

  static bool UpdateConflictZoneInfoByEgoSpeedProfile(
      const SpeedVector &ego_speed_data,
      GamingConflictZoneInfo *riskfield_conflict_info);

  static bool UpdateConflictZoneInfoByAgentSpeedProfile(
      const SpeedVector &agent_speed_data,
      GamingConflictZoneInfo *riskfield_conflict_info);

  static void ObjSpeedDataPostProcess(SpeedVector &origin_speed_data,
                                      double time_step, double min_jerk,
                                      double max_jerk);

  static double CalcJerkByAcc(double cur_acc, double target_acc, double pJerk,
                              double time_step1);

  static double CalcSaturatedJerkByAcc(double cur_acc, double target_acc,
                                       double pJerk, double time_step1,
                                       const double jerk_lower_bound,
                                       const double jerk_upper_bound);

  static SpeedIdmState UpdateNextEgoState(const SpeedIdmState &cur_state,
                                          double jerk, double time_step);

  static std::vector<TrajectoryPoint> GenerateTrajByPathAndSpeedData(
      const DiscretizedPath &av_path, const SpeedVector &av_speed_profile);

  static std::map<SpeedLimitTypeProto::Type, SpeedLimit> GetSpeedLimitMap(
      const DiscretizedPath &discretized_points, double max_speed_limit,
      double av_speed, const VehicleGeometryParamsProto &veh_geo_params,
      const VehicleDriveParamsProto &veh_drive_params,
      const DrivePassage &drive_passage);

  static SpeedLimit GenerateLaneSpeedLimit(const DiscretizedPath &path_points,
                                           double max_speed_limit,
                                           double av_speed,
                                           const DrivePassage &drive_passage);

  static SpeedLimit GenerateCurvatureSpeedLimit(
      const DiscretizedPath &path_points,
      const VehicleDriveParamsProto &veh_drive_params,
      const VehicleGeometryParamsProto &veh_geo_params, double max_speed_limit,
      double av_speed);

  static SpeedLimit GenerateCombinationSpeedLimit(
      const std::map<SpeedLimitTypeProto::Type, SpeedLimit> &speed_limit_map,
      double max_speed_limit);

  static absl::Status CombinePathAndSpeed(
      const DiscretizedPath &path_data, bool forward,
      const SpeedVector &speed_data,
      std::vector<ApolloTrajectoryPointProto> *trajectory);

  static absl::Status ExtendTrajectoryLength(
      const DiscretizedPath &path_data, bool forward,
      const SpeedVector &speed_data,
      std::vector<ApolloTrajectoryPointProto> *trajectory);
  static std::optional<double> FindYawByT(
      const std::vector<TrajectoryPoint> &traj, double t);

  static void ModifyFollowTrajAndSimResult(
      const DiscretizedPath *ego_path, const DrivePassage *drive_passage,
      double time_horizon, double time_step,
      SpacetimeTrajectoryManager *const modified_traj_mgr,
      std::unordered_map<std::string, GamingSimResult>
          *const follow_sim_results);

  static void ExtendPathAlongDrivePassage(const DrivePassage &drive_passage,
                                          double length, DiscretizedPath *path);
};
}  // namespace st::planning
