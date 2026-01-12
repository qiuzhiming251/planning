/**
 * @file speed_obstacle_processor.h
 * @author zhang.xing (zhang.xing43@byd.com)
 * @brief
 * @version 0.1
 * @date 2025-03-19
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "plan_common/util/vehicle_geometry_util.h"
#include "speed_gaming_common.h"
#include "plan_common/maps/st_boundary_with_decision.h"

namespace st::planning {

class SpeedObstacleProcessor {
 public:
  SpeedObstacleProcessor() = default;
  ~SpeedObstacleProcessor() = default;

  void Init(const SpacetimeTrajectoryManager *traj_mgr,
            const DiscretizedPath *ego_path, const DrivePassage *drive_passage,
            const VehicleGeometryParamsProto *vehicle_geo_params) {
    traj_mgr_ = traj_mgr;
    ego_path_ = ego_path;
    drive_passage_ = drive_passage;
    vehicle_geo_params_ = vehicle_geo_params;
  }

  void PreProcess(std::unordered_map<std::string, GamingConflictZoneInfo>
                      &conflict_zone_infos,
                  SpacetimeTrajectoryManager *const modified_traj_mgr);
  void ConvertStBoundaryToConflictZone(
      std::unordered_map<std::string, GamingConflictZoneInfo>
          &conflict_zone_infos,
      const std::vector<StBoundaryWithDecision> &st_boundaries_with_decision,
      const std::unordered_map<std::string, SpacetimeObjectTrajectory>
          &processed_st_objects,
      const absl::flat_hash_set<std::string> &gaming_lc_obs_set,
      const int &plan_id);
  void PostPrecess(
      const SpacetimeTrajectoryManager *traj_mgr,
      const std::unordered_map<std::string, SpacetimeObjectTrajectory>
          *processed_st_objects,
      const std::unordered_set<std::string> &ignore_objs,
      const std::unordered_map<std::string, GamingSimResult>
          &follow_sim_results,
      const std::unordered_map<std::string, GamingSimResult> &yield_sim_results,
      const std::unordered_map<std::string, GamingSimResult>
          &overtake_sim_results,
      std::unordered_map<std::string, SpacetimeObjectTrajectory>
          *const gaming_processed_st_objects,
      std::unordered_map<std::string, StBoundaryProto::DecisionType>
          *const gaming_decision,
      std::unordered_map<std::string, RiskObjInfo>
          *const gaming_risk_field_zoneinfos);
  void ConvertStBoundaryToSimResult(
      const StBoundary &st_boundary,
      const std::unordered_map<std::string, SpacetimeObjectTrajectory>
          &processed_st_objects,
      const SpacetimeTrajectoryManager &traj_mgr,
      const SpeedGamingParams &speed_gaming_params,
      GamingSimResult &yield_sim_result);
  void GenerateConflictZoneForOneAgent(
      GamingConflictZoneInfo &conflict_zone_info,
      const SpacetimeObjectTrajectory *obj_traj, bool is_static);
  void GenerateYieldConflictZoneForLaneChangeSafety(
      std::unordered_map<std::string, GamingConflictZoneInfo>
          &conflict_zone_infos);

  SpacetimeObjectTrajectory GenerateSpacetimeObjectTrajectory(
      const std::unordered_map<std::string, SpacetimeObjectTrajectory>
          *processed_st_objects,
      const SpacetimeTrajectoryManager *traj_mgr,
      const GamingSimResult &sim_result, const std::string &id);

  void RiskFieldGenerate(
      std::unordered_map<std::string, GamingConflictZoneInfo>
          &conflict_zone_infos,
      const std::vector<StBoundaryWithDecision> &st_boundaries_with_decision);

 private:
  std::optional<std::pair<double, double>> ConvertToOverlapRange(
      absl::Span<const AgentOverlap> agent_overlaps);
  InteractionType AnalyzeInteractionType(
      const prediction::PredictionObjectState &last_overlap_obj_state,
      int last_overlap_index);
  InteractionType InteractionClassification(
      const prediction::PredictionObjectState &last_overlap_obj_state,
      double last_overlap_ego_s);
  InteractionType InteractionClassification(
      absl::Span<const prediction::PredictionObjectState> states);
  InteractionType InteractionClassificationByDrivePassage(
      absl::Span<const prediction::PredictionObjectState> states);
  void RiskFieldConflictCheck(
      const DiscretizedPath *ego_path,
      const std::vector<Polygon2d> *agent_polygons,
      const std::vector<Box2d> *agent_boxs, const double &ego_half_width,
      const double &l0, const double &s_coeff, const double &theta_coeff,
      std::optional<size_t> &first_conflict_agent_idx,
      std::optional<size_t> &last_conflict_agent_idx,
      PathPoint &first_conflict_ego_point, PathPoint &last_conflict_ego_point,
      std::optional<double> &risk_field_lat_ratio,
      std::vector<std::pair<double, double>> &left_risk_edge,
      std::vector<std::pair<double, double>> &right_risk_edge);

  void PathConflictCheck(
      const SpacetimeObjectTrajectory *st_traj, const double &search_radius,
      const PathApprox &ego_path_approx, const double &path_step_length,
      std::optional<std::pair<double, double>> &first_overlap_ego_s_range,
      int &first_overlap_obj_index,
      std::optional<std::pair<double, double>> &last_overlap_ego_s_range,
      int &last_overlap_obj_index);

  void SimplifyPathConflictCheck(
      const DiscretizedPath *ego_path,
      const SpacetimeObjectTrajectory *agent_traj, bool is_static,
      std::optional<std::pair<double, double>> &ego_s_range,
      std::optional<std::pair<int, int>> &agent_index_range);

  void RefinePredTraj(const SpacetimeTrajectoryManager &origin_traj_mgr,
                      SpacetimeTrajectoryManager *const modified_traj_mgr);
  std::pair<double, double> GetProjectionDistance(const PathPoint &ego_pose,
                                                  const Polygon2d &contour,
                                                  const int &plan_id);

 private:
  const SpacetimeTrajectoryManager *traj_mgr_;
  const DiscretizedPath *ego_path_;
  const DrivePassage *drive_passage_;
  const VehicleGeometryParamsProto *vehicle_geo_params_;
  std::unique_ptr<PathApprox> path_approx_;
  FrenetBox ego_box_sl_;
};
}  // namespace st::planning
