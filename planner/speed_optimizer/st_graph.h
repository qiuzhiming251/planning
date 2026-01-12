

#ifndef ST_PLANNING_SPEED_ST_GRAPH
#define ST_PLANNING_SPEED_ST_GRAPH

#include <limits>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "plan_common/async/thread_pool.h"
#include "plan_common/path_approx.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/type_def.h"
#include "plan_common/vehicle_shape.h"
#include "plan_common/constraint_manager.h"
#include "gflags/gflags.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/segment_matcher/segment_matcher_kdtree.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "predictor/prediction_object_state.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"
#include "planner/speed_optimizer/slt_info.h"
#include "plan_common/maps/st_boundary.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "planner/speed_optimizer/st_close_trajectory.h"
#include "planner/speed_optimizer/st_graph_defs.h"
// #include "plan_common/util/hmi_content_util.h"
// #include "object_manager/planner_object.h"

DECLARE_bool(planner_use_path_approx_based_st_mapping);
DECLARE_bool(planner_use_nearest_path_boundary_st_mapping);

namespace st::planning {

class StGraph {
 public:
  struct AgentNearestPoint {
    double ra_s = 0.0;
    double ra_heading = 0.0;
    double t = 0.0;
    double obj_v = 0.0;
    double obj_heading = 0.0;
    double lat_dist = 0.0;
    int obj_idx = 0;
  };

  struct StBoundaryOutput {
    std::vector<StBoundaryRef> st_boundaries;
    std::vector<SltInfoRef> slt_infos;
    std::optional<double> gap_desired_a;
  };

  StGraph(int plan_id, const DiscretizedPath* path_points, int traj_steps,
          double plan_start_v, double plan_start_a, double max_decel,
          const VehicleGeometryParamsProto* vehicle_geo_params,
          const SpeedFinderParamsProto::StGraphParamsProto* st_graph_params,
          const std::vector<VehicleShapeBasePtr>* av_shape_on_path_points,
          const SegmentMatcherKdtree* path_kd_tree,
          const PathApprox* path_approx,
          const PathApprox* path_approx_for_mirrors);

  StBoundaryOutput GetStBoundaries(
      const SpacetimeTrajectoryManager& traj_mgr,
      const std::map<std::string, ConstraintProto::LeadingObjectProto>&
          leading_objs,
      bool consider_lane_change_gap, const ConstraintManager& constraint_mgr,
      const PlannerSemanticMapManager* psman_mgr,
      const DrivePassage* drive_passage, const PathSlBoundary* path_sl_boundary,
      const NudgeObjectInfo* nudge_object_info, ThreadPool* thread_pool);

  // For moving object, this function will only check and return the first
  // point(which is the object current state) if it is within the
  // slow_down_radius.
  std::vector<CloseSpaceTimeObject> GetCloseSpaceTimeObjects(
      absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
      absl::Span<const SpacetimeObjectTrajectory* const> spacetime_object_trajs,
      double slow_down_radius) const;

  StBoundaryOutput MapMovingSpacetimeObject(
      const SpacetimeObjectTrajectory& spacetime_object,
      bool generate_lane_change_gap, bool calc_moving_close_traj,
      const NudgeObjectInfo* nudge_object_info) const;

  /**
   * @brief Return a vector which contains the distance info on each path
   * point before collision with impassable boundaries.
   */
  const std::vector<DistanceInfo>& distance_info_to_impassable_boundaries()
      const {
    return distance_info_to_impassable_boundaries_;
  }

  const std::vector<StCloseTrajectory>& moving_close_trajs() const {
    return moving_close_trajs_;
  }

  const std::map<std::string, ObjectSlInfo>& obj_sl_map() const {
    return obj_sl_map_;
  }

  StBoundaryRef MapPathStopLine(
      const ConstraintProto::PathStopLineProto& stop_line) const;

  StBoundaryRef MapLeadingObject(
      const SpacetimeObjectTrajectory& spacetime_object,
      const DrivePassage& drive_passage,
      const PathSlBoundary& path_sl_boundary) const;

 private:
  std::vector<StBoundaryRef> MapStationarySpacetimeObjects(
      absl::Span<const SpacetimeObjectTrajectory* const>
          stationary_spacetime_objs,
      const std::map<std::string, ConstraintProto::LeadingObjectProto>&
          leading_objs) const;

  StBoundaryRef MapStopLine(const ConstraintProto::StopLineProto& stop_line,
                            const DrivePassage* drive_passage) const;

  StBoundaryRef MapNearestImpassableBoundary(
      const PlannerSemanticMapManager& psman_mgr,
      const DrivePassage* drive_passage);

  StBoundaryRef MapNearestPathBoundary(
      const DrivePassage& drive_passage,
      const PathSlBoundary& path_sl_boundary) const;

  // Generate possibly multiple st-boundaries for a moving object.
  std::vector<StBoundaryPoints> GetMovingObjStBoundaryPoints(
      const SpacetimeObjectTrajectory& spacetime_object,
      bool generate_lane_change_gap, bool calc_moving_close_traj,
      std::vector<NearestSlPoint>* const near_sl_points,
      const NudgeObjectInfo* nudge_object_info) const;

  // Generate single st-boundary for a stationary object.
  std::optional<StBoundaryPoints> GetStationaryObjStBoundaryPoints(
      const SpacetimeObjectTrajectory& spacetime_object,
      const double max_kappa = 0.0) const;

  bool FindOverlapRangeOrNearestPoint(
      const Vec2d& search_point, double search_radius,
      double search_radius_for_mirrors,
      const prediction::PredictionObjectState& obj_state, int obj_idx,
      bool consider_mirrors, double lat_buffer, double lon_buffer, int* low_idx,
      int* high_idx,
      std::vector<AgentNearestPoint>* agent_nearest_points) const;

  void GenerateLatInflatedStBoundary(
      const SpacetimeObjectTrajectory& spacetime_object,
      StBoundaryProto::ProtectionType protection_type, double lat_buffer,
      std::vector<StBoundaryPoints>* st_boundaries_points) const;

  std::optional<std::pair<int, int>> FindSegment2dOverlapRange(
      const Segment2d& segment) const;

  std::optional<std::pair<int, int>> FindStopLine2dOverlapRange(
      const Segment2d& segment, const bool is_extended) const;

  std::optional<std::pair<int, int>> FindStopLineOverlapRange(
      const ConstraintProto::StopLineProto& stop_line) const;

  std::optional<int> FindLeadingObjectLowerIndex(
      const Polygon2d& obj_shape, const DrivePassage& drive_passage,
      const PathSlBoundary& path_sl_boundary) const;

  bool GetStDistancePointInfo(const prediction::PredictionObjectState& state,
                              double slow_down_radius,
                              StDistancePoint* st_distance_point) const;

  void CalculateObjectSlPosition(const DrivePassage& drive_passage,
                                 const SpacetimeTrajectoryManager& traj_mgr);

  std::pair<double, double> GetStBoundaryTRangeByProtectionType(
      const SpacetimeObjectTrajectory& spacetime_object,
      StBoundaryProto::ProtectionType protection_type) const;

  void GenerateProtectiveStBoundaries(
      const SpacetimeObjectTrajectory& spacetime_object,
      absl::Span<const AgentNearestPoint> agent_nearest_points,
      const NudgeObjectInfo* nudge_object_info, bool generate_lane_change_gap,
      std::vector<StBoundaryPoints>* st_boundaries_points) const;

  void PostProcessFollowerGapStBoundary(
      const std::string& object_id, double object_v,
      std::vector<StBoundaryPoints>* st_boundaries_points) const;

  absl::StatusOr<double> ComputeDesiredGapAccel(
      const PlannerObject* leader_object, const PlannerObject* follower_object,
      const ObjectSlInfo* leader_sl, const ObjectSlInfo* follower_sl) const;

  StBoundaryRef GenerateGapStBoundaryByDesiredAccel(
      const PlannerObject& target_gap_object,
      const SpacetimeObjectTrajectory& target_gap_traj, bool is_follower,
      double desired_a);

  absl::StatusOr<StBoundaryRef> GenerateGapStBoundary(
      const SpacetimeTrajectoryManager& traj_mgr,
      const std::optional<std::string>& leader_id,
      const std::optional<std::string>& follower_id, double* gap_desired_a);

 private:
  int plan_id_ = 0;
  double total_plan_time_ = 0.0;
  double plan_start_v_ = 0.0;
  double plan_start_a_ = 0.0;
  double max_decel_ = 0.0;
  const DiscretizedPath* path_points_;
  // The SDC shapes on path points.
  const std::vector<VehicleShapeBasePtr>* av_shape_on_path_points_;
  const SegmentMatcherKdtree* path_kd_tree_;
  double ego_radius_ = 0.0;
  double ego_radius_for_mirrors_ = 0.0;
  double min_mirror_height_avg_ = std::numeric_limits<double>::infinity();
  double max_mirror_height_avg_ = -std::numeric_limits<double>::infinity();

  const VehicleGeometryParamsProto* vehicle_geo_params_;
  const SpeedFinderParamsProto::StGraphParamsProto* st_graph_params_;
  std::vector<DistanceInfo> distance_info_to_impassable_boundaries_;
  const PathApprox* path_approx_;
  const PathApprox* path_approx_for_mirrors_;
  mutable std::vector<StCloseTrajectory> moving_close_trajs_;
  std::map<std::string, ObjectSlInfo> obj_sl_map_;  // {id, {ds, dl}}
  TrafficGapResult traffic_gap_;

  mutable absl::Mutex mutex_;
};

}  // namespace st::planning

#endif  // ST_PLANNING_SPEED_ST_GRAPH
