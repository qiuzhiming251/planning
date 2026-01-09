

#ifndef ONBOARD_PATH_BOUNDARY_BUILDER_HELPER_H_
#define ONBOARD_PATH_BOUNDARY_BUILDER_HELPER_H_

#include <algorithm>
#include <utility>
#include <vector>

#include "absl/types/span.h"

//#include "global/buffered_logger.h"
//#include "lite/check.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"
#include "plan_common/ref_line/smooth_reference_line_result.h"
#include "decider/scheduler/path_boundary_builder.h"

namespace st::planning {

class PathBoundary {
 public:
  PathBoundary(std::vector<double> right, std::vector<double> left)
      : right_(std::move(right)), left_(std::move(left)) {}

  const std::vector<double>& right_vec() const { return right_; }

  const std::vector<double>& left_vec() const { return left_; }

  std::vector<double>* mutable_right_vec() { return &right_; }

  std::vector<double>* mutable_left_vec() { return &left_; }

  // The following two function will move the member to the caller.
  std::vector<double>&& moved_left_vec() { return std::move(left_); }
  std::vector<double>&& moved_right_vec() { return std::move(right_); }

  double right(int i) const { return right_[i]; }

  double left(int i) const { return left_[i]; }

  void ExtendLeftTo(double uniform_left) {
    for (auto& left_l : left_) {
      left_l = std::max(left_l, uniform_left);
    }
  }

  void ExtendRightTo(double uniform_right) {
    for (auto& right_l : right_) {
      right_l = std::min(right_l, uniform_right);
    }
  }

  void ShiftLeftBy(double offset) {
    for (auto& left_l : left_) {
      left_l += offset;
    }
  }

  void ShiftRightBy(double offset) {
    for (auto& right_l : right_) {
      right_l += offset;
    }
  }

  void ShiftLeftByIndex(int index, double offset) { left_[index] += offset; }

  void ShiftRightByIndex(int index, double offset) { right_[index] += offset; }

  void OuterClampRightByIndex(int index, double right_l) {
    right_[index] = std::max(right_[index], right_l);
  }

  void OuterClampLeftByIndex(int index, double left_l) {
    left_[index] = std::min(left_[index], left_l);
  }

  void InnerClampRightByIndex(int index, double right_l) {
    right_[index] = std::min(right_[index], right_l);
  }

  void InnerClampLeftByIndex(int index, double left_l) {
    left_[index] = std::max(left_[index], left_l);
  }

  void OuterClampBy(const PathBoundary& other) {
    const int n = static_cast<int>(left_.size());
    CHECK_EQ(n, right_.size());
    CHECK_LE(n, other.right_vec().size());
    for (int i = 0; i < n; ++i) {
      left_[i] = std::min(left_[i], other.left(i));
      right_[i] = std::max(right_[i], other.right(i));
    }
  }

  void SoftOuterClampBy(const PathBoundary& other,
                        const std::pair<int, int>& left_split_range,
                        const std::pair<int, int>& right_split_range) {
    const int n = static_cast<int>(left_.size());
    CHECK_EQ(n, right_.size());
    CHECK_LE(n, other.right_vec().size());
    const double ramp_factor = 0.25;
    double last_left = 0.0, last_right = 0.0;
    for (int i = 0; i < n; ++i) {
      double left_cmp =
          right_split_range.first > 0 && i >= right_split_range.first &&
                  i <= right_split_range.second && left_[i] > other.left(i) &&
                  left_[i] < last_left + 0.06
              ? std::fmax(left_[i - 1] - ramp_factor, other.left(i))
              : other.left(i);
      double right_cmp =
          left_split_range.first > 0 && i >= left_split_range.first &&
                  i <= left_split_range.second && right_[i] < other.left(i) &&
                  right_[i] > last_right - 0.06
              ? std::fmin(right_[i - 1] + ramp_factor, other.right(i))
              : other.right(i);
      last_left = left_[i];
      last_right = right_[i];
      left_[i] = std::fmin(left_[i], left_cmp);
      right_[i] = std::fmax(right_[i], right_cmp);
    }
  }

  void InnerClampBy(const PathBoundary& other) {
    const int n = static_cast<int>(left_.size());
    CHECK_EQ(n, right_.size());
    CHECK_LE(n, other.size());
    for (int i = 0; i < n; ++i) {
      left_[i] = std::max(left_[i], other.left(i));
      right_[i] = std::min(right_[i], other.right(i));
    }
  }

  void EraseFrom(int index) {
    CHECK_LT(index, size());
    left_.resize(index);
    right_.resize(index);
  }

  int size() const { return left_.size(); }

 private:
  std::vector<double> right_;
  std::vector<double> left_;
};

bool IsTurningLanePath(const PlannerSemanticMapManager& psmm,
                       mapping::ElementId lane_id);

PathBoundary BuildPathBoundaryFromTargetLane(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const double& ego_v, const std::pair<int, int>& left_split_range,
    const std::pair<int, int>& right_split_range, double min_half_lane_width,
    bool borrow_lane_boundary, const FrenetBox& sl_box = FrenetBox());

PathBoundary BuildCurbPathBoundary(const DrivePassage& drive_passage);

PathBoundary BuildSolidPathBoundary(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const FrenetCoordinate& cur_sl,
    const VehicleGeometryParamsProto& vehicle_geom,
    const ApolloTrajectoryPointProto& plan_start_point,
    const LaneChangeStateProto& lc_state, double target_lane_offset,
    const std::vector<double>& center_l);

PathBoundary BuildPathBoundaryFromAvKinematics(
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const FrenetCoordinate& cur_sl, const FrenetBox& sl_box,
    const LaneChangeStateProto& lc_state, absl::Span<const double> s_vec,
    double target_lane_offset, double max_lane_change_lat_accel,
    bool lane_change_pause);

PathBoundary ShrinkPathBoundaryForLaneChangePause(
    const VehicleGeometryParamsProto& vehicle_geom, const FrenetBox& sl_box,
    const LaneChangeStateProto& lc_state, PathBoundary boundary,
    double target_lane_offset, const std::vector<double>& center_l);

PathBoundary ExtendPathBoundaryForLaneChangePause(
    const LaneChangeStateProto& lc_state, const double half_lane_width,
    PathBoundary boundary);

PathBoundary ShrinkPathBoundaryForObject(
    const DrivePassage& drive_passage,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const ApolloTrajectoryPointProto& plan_start_point,
    absl::Span<const double> s_vec, absl::Span<const double> center_l,
    absl::Span<const Vec2d> center_xy, const PathBoundary& inner_boundary,
    const PathBoundary& curb_boundary, PathBoundary boundary,
    const PlannerSemanticMapManager& psmm, const FrenetBox& sl_box);

bool ObstsacleFilter(const SpacetimeObjectTrajectory* traj,
                     const DrivePassage& drive_passage,
                     const ApolloTrajectoryPointProto& plan_start_point,
                     const PlannerSemanticMapManager& psmm);

double ComputeTargetLaneOffset(
    const DrivePassage& drive_passage, const FrenetCoordinate& cur_sl,
    const LaneChangeStateProto& lc_state,
    const ApolloTrajectoryPointProto& plan_start_point,
    absl::Span<const SpacetimeObjectTrajectory> obj_trajs,
    const FrenetBox& sl_box, const VehicleGeometryParamsProto& vehicle_geom,
    bool is_congestion_scene = false,
    PausePushSavedOffsetProto* saved_offset = nullptr,
    absl::flat_hash_set<std::string>* unsafe_object_ids = nullptr, 
    const ObjectHistoryManager *obj_history_mgr = nullptr);

absl::StatusOr<bool> KinematicPauseOffsetSimulation(
    const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    double& pause_kinematic_offset, bool is_left);

PathSlBoundary BuildPathSlBoundary(
    const DrivePassage& drive_passage, std::vector<double> s_vec,
    std::vector<double> ref_center_l, PathBoundary inner_boundary,
    PathBoundary outer_boundary, PathBoundary opt_outer_boundary,
    CenterLineOffsetType offset_type =
        CenterLineOffsetType::CENTER_LINE_OFFSET_UNKNOWN,
    double offset_value = 0.0);

std::vector<double> ComputeSmoothedReferenceLine(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const SmoothedReferenceLineResultMap& smooth_result_map);

}  // namespace st::planning

#endif  // ONBOARD_PATH_BOUNDARY_BUILDER_HELPER_H_
