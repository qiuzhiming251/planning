

#include "planner/speed_optimizer/constraint_generator.h"

// IWYU pragma: no_include <memory>
//              for allocator_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include "absl/container/flat_hash_map.h"
//#include "global/logging.h"
//#include "global/trace.h"
//#include "lite/check.h"
#include "plan_common/math/geometry/halfplane.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "object_manager/planner_object.h"
#include "plan_common/second_order_trajectory_point.h"
#include "planner/speed_optimizer/decider/close_object_slowdown_decider.h"
#include "planner/speed_optimizer/path_semantic_analyzer.h"
#include "plan_common/maps/st_boundary.h"
#include "plan_common/util/path_util.h"

namespace st {
namespace planning {

SpeedConstraintGeneratorOutput GenerateStationaryCloseObjectConstraints(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const StGraph& st_graph, const SpacetimeTrajectoryManager& traj_mgr,
    const DiscretizedPath& path, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary, double av_speed,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const bool is_on_highway) {
  constexpr double kCloseObjectSlowdownMaxDistance = 1.5;  // m
  SpeedConstraintGeneratorOutput output;
  const auto close_spacetime_objects = st_graph.GetCloseSpaceTimeObjects(
      st_boundaries_with_decision, traj_mgr.stationary_object_trajs(),
      kCloseObjectSlowdownMaxDistance);
  if (close_spacetime_objects.empty()) return output;
  output.path_speed_regions = MakeCloseObjectSlowdownDecision(
      close_spacetime_objects, drive_passage, path, av_speed, path_sl_boundary,
      vehicle_geometry_params, is_on_highway);
  return output;
}

SpeedConstraintGeneratorOutput GenerateDenseTrafficFlowConstraint(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& traj_mgr,
    const std::vector<PathPointSemantic>& path_semantics,
    const DiscretizedPath& path, double plan_start_v,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  SpeedConstraintGeneratorOutput output;
  constexpr double kAvSpeedThres = 2.0;  // m/s.
  if (plan_start_v > kAvSpeedThres) return output;

  if (path_semantics.empty()) return output;
  const auto current_lane_semantic = path_semantics.front().lane_semantic;
  if (current_lane_semantic == LaneSemantic::NONE ||
      current_lane_semantic == LaneSemantic::ROAD ||
      current_lane_semantic == LaneSemantic::INTERSECTION_STRAIGHT) {
    return output;
  }

  constexpr double kMaxOverlapMinS = 10.0;          // m.
  constexpr double kThetaDiffThres = M_PI_2 / 3.0;  // 30 deg.
  // first: object_id, second: min_t
  absl::flat_hash_map<std::string, double> objects_in_range;
  for (const StBoundaryWithDecision& stb_wd : st_boundaries_with_decision) {
    const StBoundary* raw_stb = stb_wd.st_boundary();
    if (raw_stb->source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    // only consider the objects before the vehicle
    if (raw_stb->min_s() < kMaxOverlapMinS && raw_stb->min_s() > 0) {
      const auto object_id = raw_stb->object_id();
      CHECK(object_id.has_value());
      const auto* obj =
          CHECK_NOTNULL(traj_mgr.FindObjectByObjectId(*object_id));
      const double theta_diff =
          std::fabs(NormalizeAngle(path.front().theta() - obj->pose().theta()));
      if (theta_diff > kThetaDiffThres) {
        if (!objects_in_range.contains(*object_id) ||
            raw_stb->min_t() < objects_in_range[*object_id]) {
          objects_in_range[*object_id] = raw_stb->min_t();
        }
      }
    }
  }

  double objects_min_t = std::numeric_limits<double>::max();
  for (const auto& [_, min_t] : objects_in_range) {
    objects_min_t = std::min(objects_min_t, min_t);
  }

  constexpr int kDenseTrafficThres = 3;
  constexpr double kMinTThres = 5.0;  // s.
  if (objects_in_range.size() > kDenseTrafficThres &&
      objects_min_t < kMinTThres) {
    constexpr double kStopTime = 1.0;  // s.
    const double dist_to_front_edge = plan_start_v * kStopTime;
    const PathPoint center_pt = path.Evaluate(
        dist_to_front_edge + vehicle_geometry_params.front_edge_to_center());
    const Vec2d center = ToVec2d(center_pt);
    const Vec2d unit = Vec2d::FastUnitFromAngle(center_pt.theta());
    constexpr double kHalfWidth = 4.5;  // m.
    const HalfPlane fence(center - kHalfWidth * unit.Perp(),
                          center + kHalfWidth * unit.Perp());
    auto& path_stop_line = output.path_stop_lines.emplace_back();
    constexpr char kId[] = "dense_traffic_flow";
    fence.ToProto(path_stop_line.mutable_half_plane());
    path_stop_line.set_id(kId);
    path_stop_line.set_standoff(0.0);
    path_stop_line.set_s(dist_to_front_edge);
    path_stop_line.mutable_source()->mutable_dense_traffic_flow()->set_id(kId);
  }
  return output;
}

}  // namespace planning
}  // namespace st
