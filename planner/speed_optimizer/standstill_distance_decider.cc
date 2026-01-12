

#include "planner/speed_optimizer/standstill_distance_decider.h"

#include <algorithm>
#include <optional>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "plan_common/maps/st_boundary.h"
#include "plan_common/util/lane_path_util.h"

namespace st {
namespace planning {
namespace {

inline bool IsStBoundaryStalledObject(
    const StBoundary& st_boundary,
    const absl::flat_hash_set<std::string>& stalled_object_ids) {
  CHECK(st_boundary.traj_id().has_value());
  const auto obj_id = SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(
      *st_boundary.traj_id());
  return stalled_object_ids.contains(obj_id);
}

std::optional<Box2d> BuildBoomBarrierBoxOr(
    const PlannerSemanticMapManager* psmm, const mapping::LanePath& lane_path) {
  constexpr double kForwardDistance = 50.0;      // m.
  constexpr double kBoomBarrierBoxLength = 1.0;  // m.
  constexpr double kBoomBarrierBoxWidth = 2.0;   // m.
  const auto lanes_info = GetLanesInfoContinueIfNotFound(
      *psmm, lane_path.BeforeArclength(kForwardDistance));
  for (const auto& lane_info : lanes_info) {
    if (lane_info->endpoint_toll()) {
      CHECK_GE(lane_info->points().size(), 2);
      const auto& points = lane_info->points();
      const Vec2d end_vec(points.back() - points[points.size() - 2]);
      return Box2d(points.back(), end_vec.FastAngle(), kBoomBarrierBoxLength,
                   kBoomBarrierBoxWidth);
    }
  }
  return std::nullopt;
}

// BANDAID(ping): This is a hack to identify a gate boom barrier.
bool IsStaticStBoundaryBoomBarrier(
    const StBoundary& st_boundary, const PlannerSemanticMapManager* psmm,
    const mapping::LanePath& lane_path,
    const SpacetimeTrajectoryManager& st_traj_mgr) {
  return false;
  const auto barrier_box = BuildBoomBarrierBoxOr(psmm, lane_path);
  if (!barrier_box.has_value()) return false;
  CHECK(st_boundary.traj_id().has_value());
  const auto* obj =
      CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*st_boundary.traj_id()));
  return obj->contour().HasOverlap(*barrier_box);
}

// return: {follow_standstill_distance, lead_standstill_distance}.
std::pair<double, double> GetStBoundaryStandStillDistance(
    double plan_start_v, const StBoundary& st_boundary,
    const SpeedFinderParamsProto& speed_finder_params,
    const absl::flat_hash_set<std::string>& stalled_object_ids,
    const PlannerSemanticMapManager* psmm, const mapping::LanePath* lane_path,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const ConstraintManager& constraint_mgr,
    double extra_follow_standstill_for_large_vehicle, bool is_open_gap,
    const CipvObjectInfo* cipv_object_info) {
  double follow_standstill_distance = 0.0;
  double lead_standstill_distance = 0.0;
  switch (st_boundary.object_type()) {
    case StBoundaryProto::VEHICLE:
    case StBoundaryProto::CYCLIST:
    case StBoundaryProto::PEDESTRIAN: {
      if (st_boundary.is_stationary()) {
        if (IsStBoundaryStalledObject(st_boundary, stalled_object_ids)) {
          follow_standstill_distance =
              speed_finder_params.follow_standstill_distance_for_stalled_obj();
        } else {
          follow_standstill_distance =
              speed_finder_params.follow_standstill_distance();
        }
      } else {
        follow_standstill_distance =
            speed_finder_params.follow_standstill_distance();
      }

      CHECK(st_boundary.object_id().has_value());
      const auto& object_id = *st_boundary.object_id();
      if (is_open_gap && nullptr != cipv_object_info) {
        const auto& cipv_id = cipv_object_info->nearest_object_id;
        if (cipv_id.has_value() && *cipv_id == object_id) {
          follow_standstill_distance =
              speed_finder_params.gap_expand_follow_standstill();
        }
      }

      // large vehicle
      if (st_boundary.is_large_vehicle()) {
        follow_standstill_distance += extra_follow_standstill_for_large_vehicle;
        follow_standstill_distance = std::max(
            follow_standstill_distance,
            speed_finder_params.follow_standstill_distance_for_large_obj());
      }

      lead_standstill_distance = speed_finder_params.lead_standstill_distance();
      break;
    }
    case StBoundaryProto::STATIC: {
      if (lane_path != nullptr &&
          IsStaticStBoundaryBoomBarrier(st_boundary, psmm, *lane_path,
                                        st_traj_mgr)) {
        constexpr double kTollStandstillDist = 1.5;  // m.
        follow_standstill_distance = kTollStandstillDist;
      } else {
        follow_standstill_distance =
            speed_finder_params.follow_standstill_distance_for_static_obj();
      }
      lead_standstill_distance = speed_finder_params.lead_standstill_distance();
      break;
    }
    case StBoundaryProto::IMPASSABLE_BOUNDARY: {
      follow_standstill_distance =
          speed_finder_params.follow_standstill_distance_for_curb();
      lead_standstill_distance = 0.0;
      break;
    }
    case StBoundaryProto::PATH_BOUNDARY: {
      constexpr double kPathBoundaryStandstillDist = 1.0;
      follow_standstill_distance = kPathBoundaryStandstillDist;
      lead_standstill_distance = 0.0;
      break;
    }
    case StBoundaryProto::VIRTUAL: {
      const auto stop_line_it = std::find_if(
          constraint_mgr.StopLine().begin(), constraint_mgr.StopLine().end(),
          [&st_boundary](const ConstraintProto::StopLineProto& stopline) {
            return stopline.id() == st_boundary.id();
          });
      const auto path_stop_line_it = std::find_if(
          constraint_mgr.PathStopLine().begin(),
          constraint_mgr.PathStopLine().end(),
          [&st_boundary](const ConstraintProto::PathStopLineProto& stopline) {
            return stopline.id() == st_boundary.id();
          });
      CHECK(stop_line_it != nullptr || path_stop_line_it != nullptr);
      follow_standstill_distance = stop_line_it ? stop_line_it->standoff()
                                                : path_stop_line_it->standoff();
      constexpr double kFullStopSpeedThres = 0.2;  // m/s.
      if (st_boundary.is_traffic_light() &&
          plan_start_v < kFullStopSpeedThres) {
        constexpr double kFullStopDeadZone = 0.5;  // m.
        follow_standstill_distance += kFullStopDeadZone;
      }
      lead_standstill_distance = 0.0;
      break;
    }
    case StBoundaryProto::IGNORABLE:
    case StBoundaryProto::UNKNOWN_OBJECT: {
      LOG_FATAL << "Unpexted "
                << StBoundaryProto::ObjectType_Name(st_boundary.object_type())
                << " st-boundary " << st_boundary.id();
      break;
    }
  }

  return std::make_pair(follow_standstill_distance, lead_standstill_distance);
}

}  // namespace

void DecideStandstillDistanceForStBoundary(
    const StandstillDistanceDeciderInput& input,
    StBoundaryWithDecision* st_boundary_wd, bool is_open_gap,
    const CipvObjectInfo* cipv_object_info) {
  CHECK_NOTNULL(input.speed_finder_params);
  CHECK_NOTNULL(input.stalled_object_ids);
  CHECK_NOTNULL(input.st_traj_mgr);
  CHECK_NOTNULL(input.constraint_mgr);
  CHECK_NOTNULL(st_boundary_wd);
  const auto& st_boundary = *st_boundary_wd->raw_st_boundary();
  const double extra_standstill = PiecewiseLinearFunctionFromProto(
      input.speed_finder_params
          ->extra_follow_standstill_distance_for_large_vehicle_plf())(
      input.plan_start_v);
  const auto standstill_distance = GetStBoundaryStandStillDistance(
      input.plan_start_v, st_boundary, *input.speed_finder_params,
      *input.stalled_object_ids, input.planner_semantic_map_manager,
      input.lane_path, *input.st_traj_mgr, *input.constraint_mgr,
      extra_standstill, is_open_gap, cipv_object_info);
  st_boundary_wd->set_follow_standstill_distance(standstill_distance.first);
  st_boundary_wd->set_lead_standstill_distance(standstill_distance.second);

  return;
}

}  // namespace planning
}  // namespace st
