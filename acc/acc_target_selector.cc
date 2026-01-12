#include "acc/acc_target_selector.h"

#include <algorithm>
#include <limits>
#include <memory>

#include "plan_common/timer.h"
#include "plan_common/util/path_util.h"
#include "plan_common/util/vehicle_geometry_util.h"

namespace st::planning {
namespace {

bool IsAccConsideredObject(ObjectType type) {
  switch (type) {
    case OT_UNKNOWN_STATIC:
    case OT_VEHICLE:
    case OT_MOTORCYCLIST:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
    case OT_UNKNOWN_MOVABLE:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
    case OT_TRICYCLIST:
    case OT_LARGE_VEHICLE:
      return true;
    case OT_FOD:
    case OT_VEGETATION:
      return false;
    default:
      return false;
      break;
  }
  return false;
}

double ComputeObjectInvasionRate(const PathSlBoundary& path_boundary,
                                 const FrenetFrame& path_frenet,
                                 const Box2d& obj_bounding_box,
                                 const FrenetBox& obj_frenet_box) {
  const bool from_right =
      std::fabs(obj_frenet_box.l_min) > std::fabs(obj_frenet_box.l_max);
  double max_invasion = -std::numeric_limits<double>::infinity();
  for (const Vec2d& pt : obj_bounding_box.GetCornersCounterClockwise()) {
    const auto corner_sl = path_frenet.XYToSL(pt);
    const auto [right_l, left_l] =
        path_boundary.QueryTargetBoundaryL(corner_sl.s);
    const double ref_width = from_right ? -right_l : right_l;
    const double invasion =
        from_right ? corner_sl.l - right_l : left_l - corner_sl.l;
    max_invasion = std::max(invasion, max_invasion / ref_width);
  }
  return std::min(max_invasion, 1.0);
}

}  // namespace

std::vector<std::string> SelectAccTarget(
    const AccPathCorridor& path_corridor,
    const SpacetimeTrajectoryManager& traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const ApolloTrajectoryPointProto& plan_start_point) {
  Timer timer(__FUNCTION__);

  const Vec2d av_pos = ToVec2d(plan_start_point.path_point());
  const double av_theta = plan_start_point.path_point().theta();
  const Box2d av_box = ComputeAvBox(av_pos, av_theta, vehicle_geometry_params);

  const FrenetFrame* ff = path_corridor.frenet_frame.get();
  const auto av_box_or = ff->QueryFrenetBoxAt(av_box);
  double av_front_s = 0.0;
  if (av_box_or.ok()) {
    av_front_s = av_box_or->s_max;
  } else {
    const auto av_front_center =
        ComputeAvFrontCenter(av_pos, av_theta, vehicle_geometry_params);
    const FrenetCoordinate av_front_sl = ff->XYToSL(av_front_center);
    av_front_s = av_front_sl.s;
  }

  std::vector<std::string> considered_traj_id;
  const PathSlBoundary& path_boundary = path_corridor.boundary;
  for (const SpacetimeObjectTrajectory& st_traj : traj_mgr.trajectories()) {
    if (!IsAccConsideredObject(st_traj.object_type())) {
      continue;
    }
    const auto object_fbox_or = ff->QueryFrenetBoxAtContour(st_traj.contour());
    if (!object_fbox_or.ok()) {
      continue;
    }

    const double obj_center_s = object_fbox_or->center_s();

    // Filter objects behind the vehicle and within half its length.
    const double object_length = st_traj.bounding_box().length();
    if (object_fbox_or->s_max - av_front_s < object_length * 0.5) {
      continue;
    }

    // Filter objects beyond the map's credible distance.
    if (object_fbox_or->s_min > path_corridor.loaded_map_dist) {
      continue;
    }

    // Filter objects outside the path boundary.
    const double query_s = std::clamp(obj_center_s, path_boundary.start_s(),
                                      path_boundary.end_s());
    const auto [right_l, left_l] = path_boundary.QueryBoundaryL(query_s);
    if (object_fbox_or->l_min > left_l || object_fbox_or->l_max < right_l) {
      continue;
    }

    // Filter moving oncoming targets with insufficient overlap rate.
    const auto nearest_path_point = path_corridor.path.Evaluate(obj_center_s);
    const double heading_diff = std::fabs(
        AngleDifference(st_traj.pose().theta(), nearest_path_point.theta()));
    constexpr double kHeadingDiffThres = 3.0 * M_PI_4;
    const double invasion_rate = ComputeObjectInvasionRate(
        path_corridor.boundary, *path_corridor.frenet_frame,
        st_traj.bounding_box(), *object_fbox_or);
    constexpr double kOncomingInvasionRateThres = 0.3;
    constexpr double kMovingSpeedThres = 1.0;
    if (heading_diff > kHeadingDiffThres &&
        std::fabs(invasion_rate) < kOncomingInvasionRateThres &&
        st_traj.pose().v() > kMovingSpeedThres) {
      continue;
    }
    considered_traj_id.push_back(st_traj.traj_id());
  }

  return considered_traj_id;
}

std::unique_ptr<SpacetimeTrajectoryManager> BuildSpacetimeTrajectoryManager(
    const std::vector<std::string>& considered_targets_id,
    const SpacetimeTrajectoryManager& traj_mgr) {
  std::vector<SpacetimeObjectTrajectory> considered_st_trajs;
  considered_st_trajs.reserve(considered_targets_id.size());
  for (const std::string& traj_id : considered_targets_id) {
    const SpacetimeObjectTrajectory* traj =
        traj_mgr.FindTrajectoryById(traj_id);
    if (traj == nullptr) continue;
    SpacetimeObjectTrajectory new_traj = *traj;
    new_traj.set_required_lateral_gap(/*lateral_gap=*/0.0);
    considered_st_trajs.push_back(std::move(new_traj));
  }
  return std::make_unique<SpacetimeTrajectoryManager>(
      absl::MakeSpan(considered_st_trajs));
}

}  // namespace st::planning
