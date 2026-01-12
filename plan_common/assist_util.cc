

#include "assist_util.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <iterator>
#include <optional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "plan_common/container/strong_int.h"
#include "gflags/gflags.h"
//#include "global/clock.h"
//#include "global/trace.h"
//#include "hmi/events/run_event.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/affine_transformation.pb.h"
#include "modules/cnoa_pnc/planning/proto/autonomy_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/lite_common.pb.h"
#include "plan_common/plan_common_defs.h"
//#include "remote_assist_common.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
//#include "plan_common/trajectory_util.h"
//#include "decider/selector/cost_feature_util.h"
#include "plan_common/util/lane_path_util.h"
#include "plan_common/util/planner_semantic_map_util.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/time_util.h"

DEFINE_int32(teleop_expire_seconds, 3, "Instruction expiration");

namespace st::planning {

namespace {
// absl::Status ValidateLiteHeader(const LiteHeader& header,
//                                 const absl::Duration& duration) {
//   // backward compatibility
//   if (header.timestamp() == 0) return absl::OkStatus();

//   const auto delay = Clock::Now() - absl::FromUnixMicros(header.timestamp());
//   if (delay > duration) {
//     return absl::FailedPreconditionError(absl::StrFormat(
//         "Channel[%s]: header timestamp is %.3f, now is %.3f, the "
//         "delay is %.3f seconds.",
//         header.channel(), header.timestamp() * 1e-6,
//         ToUnixDoubleSeconds(Clock::Now()), absl::ToDoubleSeconds(delay)));
//   }
//   return absl::OkStatus();
// }

// absl::Status ValidateRemoteAssistMessage(
//     const RemoteAssistToCarProto& ra_to_car) {
//   return ValidateLiteHeader(ra_to_car.header(),
//                             absl::Seconds(FLAGS_teleop_expire_seconds));
// }

// absl::Status ValidateDriverActionMessage(const DriverAction& driver_action) {
//   return ValidateLiteHeader(driver_action.header(),
//                             absl::Seconds(FLAGS_teleop_expire_seconds));
// }

absl::StatusOr<bool> LaneChangeCompleted(
    const DrivePassage& dp, const Vec2d& ego_pos, double ego_heading,
    const double max_lane_keep_lat_offset) {
  ASSIGN_OR_RETURN(const auto sl, dp.QueryFrenetCoordinateAt(ego_pos));
  ASSIGN_OR_RETURN(const auto angle, dp.QueryTangentAngleAtS(sl.s));

  constexpr double kAngleThreshold = d2r(10.0);

  return std::abs(sl.l) < max_lane_keep_lat_offset &&
         std::abs(NormalizeAngle(angle - ego_heading)) < kAngleThreshold;
}

PoseProto CreatePoseFromApolloTrajectoryPoint(
    const ApolloTrajectoryPointProto& traj_point) {
  PoseProto proto;
  const auto& path_point = traj_point.path_point();
  proto.mutable_pos_smooth()->set_x(path_point.x());
  proto.mutable_pos_smooth()->set_y(path_point.y());
  proto.set_yaw(path_point.theta());
  return proto;
}

absl::StatusOr<mapping::LanePath> FindMostSimilarLanePath(
    const DrivingMapTopo& driving_map, const mapping::LanePath& lane_path,
    const PlannerSemanticMapManager& psmm) {
  const auto* start_lane = driving_map.GetLaneById(lane_path.front().lane_id());

  if (start_lane == nullptr) {
    return absl::NotFoundError(absl::StrCat(
        "start lane ", lane_path.front().lane_id(), " not found."));
  }

  std::vector<const DrivingMapTopo::Lane*> new_lanes = {start_lane};

  for (int origin_idx = 1;; ++origin_idx) {
    if (new_lanes.back()->outgoing_lane_ids.empty()) {
      break;
    }

    const mapping::ElementId next_id = origin_idx < lane_path.size()
                                           ? lane_path.lane_id(origin_idx)
                                           : mapping::kInvalidElementId;

    const auto it =
        std::find(new_lanes.back()->outgoing_lane_ids.begin(),
                  new_lanes.back()->outgoing_lane_ids.end(), next_id);

    // TODO: ForwardExtendLanePathWithMinimumHeadingDiff
    const auto new_next_id = it == new_lanes.back()->outgoing_lane_ids.end()
                                 ? new_lanes.back()->outgoing_lane_ids.front()
                                 : *it;

    const auto* new_next_lane = driving_map.GetLaneById(new_next_id);
    if (new_next_lane == nullptr) {
      return absl::NotFoundError(
          absl::StrCat("lane ", new_next_id, " not found."));
    }

    new_lanes.push_back(new_next_lane);
  }

  std::vector<mapping::ElementId> lane_ids;
  lane_ids.reserve(new_lanes.size());
  for (const auto* lane : new_lanes) {
    lane_ids.push_back(lane->id);
  }

  return BuildLanePathFromData(
      mapping::LanePathData(new_lanes.front()->start_fraction,
                            new_lanes.back()->end_fraction,
                            std::move(lane_ids)),
      psmm);
}

}  // namespace

// absl::Status UpdateExternalCmdStatusFromRemoteAssist(
//     const RemoteAssistToCarProto& proto, ExternalCommandStatus* status) {
//   RETURN_IF_ERROR(ValidateRemoteAssistMessage(proto));

//   switch (proto.request_case()) {
//     case RemoteAssistToCarProto::kLeftBlinkerOverride:
//       status->override_left_blinker_on =
//           proto.left_blinker_override().has_on() &&
//           proto.left_blinker_override().on();
//       break;

//     case RemoteAssistToCarProto::kRightBlinkerOverride:
//       status->override_right_blinker_on =
//           proto.right_blinker_override().has_on() &&
//           proto.right_blinker_override().on();
//       break;

//     case RemoteAssistToCarProto::kEmergencyBlinkerOverride:
//       status->override_emergency_blinker_on =
//           proto.emergency_blinker_override().has_on() &&
//           proto.emergency_blinker_override().on();
//       break;

//     case RemoteAssistToCarProto::kDoorOverride: {
//       status->override_door_open = proto.door_override().open();
//       break;
//     }
//     case RemoteAssistToCarProto::kLaneChangeStyle: {
//       status->lane_change_style = proto.lane_change_style();
//       break;
//     }
//     case RemoteAssistToCarProto::kEnableFeatureOverride: {
//       const auto& enable_feature_override = proto.enable_feature_override();
//       if (enable_feature_override.has_enable_traffic_light_stopping()) {
//         status->enable_traffic_light_stopping =
//             enable_feature_override.enable_traffic_light_stopping();
//       }
//       if (enable_feature_override.has_enable_lc_objects()) {
//         status->enable_lc_objects =
//         enable_feature_override.enable_lc_objects();
//       }
//       if (enable_feature_override.has_enable_pull_over()) {
//         status->enable_pull_over =
//         enable_feature_override.enable_pull_over();
//       }
//       break;
//     }
//     case RemoteAssistToCarProto::kStopVehicle:
//       if (proto.stop_vehicle().has_brake()) {
//         status->brake_to_stop =
//             proto.stop_vehicle().brake() > 0.0
//                 ? std::optional<double>(proto.stop_vehicle().brake())
//                 : std::nullopt;
//       }
//       break;
//     case RemoteAssistToCarProto::kEnableStopPolylineStoppingOverride:
//       status->enable_stop_polyline_stopping =
//           proto.enable_stop_polyline_stopping_override();
//       break;
//     case RemoteAssistToCarProto::kDrivingActionRequest:
//     case RemoteAssistToCarProto::kHeartbeat:
//     case RemoteAssistToCarProto::kDrivableAgentUpdateRequest:
//     case RemoteAssistToCarProto::kUseManualControlCmd:
//     case RemoteAssistToCarProto::kPlayAudioRequest:
//     case RemoteAssistToCarProto::kAebRequest:
//     case RemoteAssistToCarProto::REQUEST_NOT_SET:
//       break;
//   }

//   return absl::OkStatus();
// }

// absl::Status UpdateExternalCmdQueueFromDriverAction(
//     const DriverAction& driver_action, ExternalCommandQueue* queue) {
//   RETURN_IF_ERROR(ValidateDriverActionMessage(driver_action));

//   queue->pending_driver_actions.push_back(driver_action);
//   return absl::OkStatus();
// }

// DriverAction::LaneChangeCommand ProcessLaneChangeCommands(
//     const ExternalCommandQueue& ext_cmd_queue) {
//   if (!ext_cmd_queue.pending_lane_change_requests.empty()) {
//     switch (ext_cmd_queue.pending_lane_change_requests.back().direction()) {
//       case LaneChangeRequestProto::LEFT:
//         return DriverAction::LC_CMD_LEFT;
//       case LaneChangeRequestProto::RIGHT:
//         return DriverAction::LC_CMD_RIGHT;
//       case LaneChangeRequestProto::CANCEL:
//         return DriverAction::LC_CMD_CANCEL;
//       case LaneChangeRequestProto::STRAIGHT:
//         return DriverAction::LC_CMD_STRAIGHT;
//     }
//   }

//   // NOTE: we may lose some key commands.
//   for (const auto& action : ext_cmd_queue.pending_driver_actions) {
//     if (action.has_lane_change_command() &&
//         action.lane_change_command() == DriverAction::LC_CMD_CANCEL) {
//       return DriverAction::LC_CMD_CANCEL;
//     }
//   }
//   for (auto it = ext_cmd_queue.pending_driver_actions.rbegin();
//        it != ext_cmd_queue.pending_driver_actions.rend(); ++it) {
//     if (it->has_lane_change_command() &&
//         (it->lane_change_command() == DriverAction::LC_CMD_LEFT ||
//          it->lane_change_command() == DriverAction::LC_CMD_RIGHT)) {
//       return it->lane_change_command();
//     }
//   }

//   return DriverAction::LC_CMD_NONE;
// }
void AddBoundariesToIntervals(
    const DrivePassage& drive_passage,
    const std::vector<IndexedStationBoundary>& boundaries,
    std::vector<BoundaryInterval>* intervals) {
  if (boundaries.size() < 2) return;

  BoundaryInterval interval;
  interval.type = boundaries.front().second.type;
  interval.points.reserve(boundaries.size());
  for (const auto& boundary : boundaries) {
    interval.points.emplace_back(drive_passage.station(boundary.first)
                                     .lat_point(boundary.second.lat_offset));
  }
  intervals->emplace_back(std::move(interval));
}

std::vector<BoundaryInterval> FindSolidBoundaryIntervals(
    const DrivePassage& drive_passage, const FrenetCoordinate& first_point_sl,
    double cutoff_s, const double route_station_unit_step) {
  std::vector<BoundaryInterval> intervals;
  std::vector<std::vector<IndexedStationBoundary>> active_intervals;
  for (const auto index : drive_passage.stations().index_range()) {
    const auto& station = drive_passage.station(index);
    if (station.accumulated_s() < -route_station_unit_step) continue;
    if (station.accumulated_s() > cutoff_s) break;

    std::vector<StationBoundary> new_boundaries;
    for (const auto& boundary : station.boundaries()) {
      if (!boundary.IsSolid(first_point_sl.l) ||
          boundary.type == StationBoundaryType::VIRTUAL_CURB) {
        // Only consider real boundaries here, not the virtual curbs.
        continue;
      }

      double match_dist = kContinuousBoundaryMaxLatOffset;
      int match_idx = -1;
      for (int j = 0; j < active_intervals.size(); ++j) {
        const auto& interval_back = active_intervals[j].back();
        if (interval_back.first.value() + 1 == index.value() &&
            interval_back.second.type == boundary.type) {
          const double lat_dist =
              std::abs(interval_back.second.lat_offset - boundary.lat_offset);
          if (lat_dist < match_dist) {
            match_dist = lat_dist;
            match_idx = j;
          }
        }
      }
      if (match_idx == -1) {
        new_boundaries.push_back(boundary);
      } else {
        active_intervals[match_idx].emplace_back(index, boundary);
      }
    }
    for (auto it = active_intervals.begin(); it != active_intervals.end();) {
      if (it->back().first != index) {
        AddBoundariesToIntervals(drive_passage, *it, &intervals);
        it = active_intervals.erase(it);
      } else {
        ++it;
      }
    }
    for (auto& new_boundary : new_boundaries) {
      active_intervals.emplace_back().emplace_back(index, new_boundary);
    }
  }
  for (auto& interval : active_intervals) {
    AddBoundariesToIntervals(drive_passage, interval, &intervals);
  }

  return intervals;
}

absl::StatusOr<bool> CrossedBoundary(const DrivePassage& dp,
                                     const Vec2d& ego_pos) {
  ASSIGN_OR_RETURN(const auto sl, dp.QueryFrenetCoordinateAt(ego_pos));

  constexpr double kMaxHalfWidth = 2.5;  // m.
  if (std::abs(sl.l) > kMaxHalfWidth) return false;

  constexpr double kMinHalfWidth = 1.0;  // m.
  if (std::abs(sl.l) < kMinHalfWidth) return true;

  ASSIGN_OR_RETURN(const auto l_pair,
                   dp.QueryNearestBoundaryLateralOffset(sl.s));

  return sl.l > l_pair.first && sl.l < l_pair.second;
}

absl::StatusOr<bool> HasTrajectoryCrossedSolidBoundary(
    const DrivePassage& drive_passage, const PathSlBoundary& sl_boundary,
    const std::vector<ApolloTrajectoryPointProto>& traj_pts,
    const VehicleGeometryParamsProto& vehicle_geom, bool lc_pause,
    const double route_station_unit_step) {
  constexpr int kCheckEveryNPt = 5;
  constexpr double kTrajectorySExtension = 10.0;  // m.

  const int check_first_n =
      lc_pause
          ? std::min<int>(CeilToInt(0.3 * traj_pts.size()) + 1, traj_pts.size())
          : traj_pts.size();
  const auto last_pt_in_rear_center =
      Vec2dFromApolloTrajectoryPointProto(traj_pts[check_first_n - 1]);
  const auto heading = traj_pts[check_first_n - 1].path_point().theta();
  const auto ego_width = vehicle_geom.width();
  const auto ego_length = vehicle_geom.length();
  const auto last_pt_in_front_center =
      last_pt_in_rear_center +
      Vec2d::FastUnitFromAngle(heading) * vehicle_geom.front_edge_to_center();

  ASSIGN_OR_RETURN(const auto last_pt_in_front_center_frenet,
                   drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                       last_pt_in_front_center),
                   _ << "Last considered traj point not on drive passage.");
  ASSIGN_OR_RETURN(const auto first_point_sl,
                   drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
                       Vec2dFromApolloTrajectoryPointProto(traj_pts.front())),
                   _ << "First traj point not on drive passage.");

  const auto solid_boundaries = FindSolidBoundaryIntervals(
      drive_passage, first_point_sl,
      last_pt_in_front_center_frenet.s + kTrajectorySExtension,
      route_station_unit_step);

  // Generate ego car trajectory box.
  std::vector<Box2d> ego_boxes;
  ego_boxes.reserve(
      CeilToInt(check_first_n / static_cast<float>(kCheckEveryNPt)));
  for (int i = 0; i < check_first_n; i += kCheckEveryNPt) {
    const auto& traj_pt = traj_pts[i];
    ego_boxes.emplace_back(Vec2dFromApolloTrajectoryPointProto(traj_pt),
                           traj_pt.path_point().theta(), ego_length, ego_width);
  }

  // For low speed condition before a stop line.
  const Segment2d last_pt_to_ref_center_seg(
      last_pt_in_front_center,
      sl_boundary.QueryReferenceCenterXY(last_pt_in_front_center_frenet.s));
  ego_boxes.emplace_back(last_pt_to_ref_center_seg, ego_width);

  for (const auto& boundary : solid_boundaries) {
    const auto& boundary_pts = boundary.points;
    for (const auto& ego_box : ego_boxes) {
      for (int j = 1; j < boundary_pts.size(); ++j) {
        const Segment2d boundary_seg(boundary_pts[j - 1], boundary_pts[j]);
        if (ego_box.HasOverlap(boundary_seg)) {
          return true;
        }
      }
    }
  }
  return false;
}

absl::StatusOr<ALCState> UpdateAlcState(ALCState state,
                                        const Vec2d& preview_pos,
                                        const Vec2d& ego_pos,
                                        double ego_heading,
                                        const DrivePassage& drive_passage,
                                        const double max_lane_keep_lat_offset) {
  switch (state) {
    case ALCState::ALC_OFF:
    case ALCState::ALC_STANDBY:
    case ALCState::ALC_STANDBY_ENABLE:
    case ALCState::ALC_PREPARE:
      return state;

    case ALCState::ALC_RETURN_COMPLETED:
    case ALCState::ALC_COMPLETED:
      return ALCState::ALC_STANDBY_ENABLE;

    case ALCState::ALC_CROSSING_LANE: {
      ASSIGN_OR_RETURN(const bool comp,
                       LaneChangeCompleted(drive_passage, ego_pos, ego_heading,
                                           max_lane_keep_lat_offset));
      return comp ? ALCState::ALC_COMPLETED : ALCState::ALC_CROSSING_LANE;
    }

    case ALCState::ALC_RETURNING: {
      ASSIGN_OR_RETURN(const bool comp,
                       LaneChangeCompleted(drive_passage, ego_pos, ego_heading,
                                           max_lane_keep_lat_offset));
      return comp ? ALCState::ALC_RETURN_COMPLETED : ALCState::ALC_RETURNING;
    }
    case ALCState::ALC_ONGOING: {
      ASSIGN_OR_RETURN(bool cross, CrossedBoundary(drive_passage, preview_pos));
      return cross ? ALCState::ALC_CROSSING_LANE : ALCState::ALC_ONGOING;
    }
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

void ReportPlcEventSignal(ALCState old_state, ALCState new_state,
                          DriverAction::LaneChangeCommand lc_cmd) {
  switch (new_state) {
    case ALC_OFF:
    case ALC_STANDBY:
    case ALC_STANDBY_ENABLE:
    case ALC_CROSSING_LANE:
      return;
    case ALC_PREPARE:
      if (old_state != ALC_PREPARE) {
        if (lc_cmd == DriverAction::LC_CMD_LEFT) {
          LOG_INFO << "Start waiting to lane change to left.";
        } else {
          LOG_INFO << "Start waiting to lane change to right.";
        }
      }
      return;
    case ALC_ONGOING:
      if (old_state != ALC_ONGOING) {
        if (lc_cmd == DriverAction::LC_CMD_LEFT) {
          LOG_INFO << "Start lane changing to left.";
        } else {
          LOG_INFO << "Start lane changing to right.";
        }
      }
      return;
    case ALC_RETURNING:
      if (old_state != ALC_RETURNING) {
        if (lc_cmd == DriverAction::LC_CMD_LEFT) {
          LOG_INFO << "Start returning to left.";
        } else {
          LOG_INFO << "Start returning to right.";
        }
      }
      return;
    case ALC_COMPLETED:
      LOG_INFO << "Completed lane change.";
      return;
    case ALC_RETURN_COMPLETED:
      LOG_INFO << "Completed lane change return.";
      return;
  }
}

}  // namespace st::planning
