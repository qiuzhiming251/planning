

#include "decider/decision_manager/leading_object.h"

#include <float.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "decider/decision_manager/decision_util.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/lane_path.pb.h"
#include "object_manager/planner_object.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_planner_object_trajectories_filter.h"
#include "plan_common/second_order_trajectory_point.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/lane_point_util.h"
#include "plan_common/util/planner_semantic_map_util.h"

namespace st {
namespace planning {

namespace {

using ObjectsOnLane =
    std::vector<std::pair<FrenetBox, const SpacetimeObjectTrajectory*>>;

// Filter on coming object by drive passage.
absl::StatusOr<bool> IsOncomingObjectJudgeByDrivePassage(
    const DrivePassage& passage, const SecondOrderTrajectoryPoint& obj_pose) {
  ASSIGN_OR_RETURN(const auto tangent, passage.QueryTangentAt(obj_pose.pos()));
  const double passage_angle = tangent.Angle();
  const double angle_diff =
      std::abs(NormalizeAngle(passage_angle - obj_pose.theta()));

  return angle_diff > M_PI_2;
}

// Filter on coming object by ego heading.
bool IsOncomingObjectJudgeByEgoHeading(
    const ApolloTrajectoryPointProto& plan_start_point,
    const SecondOrderTrajectoryPoint& obj_pose) {
  const double ego_heading_angle = plan_start_point.path_point().theta();
  const double object_heading_angle = obj_pose.theta();
  const double angle_diff =
      std::abs(NormalizeAngle(ego_heading_angle - object_heading_angle));

  // Object moving in the opposite direction of ego vehicle.
  return angle_diff > M_PI_2;
}

absl::StatusOr<FrenetBox> FilterObjectViaDrivePassage(
    const PlannerObject& object, const DrivePassage& passage,
    const PathSlBoundary& sl_boundary, const FrenetBox& ego_frenet_box,
    const absl::flat_hash_set<std::string>& stalled_objects,
    absl::flat_hash_set<std::string>& collect_stdlled) {
  // Calculate object frenet coordinate.
  ASSIGN_OR_RETURN(const auto object_frenet_box,
                   passage.QueryFrenetBoxAtContour(object.contour()));

  // Filter objects behind ego front edge or beyond drive passage length.
  // if (object_frenet_box.s_min < ego_frenet_box.s_max ||
  //     object_frenet_box.s_min > sl_boundary.end_s()) {
  //   return absl::OutOfRangeError(absl::StrFormat(
  //       "Object %s out of longitudinal boundary, s range: ( %.2f, %.2f)",
  //       object.id(), object_frenet_box.s_min, object_frenet_box.s_max));
  // }

  constexpr double kLateralEnterThres = 0.5;  // m.
  const auto [boundary_l_max, boundary_l_min] =
      CalcSlBoundaries(sl_boundary, object_frenet_box);
  // Filter objects not on path boundary.
  if (object_frenet_box.l_min > boundary_l_max - kLateralEnterThres ||
      object_frenet_box.l_max < boundary_l_min + kLateralEnterThres) {
    return absl::OutOfRangeError(absl::StrFormat(
        "Object %s out of lateral boundary, l range: (%.2f, %.2f)", object.id(),
        object_frenet_box.l_min, object_frenet_box.l_max));
  }

  if (stalled_objects.find(object.id()) != stalled_objects.end() &&
      ego_frenet_box.center_l() > object_frenet_box.center_l()) {
    collect_stdlled.insert(object.id());
  }

  if (object_frenet_box.s_min < ego_frenet_box.s_max ||
      object_frenet_box.s_min > sl_boundary.end_s()) {
    return absl::OutOfRangeError(absl::StrFormat(
        "Object %s out of longitudinal boundary, s range: ( %.2f, %.2f)",
        object.id(), object_frenet_box.s_min, object_frenet_box.s_max));
  }
  return object_frenet_box;
}

ObjectsOnLane FindFrontObjectsOnLane(
    const DrivePassage& passage, const PathSlBoundary& sl_boundary,
    absl::Span<const SpacetimeObjectTrajectory> st_trajs,
    const FrenetBox& ego_frenet_box,
    absl::flat_hash_set<std::string>& collect_stdlled,
    const absl::flat_hash_set<std::string>& stalled_objects) {
  ObjectsOnLane st_trajs_on_lane;
  st_trajs_on_lane.reserve(st_trajs.size());

  for (const auto& st_traj : st_trajs) {
    // Filter by object type.
    if (!IsLeadingObjectType(st_traj.planner_object().type())) {
      continue;
    }

    // Filter oncoming object.
    const auto res =
        IsOncomingObjectJudgeByDrivePassage(passage, st_traj.pose());
    if ((!res.ok() || *res == true) &&
        stalled_objects.find(st_traj.planner_object().id()) ==
            stalled_objects.end()) {
      continue;
    }

    // Filter by drive passage and ego frenet box.
    ASSIGN_OR_CONTINUE(const auto obj_fbox,
                       FilterObjectViaDrivePassage(
                           st_traj.planner_object(), passage, sl_boundary,
                           ego_frenet_box, stalled_objects, collect_stdlled));
    // if (stalled_objects.find(st_traj.planner_object().id()) !=
    //         stalled_objects.end() &&
    //     ego_frenet_box.center_l() > obj_fbox.center_l()) {
    //   collect_stdlled.insert(st_traj.planner_object().id());
    // }
    st_trajs_on_lane.emplace_back(obj_fbox, &st_traj);
  }

  // Sort by objects arc length on lane path.
  std::stable_sort(st_trajs_on_lane.begin(), st_trajs_on_lane.end(),
                   [](const auto& a, const auto& b) {
                     return a.first.s_min < b.first.s_min;
                   });

  return st_trajs_on_lane;
}

bool IsUnsafeType(ObjectType type) {
  switch (type) {
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_MOTORCYCLIST:
    case OT_TRICYCLIST:
      return true;
    case OT_UNKNOWN_MOVABLE:
    case OT_UNKNOWN_STATIC:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
    // case OT_TRICYCLIST:
    case OT_FOD:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return false;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

void IsEnvUnsafe(absl::Span<const SpacetimeObjectTrajectory> st_trajs,
                 const DrivePassage& passage, const FrenetBox& ego_frenet_box,
                 std::set<std::string>& l_unsafe_set,
                 std::set<std::string>& r_unsafe_set,
                 const ApolloTrajectoryPointProto& plan_start_point,
                 double lat_thr, const std::string& judge_obj_id,
                 double td = 0.8) {
  for (const auto& st_traj : st_trajs) {
    const double ego_heading_angle = plan_start_point.path_point().theta();
    const double object_heading_angle = st_traj.pose().theta();
    const std::string obj_id = std::string(st_traj.object_id());
    double obj_v = st_traj.pose().v();
    const double angle_diff =
        NormalizeAngle(ego_heading_angle - object_heading_angle);
    if (!IsUnsafeType(st_traj.planner_object().type()) ||
        obj_id == judge_obj_id)
      continue;
    std::string env_debug =
        absl::StrCat(" angle_diff: ", angle_diff, " lat_thr: ", lat_thr);
    const auto object_frenet_box =
        passage.QueryFrenetBoxAtContour(st_traj.planner_object().contour());
    if (object_frenet_box.ok()) {
      if (std::abs(angle_diff) > M_PI_2) {
        if (((object_frenet_box.value().l_min > ego_frenet_box.l_max &&
              object_frenet_box.value().l_min <
                  ego_frenet_box.l_max + lat_thr))) {
          if (ego_frenet_box.s_min < object_frenet_box.value().s_max &&
              ego_frenet_box.s_max + (plan_start_point.v() + obj_v) >
                  object_frenet_box.value().s_min) {
            l_unsafe_set.insert(st_traj.planner_object().id());
            env_debug +=
                absl::StrCat(" is_resleft_front_ttc ", obj_id, " v: ", obj_v);
          }
        }
      } else if (((object_frenet_box.value().l_min > ego_frenet_box.l_max &&
                   object_frenet_box.value().l_min <
                       ego_frenet_box.l_max + lat_thr) &&
                  angle_diff > -M_PI / 18) ||
                 ((object_frenet_box.value().l_max < ego_frenet_box.l_min &&
                   object_frenet_box.value().l_max >
                       ego_frenet_box.l_min - lat_thr) &&
                  angle_diff < M_PI / 18)) {
        // bool is_unsafe_debug =
        //     ego_frenet_box.s_min > object_frenet_box.value().s_max &&
        //     (ego_frenet_box.s_min + 2 * (plan_start_point.v() - obj_v) <
        //      object_frenet_box.value().s_max);
        // env_debug += absl::StrCat(" is_unsafe_debug: ",is_unsafe_debug);
        if (ego_frenet_box.s_max < object_frenet_box.value().s_min &&
            (ego_frenet_box.s_max + 2 * (plan_start_point.v() - obj_v) >
             object_frenet_box.value().s_min)) {
          if (object_frenet_box.value().l_min > ego_frenet_box.l_max) {
            l_unsafe_set.insert(st_traj.planner_object().id());
            env_debug +=
                absl::StrCat(" is_left_front_ttc ", obj_id, " v: ", obj_v);
          } else if (object_frenet_box.value().l_max < ego_frenet_box.l_min) {
            r_unsafe_set.insert(st_traj.planner_object().id());
            env_debug +=
                absl::StrCat(" is_right_front_ttc ", obj_id, " v: ", obj_v);
          }
        } else if (ego_frenet_box.s_min > object_frenet_box.value().s_max &&
                   ((ego_frenet_box.s_min + 2 * (plan_start_point.v() - obj_v) <
                     object_frenet_box.value().s_max) ||
                    (std::max(obj_v * td, 1.0) >
                     ego_frenet_box.s_min - object_frenet_box.value().s_max))) {
          if (object_frenet_box.value().l_min > ego_frenet_box.l_max) {
            l_unsafe_set.insert(st_traj.planner_object().id());
            env_debug +=
                absl::StrCat(" is_left_back_ttc ", obj_id, " v: ", obj_v);
          } else if (object_frenet_box.value().l_max < ego_frenet_box.l_min) {
            r_unsafe_set.insert(st_traj.planner_object().id());
            env_debug +=
                absl::StrCat(" is_right_back_ttc ", obj_id, " v: ", obj_v);
          }
        } else if ((ego_frenet_box.s_min > object_frenet_box.value().s_min &&
                    ego_frenet_box.s_min < object_frenet_box.value().s_max) ||
                   (ego_frenet_box.s_max < object_frenet_box.value().s_max &&
                    ego_frenet_box.s_max > object_frenet_box.value().s_min)) {
          if (object_frenet_box.value().l_min > ego_frenet_box.l_max) {
            l_unsafe_set.insert(st_traj.planner_object().id());
            env_debug +=
                absl::StrCat(" is_left_use_pos ", obj_id, " v: ", obj_v);
          } else if (object_frenet_box.value().l_max < ego_frenet_box.l_min) {
            r_unsafe_set.insert(st_traj.planner_object().id());
            env_debug +=
                absl::StrCat(" is_right_use_pos ", obj_id, " v: ", obj_v);
          }
        }
      } else {
        bool is_lat_enough =
            (object_frenet_box.value().l_min > ego_frenet_box.l_max &&
             object_frenet_box.value().l_min < ego_frenet_box.l_max + lat_thr);
        env_debug +=
            absl::StrCat(" lat_enough: ", obj_id, " v: ", obj_v,
                         " is_lat_enough: ", is_lat_enough,
                         " obj_l_min: ", object_frenet_box.value().l_min,
                         " ego_frenet_box.l_max: ", ego_frenet_box.l_max);
      }
    } else {
      env_debug += absl::StrCat(" obj_no_box: ", obj_id, " v: ", obj_v);
    }
    Log2DDS::LogDataV2("env_debug", env_debug);
  }
  return;
}

// bool IsEnvUnsafe(absl::Span<const SpacetimeObjectTrajectory> st_trajs,
//                  const DrivePassage& passage, const FrenetBox&
//                  ego_frenet_box, const std::string& judge_obj_id, const
//                  ApolloTrajectoryPointProto& plan_start_point, double
//                  lat_threshold) {
//   for (const auto& st_traj : st_trajs) {
//     const double ego_heading_angle = plan_start_point.path_point().theta();
//     const double object_heading_angle = st_traj.pose().theta();
//     const std::string obj_id = std::string(st_traj.object_id());
//     double obj_v = st_traj.pose().v();
//     const double angle_diff =
//         NormalizeAngle(ego_heading_angle - object_heading_angle);
//     if (!IsUnsafeType(st_traj.planner_object().type()) ||judge_obj_id ==
//     obj_id ) continue; std::string env_debug = ""; const auto
//     object_frenet_box =
//         passage.QueryFrenetBoxAtContour(st_traj.planner_object().contour());
//     if (object_frenet_box.ok()) {
//       if (std::abs(angle_diff) > M_PI_2) {
//         if (((object_frenet_box.value().l_min > ego_frenet_box.l_max &&
//               object_frenet_box.value().l_min <
//                   ego_frenet_box.l_max + lat_threshold))) {
//           if (ego_frenet_box.s_min < object_frenet_box.value().s_max &&
//               ego_frenet_box.s_max + (plan_start_point.v() + obj_v) >
//                   object_frenet_box.value().s_min) {
//             env_debug += absl::StrCat(" is_resleft_front_ttc ", obj_id,
//                                       " v: ", obj_v);
//             return true;
//           }
//         }
//       } else if (((object_frenet_box.value().l_min > ego_frenet_box.l_max &&
//                    object_frenet_box.value().l_min <
//                        ego_frenet_box.l_max + lat_threshold) &&
//                   angle_diff > -M_PI / 18) ||
//                  ((object_frenet_box.value().l_max < ego_frenet_box.l_min &&
//                    object_frenet_box.value().l_max >
//                        ego_frenet_box.l_min - lat_threshold) &&
//                   angle_diff < M_PI / 18)) {
//         if (ego_frenet_box.s_max < object_frenet_box.value().s_min &&
//             (ego_frenet_box.s_max + 2 * (plan_start_point.v() - obj_v) >
//              object_frenet_box.value().s_min)) {
//           if (object_frenet_box.value().l_min > ego_frenet_box.l_max) {
//             env_debug +=
//                 absl::StrCat(" is_left_front_ttc ", obj_id, " v: ", obj_v);
//             return true;
//           } else if (object_frenet_box.value().l_max < ego_frenet_box.l_min)
//           {
//             env_debug += absl::StrCat(" is_right_front_ttc ", obj_id,
//                                       " v: ", obj_v);
//             return true;
//           }
//         } else if (ego_frenet_box.s_min > object_frenet_box.value().s_max &&
//                    (ego_frenet_box.s_min + 2 * (plan_start_point.v() - obj_v)
//                    <
//                     object_frenet_box.value().s_max)) {
//           if (object_frenet_box.value().l_min > ego_frenet_box.l_max) {
//             env_debug +=
//                 absl::StrCat(" is_left_back_ttc ", obj_id, " v: ", obj_v);
//             return true;
//           } else if (object_frenet_box.value().l_max < ego_frenet_box.l_min)
//           {
//             env_debug +=
//                 absl::StrCat(" is_right_back_ttc ", obj_id, " v: ", obj_v);
//             return true;
//           }
//         } else if ((ego_frenet_box.s_min > object_frenet_box.value().s_min &&
//                     ego_frenet_box.s_min < object_frenet_box.value().s_max)
//                     ||
//                    (ego_frenet_box.s_max < object_frenet_box.value().s_max &&
//                     ego_frenet_box.s_max > object_frenet_box.value().s_min))
//                     {
//           if (object_frenet_box.value().l_min > ego_frenet_box.l_max) {
//             env_debug +=
//                 absl::StrCat(" is_left_use_pos ", obj_id, " v: ", obj_v);
//             return true;
//           } else if (object_frenet_box.value().l_max < ego_frenet_box.l_min)
//           {
//             env_debug +=
//                 absl::StrCat(" is_right_use_pos ", obj_id, " v: ", obj_v);
//             return true;
//           }
//         }
//       } else {
//         env_debug += absl::StrCat(" lat_enough: ", obj_id, " v: ", obj_v);
//       }
//     } else {
//       env_debug += absl::StrCat(" obj_no_box: ", obj_id, " v: ", obj_v);
//     }
//     Log2DDS::LogDataV2("env_debug", env_debug);
//   }
//   return false;
// }

void IsEnvUnsafe(absl::Span<const SpacetimeObjectTrajectory> st_trajs,
                 const DrivePassage& passage, const FrenetBox& ego_frenet_box,
                 bool& is_left, bool& is_right, const std::string& obj_id,
                 const ApolloTrajectoryPointProto& plan_start_point) {
  constexpr double kLateralEnterThres = 1.5;
  for (const auto& st_traj : st_trajs) {
    std::string env_debug = "";
    const double obj_v = st_traj.pose().v();
    if (obj_id == st_traj.planner_object().id()) continue;
    const auto object_frenet_box =
        passage.QueryFrenetBoxAtContour(st_traj.planner_object().contour());
    if (object_frenet_box.ok()) {
      if ((object_frenet_box.value().l_min > ego_frenet_box.l_max &&
           object_frenet_box.value().l_min <
               ego_frenet_box.l_max + kLateralEnterThres) ||
          (object_frenet_box.value().l_max < ego_frenet_box.l_min &&
           object_frenet_box.value().l_max >
               ego_frenet_box.l_min - kLateralEnterThres)) {
        if (ego_frenet_box.s_max < object_frenet_box.value().s_min &&
            (ego_frenet_box.s_max + 2 * (plan_start_point.v() - obj_v) >
             object_frenet_box.value().s_min)) {
          if (object_frenet_box.value().l_min > ego_frenet_box.l_max) {
            is_left = true;
            env_debug += absl::StrCat(" is_left: ", is_left, " front_ttc");
            // return;
          } else if (object_frenet_box.value().l_max < ego_frenet_box.l_min) {
            is_right = true;
            env_debug += absl::StrCat(" is_right: ", is_right, " front_ttc");
            // return;
          }
        } else if (ego_frenet_box.s_min > object_frenet_box.value().s_max &&
                   (ego_frenet_box.s_min + 2 * (plan_start_point.v() - obj_v) <
                    object_frenet_box.value().s_max)) {
          if (object_frenet_box.value().l_min > ego_frenet_box.l_max) {
            is_left = true;
            env_debug += absl::StrCat(" is_left: ", is_left, " back_ttc");
            // return;
          } else if (object_frenet_box.value().l_max < ego_frenet_box.l_min) {
            is_right = true;
            env_debug += absl::StrCat(" is_right: ", is_left, " back_ttc");
            // return;
          }
        } else if ((ego_frenet_box.s_min > object_frenet_box.value().s_min &&
                    ego_frenet_box.s_min < object_frenet_box.value().s_max) ||
                   (ego_frenet_box.s_max < object_frenet_box.value().s_max &&
                    ego_frenet_box.s_max > object_frenet_box.value().s_min)) {
          if (object_frenet_box.value().l_min > ego_frenet_box.l_max) {
            is_left = true;
            env_debug += absl::StrCat(" is_left: ", is_left, " use_pos");
            // return;
          } else if (object_frenet_box.value().l_max < ego_frenet_box.l_min) {
            is_right = true;
            env_debug += absl::StrCat(" is_right: ", is_right, " use_pos");
            // return;
          }
        }
      } else {
        env_debug += " lat_enough";
      }
    } else {
      env_debug += " obj_no_box";
    }
    env_debug += absl::StrCat(" id: ", obj_id);
    Log2DDS::LogDataV2("env_debug", env_debug);
  }
  return;
}

bool IsObjectAvoidableWithinEnv(
    const DrivePassage& passage, const FrenetBox& obj_frenet_box,
    const std::string& obj_id,
    absl::Span<const SpacetimeObjectTrajectory> st_trajs, const bool& l_space,
    const bool& r_space, const NudgeObjectInfo* nudge_object_info,
    const FrenetBox& ego_frenet_box, bool is_stationary) {
  constexpr double kLateralEnterThres = 2.5;
  std::string obj_env_debug = " obj_id: " + obj_id;
  obj_env_debug += absl::StrCat(" l_space: ", l_space, " r_space: ", r_space,
                                " ego_s_min: ", obj_frenet_box.s_min,
                                "ego_s_max: ", obj_frenet_box.s_max);
  bool UnOverlap = (obj_frenet_box.l_min > ego_frenet_box.l_max ||
                    obj_frenet_box.l_max < ego_frenet_box.l_min);
  if (nudge_object_info || !is_stationary) return true;
  if (UnOverlap) return true;
  bool left_unsafe = false;
  bool right_unsafe = false;
  for (const auto& st_traj : st_trajs) {
    const double obj_v = st_traj.pose().v();
    const std::string other_id = st_traj.planner_object().id();
    if (obj_id == st_traj.planner_object().id() || !st_traj.is_stationary() ||
        obj_v != 0.0)
      continue;
    const auto object_frenet_box =
        passage.QueryFrenetBoxAtContour(st_traj.planner_object().contour());
    if (object_frenet_box.ok()) {
      bool is_lon_unverlap =
          object_frenet_box.value().s_min > obj_frenet_box.s_max + 2.0 ||
          object_frenet_box.value().s_max < obj_frenet_box.s_min - 2.0;
      std::string sl_debug =
          absl::StrCat(" obj_s_min: ", object_frenet_box.value().s_min,
                       " obj_s_max: ", object_frenet_box.value().s_max);
      if (!is_lon_unverlap) {
        bool LatUnOverlap =
            (obj_frenet_box.l_min > object_frenet_box.value().l_max ||
             obj_frenet_box.l_max < object_frenet_box.value().l_min);
        if (!LatUnOverlap) return true;
        if (object_frenet_box.value().center_l() > obj_frenet_box.center_l() &&
            object_frenet_box.value().l_min <
                obj_frenet_box.l_max + kLateralEnterThres) {
          obj_env_debug += sl_debug;
          obj_env_debug += absl::StrCat(" left is ununsafe ", other_id);
          left_unsafe = true;
        } else if (object_frenet_box.value().center_l() <
                       obj_frenet_box.center_l() &&
                   object_frenet_box.value().l_max >
                       obj_frenet_box.l_min - kLateralEnterThres) {
          obj_env_debug += sl_debug;
          obj_env_debug += absl::StrCat(" right is ununsafe ", other_id);
          right_unsafe = true;
        }
      }
    }
  }
  Log2DDS::LogDataV0("obj_env_debug", obj_env_debug);
  if (r_space && !right_unsafe) return true;
  if (l_space && !left_unsafe) return true;
  return false;
}

// According the first lane id of traffic waiting queue's lane path to match
// current lane path.
absl::flat_hash_set<std::string_view> CollectTrafficWaitingObjectOnCurrentLane(
    const SceneOutputProto& scene_reasoning,
    const mapping::LanePath& lane_path) {
  absl::flat_hash_set<std::string_view> traffic_waiting_objects;
  std::string traffic_wait_debug = " ";
  int size_queue = scene_reasoning.traffic_waiting_queue().size();
  traffic_wait_debug += absl::StrCat(size_queue);
  for (const auto& traffic_waiting_queue :
       scene_reasoning.traffic_waiting_queue()) {
    traffic_wait_debug += "---------";
    const auto& lane_ids = lane_path.lane_ids();
    if (!traffic_waiting_queue.has_lane_path() ||
        traffic_waiting_queue.lane_path().lane_ids().empty()) {
      continue;
    }
    const mapping::ElementId first_lane_id(
        traffic_waiting_queue.lane_path().lane_ids(0));

    if (std::find(lane_ids.begin(), lane_ids.end(), first_lane_id) !=
        lane_ids.end()) {
      for (const auto& object_id : traffic_waiting_queue.object_id()) {
        traffic_wait_debug += absl::StrCat(" obj_id ", object_id);
        traffic_waiting_objects.insert(object_id);
      }
    }
  }
  Log2DDS::LogDataV0("traffic_wait_debug_leading", traffic_wait_debug);
  return traffic_waiting_objects;
}

bool IsObjectAvoidableWithinCurbBuffer(
    const DrivePassage& passage, const FrenetBox& obj_frenet_box,
    const PathSlBoundary& sl_boundary, const double& lead_threshold,
    const std::string& obj_id, const double& ego_width, const bool& l_space,
    const bool& r_space) {
  constexpr double kSampleStepAlongS = 1.0;  // m.
  double min_left_space = DBL_MAX, min_right_space = DBL_MAX;
  for (double sample_s = obj_frenet_box.s_min; sample_s <= obj_frenet_box.s_max;
       sample_s += kSampleStepAlongS) {
    const auto curb_l = passage.QueryCurbOffsetAtS(sample_s);
    if (!curb_l.ok()) {
      continue;
    }
    // const auto [inner_right_l, inner_left_l] =
    //     sl_boundary.QueryTargetBoundaryL(sample_s);
    // const double right_l =
    //     std::fmax(curb_l->first + 0.5, inner_right_l + lead_threshold);
    // const double left_l =
    //     std::fmin(curb_l->second - 0.5, inner_left_l - lead_threshold);
    const double right_l = curb_l->first + 0.8;
    const double left_l = curb_l->second - 0.8;
    min_left_space = std::clamp(
        left_l - std::max(right_l, obj_frenet_box.l_max), 0.0, min_left_space);
    min_right_space = std::clamp(
        std::min(left_l, obj_frenet_box.l_min) - right_l, 0.0, min_right_space);
  }
  const std::string curb_info =
      absl::StrCat("id:", obj_id, " min_left_space: ", min_left_space,
                   " min_right_space: ", min_right_space, " l_space; ", l_space,
                   " r_space; ", r_space);
  Log2DDS::LogDataV2("curb_info", curb_info);
  // return std::max(min_left_space, min_right_space) > ego_width;
  if (l_space && r_space) {
    return std::max(min_left_space, min_right_space) > ego_width;
  } else if (l_space) {
    return min_left_space > ego_width;
  }
  return min_right_space > ego_width;
}

bool IsObjectAvoidableWithinSlBoundary(
    const PathSlBoundary& sl_boundary, const FrenetBox& obj_frenet_box,
    double ego_width, double lead_threshold, const std::string& id,
    bool is_out_boundary, const std::set<std::string>& l_unsafe_set,
    const std::set<std::string>& r_unsafe_set,
    const NudgeObjectInfo* nudge_object_info, const bool& is_lane_change,
    bool judge_type, const FrenetBox& ego_frenet_box) {
  constexpr double kSampleStepAlongS = 1.0;  // m.
  double min_left_space = DBL_MAX, min_right_space = DBL_MAX;

  bool UnOverlap = (obj_frenet_box.l_min > ego_frenet_box.l_max ||
                    obj_frenet_box.l_max < ego_frenet_box.l_min);
  for (double sample_s = obj_frenet_box.s_min; sample_s <= obj_frenet_box.s_max;
       sample_s += kSampleStepAlongS) {
    const auto [right_l, left_l] =
        is_out_boundary ? sl_boundary.QueryBoundaryL(sample_s)
                        : sl_boundary.QueryTargetBoundaryL(sample_s);
    min_left_space = std::clamp(
        left_l - std::max(right_l, obj_frenet_box.l_max), 0.0, min_left_space);
    min_right_space = std::clamp(
        std::min(left_l, obj_frenet_box.l_min) - right_l, 0.0, min_right_space);
  }
  const std::string obj_thr = absl::StrCat(
      "id: ", id, "leading_threshold: ", lead_threshold,
      " l_unsafe_set_size: ", l_unsafe_set.size(),
      " r_unsafe_set_size: ", r_unsafe_set.size(),
      " min_left_space: ", min_left_space,
      " min_right_space: ", min_right_space, " judge_type: ", judge_type);
  Log2DDS::LogDataV2("obj_thr", obj_thr);
  if (!is_lane_change && judge_type && !UnOverlap) {
    if (min_left_space > (ego_width + lead_threshold) && !nudge_object_info) {
      if (l_unsafe_set.size() > 1 ||
          (l_unsafe_set.size() == 1 &&
           l_unsafe_set.find(id) == l_unsafe_set.end())) {
        return false;
      }
    } else if (min_right_space > (ego_width + lead_threshold) &&
               !nudge_object_info) {
      if (r_unsafe_set.size() > 1 ||
          (r_unsafe_set.size() == 1 &&
           r_unsafe_set.find(id) == r_unsafe_set.end())) {
        return false;
      }
    }
  }

  return std::max(min_left_space, min_right_space) >
         (ego_width + lead_threshold);
}

bool IsObjectAvoidableWithinSlBoundary(
    const PathSlBoundary& sl_boundary, const FrenetBox& obj_frenet_box,
    double ego_width, double lead_threshold, const std::string& id,
    const NudgeObjectInfo* nudge_object_info, const bool& is_lane_change,
    const std::pair<std::string, double>& nearest_obj,
    const DrivePassage& passage, const FrenetBox& ego_frenet_box,
    const ApolloTrajectoryPointProto& plan_start_point,
    const SpacetimeTrajectoryManager& st_traj_mgr, bool& l_space, bool& r_space,
    bool is_stalled, const double& obj_v, bool& is_emergency) {
  bool is_nearest = (id == nearest_obj.first) ? true : false;
  std::set<std::string> l_set, r_set;

  constexpr double kSampleStepAlongS = 1.0;  // m.
  double min_left_space = DBL_MAX, min_right_space = DBL_MAX;

  for (double sample_s = obj_frenet_box.s_min; sample_s <= obj_frenet_box.s_max;
       sample_s += kSampleStepAlongS) {
    const auto [right_l, left_l] = sl_boundary.QueryTargetBoundaryL(sample_s);
    min_left_space = std::clamp(
        left_l - std::max(right_l, obj_frenet_box.l_max), 0.0, min_left_space);
    min_right_space = std::clamp(
        std::min(left_l, obj_frenet_box.l_min) - right_l, 0.0, min_right_space);
  }
  // TODO: if ego in current lane, check ENV safety each frame
  bool ego_in_lane = true;
  double td = 0.8;  // s
  for (double sample_s = ego_frenet_box.s_min; sample_s <= ego_frenet_box.s_max;
       sample_s += kSampleStepAlongS) {
    const auto [right_l, left_l] = sl_boundary.QueryTargetBoundaryL(sample_s);
    if (ego_frenet_box.l_max > left_l || ego_frenet_box.l_min < right_l) {
      ego_in_lane = false;
      break;
    }
  }
  if (nudge_object_info && ego_in_lane) {
    td = 0.65;
  }
  if (is_nearest && !is_lane_change && (!nudge_object_info || ego_in_lane)) {
    IsEnvUnsafe(st_traj_mgr.trajectories(), passage, ego_frenet_box, l_set,
                r_set, plan_start_point, nearest_obj.second, id, td);
  }

  std::string obj_thr = absl::StrCat(
      "id: ", id, "leading_threshold: ", lead_threshold,
      " l_unsafe_set_size: ", l_set.size(),
      " r_unsafe_set_size: ", r_set.size(), " min_left_space: ", min_left_space,
      " min_right_space: ", min_right_space);

  if (is_stalled && !is_nearest) {
    Log2DDS::LogDataV2("obj_thr", obj_thr);
    l_space = true;
    r_space = true;
    return true;
  }
  if (is_stalled) {
    Log2DDS::LogDataV2("obj_thr", obj_thr);
    l_space = true;
    r_space = true;
    if (ego_frenet_box.center_l() >= obj_frenet_box.center_l() &&
        !l_set.empty())
      return false;
    if (ego_frenet_box.center_l() < obj_frenet_box.center_l() && !r_set.empty())
      return false;
    return true;
  }

  // const std::string obj_thr =
  //     absl::StrCat("id: ", id, "leading_threshold: ", lead_threshold,
  //                  " l_unsafe_set_size: ", ,l_set.size(),
  //                  " r_unsafe_set_size: ", r_set.size(),
  //                  " min_left_space: ", min_left_space,
  //                  " min_right_space: ", min_right_space);
  // Log2DDS::LogDataV2("obj_thr", obj_thr);
  l_space = min_left_space > (ego_width + lead_threshold);
  r_space = min_right_space > (ego_width + lead_threshold);

  if (!l_space && !r_space && nudge_object_info) {
    const double comfor_a = -2.0;
    if (obj_v < plan_start_point.v()) {
      double safe_s = (Sqr(obj_v) - Sqr(plan_start_point.v())) / (2 * comfor_a);
      double dist_to_obj = obj_frenet_box.s_min - ego_frenet_box.s_max;
      bool is_comf = dist_to_obj > safe_s;
      obj_thr += absl::StrCat(" safe_s: ", safe_s, " dist_to_obj ", dist_to_obj,
                              " is_comf: ", is_comf);
      Log2DDS::LogDataV2("obj_thr", obj_thr);
      if (!is_comf) {
        is_emergency = true;
        return true;
      }
    }
  }

  Log2DDS::LogDataV2("obj_thr", obj_thr);

  if (min_left_space > (ego_width + lead_threshold) && !nudge_object_info) {
    if (!l_set.empty()) {
      return false;
    }
  } else if (min_right_space > (ego_width + lead_threshold) &&
             !nudge_object_info) {
    if (!r_set.empty()) {
      return false;
    }
  }

  return std::max(min_left_space, min_right_space) >
         (ego_width + lead_threshold);
}

bool IsObjectAvoidableWithinSlBoundary(
    const PathSlBoundary& sl_boundary, const FrenetBox& obj_frenet_box,
    double ego_width, double lead_threshold, const std::string& id,
    bool is_out_boundary, const SpacetimeTrajectoryManager& st_traj_mgr,
    const ApolloTrajectoryPointProto& plan_start_point,
    const DrivePassage& passage, const FrenetBox& ego_frenet_box) {
  bool l_unsafe = false, r_unsafe = false;
  IsEnvUnsafe(st_traj_mgr.trajectories(), passage, ego_frenet_box, l_unsafe,
              r_unsafe, id, plan_start_point);
  constexpr double kSampleStepAlongS = 1.0;  // m.
  double min_left_space = DBL_MAX, min_right_space = DBL_MAX;
  for (double sample_s = obj_frenet_box.s_min; sample_s <= obj_frenet_box.s_max;
       sample_s += kSampleStepAlongS) {
    const auto [right_l, left_l] =
        is_out_boundary ? sl_boundary.QueryBoundaryL(sample_s)
                        : sl_boundary.QueryTargetBoundaryL(sample_s);
    min_left_space = std::clamp(
        left_l - std::max(right_l, obj_frenet_box.l_max), 0.0, min_left_space);
    min_right_space = std::clamp(
        std::min(left_l, obj_frenet_box.l_min) - right_l, 0.0, min_right_space);
  }
  const std::string obj_thr =
      absl::StrCat("id: ", id, "leading_threshold: ", lead_threshold,
                   " l_unsafe: ", l_unsafe);
  Log2DDS::LogDataV2("obj_thr", obj_thr);
  if (min_left_space > (ego_width + lead_threshold) && l_unsafe) return false;
  if (min_left_space > (ego_width + lead_threshold) && r_unsafe) return false;

  return std::max(min_left_space, min_right_space) >
         (ego_width + lead_threshold);
}

bool IsLargeVehicleAvoidableWithinSlBoundary(
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const PathSlBoundary& sl_boundary, const FrenetBox& obj_frenet_box,
    const SpacetimeObjectTrajectory& traj_ptr, const double ego_width) {
  // object speed
  if (traj_ptr.is_stationary()) return true;

  // object type
  const ObjectType obj_type = traj_ptr.planner_object().type();
  if (obj_type != OT_LARGE_VEHICLE) return true;

  // longitudinal distance
  if (obj_frenet_box.s_min - plan_start_point.path_point().s() > 15.0) {
    return true;
  }

  // latitudinal position
  // double obj_dl = obj_frenet_box.center_l() > 0.0
  //                     ? std::fmax(obj_frenet_box.l_max - 0.5 * ego_width,
  //                     0.0) : std::fmin(obj_frenet_box.l_min + 0.5 *
  //                     ego_width, 0.0);
  // if (std::fabs(obj_dl) > 0.45) return true;
  if (obj_frenet_box.l_max * obj_frenet_box.l_min < 0.0) return true;
  const auto [right_width, left_width] =
      sl_boundary.QueryTargetBoundaryL(obj_frenet_box.center_s());
  bool is_left = (obj_frenet_box.center_l() > 0.0);
  double target_l_lane = is_left ? left_width : right_width;
  double l_obj_edge = is_left ? obj_frenet_box.l_min : obj_frenet_box.l_max;
  if (std::fabs(l_obj_edge) > std::fabs(target_l_lane)) return true;

  // neighbor lane
  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  const auto& cur_lane = psmm.GetNearestLane(ego_pos);
  if (!cur_lane || !(cur_lane->junction_id() == 0)) return true;
  // const auto& neighbor_lane_id =
  //     is_left ? cur_lane->right_lane_id() : cur_lane->left_lane_id();
  const auto& neighbor_lane_id = cur_lane->right_lane_id();
  if (!(neighbor_lane_id == 0)) return true;

  // check left space within s-l boundary
  const double step_s = 1.0, lead_threshold = 1.4;
  const double preview_s = 10.0;
  double min_left_space = DBL_MAX, min_right_space = DBL_MAX;
  for (double sample_s = obj_frenet_box.s_min;
       sample_s <= obj_frenet_box.s_max + preview_s; sample_s += step_s) {
    // const auto [right_l, left_l] =
    // sl_boundary.QueryTargetBoundaryL(sample_s);
    const auto curb_offset_or = passage.QueryCurbOffsetAtS(sample_s);
    if (!curb_offset_or.ok()) continue;
    min_left_space =
        std::clamp(curb_offset_or->second -
                       std::max(curb_offset_or->first, obj_frenet_box.l_max),
                   0.0, min_left_space);
    min_right_space =
        std::clamp(std::min(curb_offset_or->second, obj_frenet_box.l_min) -
                       curb_offset_or->first,
                   0.0, min_right_space);
  }
  return std::max(min_left_space, min_right_space) >
         (ego_width + lead_threshold);
}

// Check whether object within traffic light controlled intersection.
bool IsObjectWithinTlControlledIntersection(
    const ad_byd::planning::TrafficLightStatusMap& tl_status_map,
    const PlannerSemanticMapManager& psmm, const FrenetBox& obj_frenet_box,
    const mapping::LanePath& lane_path, double s_offset) {
  const double const_zero = 1e-5;
  const double obj_center_s = obj_frenet_box.center_s();
  const double obj_center_l = obj_frenet_box.center_l();
  for (const auto& seg : lane_path) {
    const auto lane_info_ptr = psmm.map_ptr()->GetLaneById(seg.lane_id);
    if (lane_info_ptr == nullptr) continue;
    if (lane_info_ptr->junction_id() == 0) continue;
    // if (!IsTrafficLightControlledLane(*CHECK_NOTNULL(lane_info_ptr->proto)))
    // {
    //   return false;
    // }
    if (!tl_status_map.empty()) {
      const auto& tl_lane_info = tl_status_map.find(seg.lane_id);
      if (tl_status_map.end() == tl_lane_info) {
        Log2DDS::LogDataV2("traffic-wait", "tl_lane_info-0");
        return false;
      } else {
        const auto& traffic_light_status = tl_lane_info->second;
        if (traffic_light_status.junction_id.has_value() &&
            ad_byd::planning::LightStatus::NONE_LIGHT ==
                traffic_light_status.light_status) {
          Log2DDS::LogDataV2("traffic-wait", "NONE_LIGHT-0");
          return false;
        }
      }
    }
    // leftmost scenario
    if (lane_info_ptr->left_lane_id() == 0 && obj_center_l > 0.5 &&
        lane_info_ptr->turn_type() != TurnType::LEFT_TURN) {
      Log2DDS::LogDataV2("traffic-wait", "obj_center_l-0");
      return false;
    }
    // TODO: reopen by user
    if (seg.end_s + const_zero > obj_center_s && seg.start_s < obj_center_s) {
      double left_width = 0.0, right_width = 0.0;
      lane_info_ptr->GetWidthFromS(obj_center_s - seg.start_s, &left_width,
                                   &right_width);
      if (-right_width < obj_center_l && obj_center_l < left_width) {
        Log2DDS::LogDataV2("traffic-wait", "in-lane-1");
        return true;
      }
    }
    // for (const auto& [id, frac] : lane_info_ptr->Intersections()) {
    //   const auto* intersection_ptr = psmm.FindIntersectionByIdOrNull(id);
    //   if (intersection_ptr == nullptr ||
    //       !intersection_ptr->proto->traffic_light_controlled()) {
    //     continue;
    //   }

    //   const double intersection_start_s =
    //       lane_path.LaneIndexPointToArclength(seg.lane_index, frac.x());
    //   const double intersection_end_s =
    //       lane_path.LaneIndexPointToArclength(seg.lane_index, frac.y());
    //   if (obj_frenet_box.s_min < intersection_end_s + s_offset &&
    //       obj_frenet_box.s_max > intersection_start_s + s_offset) {
    //     return true;
    //   }
    // }
    if (seg.end_s + s_offset > obj_frenet_box.s_max) break;
  }
  return false;
}

bool IsObjectBlockingRefCenter(const PathSlBoundary& sl_boundary,
                               const FrenetBox& obj_frenet_box,
                               double ego_half_width) {
  const double obj_l_offset =
      sl_boundary.QueryReferenceCenterL(obj_frenet_box.center_s());

  return obj_frenet_box.l_max > obj_l_offset - ego_half_width &&
         obj_frenet_box.l_min < obj_l_offset + ego_half_width;
}

bool IsUnsafe(const FrenetBox& obj_frenet_box, const FrenetBox& ego_frenet_box,
              const SpacetimeObjectTrajectory& traj_ptr,
              const ApolloTrajectoryPointProto& plan_start_point,
              const std::string& obj_id, bool is_nudge,
              const NudgeObjectInfo* nudge_object_info,
              const bool& is_lane_change, const double& ego_width,
              const double& ego_length, const DrivePassage& passage) {
  double lon_dist = obj_frenet_box.s_min - ego_frenet_box.s_max;
  double lat_overlap = 0.0;
  double front_safe_dis = 0.0;
  std::vector<double> lat_threshold = {0.1, 2.0};
  std::vector<double> lon_dis = {0.2, 6.0};
  const double ego_heading_angle = plan_start_point.path_point().theta();
  const double object_heading_angle = traj_ptr.pose().theta();
  const double angle_diff =
      std::abs(NormalizeAngle(ego_heading_angle - object_heading_angle));

  const auto obj_box = traj_ptr.bounding_box();
  const auto ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  st::Box2d ego_box(ego_pos, plan_start_point.path_point().theta(), ego_length,
                    ego_width);
  auto& objects_proto = traj_ptr.planner_object().object_proto();
  st::Box2d obj_boxV2({objects_proto.pos().x(), objects_proto.pos().y()},
                      objects_proto.bounding_box().heading(),
                      objects_proto.bounding_box().length(),
                      objects_proto.bounding_box().width());
  const auto right_front_conner = passage.QueryUnboundedFrenetCoordinateAt(
      ego_box.GetCorner(Box2d::Corner::FRONT_RIGHT));

  const auto left_front_conner = passage.QueryUnboundedFrenetCoordinateAt(
      ego_box.GetCorner(Box2d::Corner::FRONT_LEFT));

  const auto right_rear_conner = passage.QueryUnboundedFrenetCoordinateAt(
      obj_box.GetCorner(Box2d::Corner::REAR_RIGHT));

  const auto left_rear_conner = passage.QueryUnboundedFrenetCoordinateAt(
      obj_box.GetCorner(Box2d::Corner::REAR_LEFT));

  const auto right_rear_connerV2 = passage.QueryUnboundedFrenetCoordinateAt(
      obj_boxV2.GetCorner(Box2d::Corner::REAR_RIGHT));

  const auto left_rear_connerv2 = passage.QueryUnboundedFrenetCoordinateAt(
      obj_boxV2.GetCorner(Box2d::Corner::REAR_LEFT));

  double left_front_l = DBL_MAX, right_front_l = DBL_MAX;
  double obj_left_rear_l = DBL_MAX, obj_right_rear_l = DBL_MAX;

  double obj_left_rear_l_V2 = DBL_MAX, obj_right_rear_l_V2 = DBL_MAX;

  if (right_front_conner.ok()) right_front_l = right_front_conner.value().l;
  if (left_front_conner.ok()) left_front_l = left_front_conner.value().l;

  if (right_rear_conner.ok()) obj_right_rear_l = right_rear_conner.value().l;
  if (left_rear_conner.ok()) obj_left_rear_l = left_rear_conner.value().l;

  if (right_rear_connerV2.ok())
    obj_right_rear_l_V2 = right_rear_connerV2.value().l;
  if (left_rear_connerv2.ok())
    obj_left_rear_l_V2 = left_rear_connerv2.value().l;

  if (obj_frenet_box.l_max < ego_frenet_box.l_max &&
      obj_frenet_box.l_max >= ego_frenet_box.l_min) {
    // lat_overlap = obj_frenet_box.l_max - ego_frenet_box.l_min;

    // lat_overlap = obj_frenet_box.l_max -
    //               (ego_frenet_box.center_l() +
    //                front_edge_to_center * std::sin(ego_heading_angle) -
    //                half_ego_width * std::cos(ego_heading_angle));

    // if (right_front_conner.ok() && left_rear_conner.ok()) {
    //   lat_overlap = obj_left_rear_l - right_front_l;
    // }
    if (right_front_conner.ok() && left_rear_connerv2.ok()) {
      lat_overlap = obj_left_rear_l_V2 - right_front_l;
    }
  } else if (ego_frenet_box.l_max < obj_frenet_box.l_max &&
             ego_frenet_box.l_max >= obj_frenet_box.l_min) {
    // lat_overlap = ego_frenet_box.l_max - obj_frenet_box.l_min;

    // lat_overlap = ego_frenet_box.center_l() +
    //               half_ego_width / std::cos(ego_heading_angle) +
    //               (front_edge_to_center * std::cos(ego_heading_angle) -
    //                half_ego_width * std::sin(ego_heading_angle)) -
    //               obj_frenet_box.l_min;

    // if (left_front_conner.ok()) {
    //   lat_overlap = left_front_conner.value().l - obj_frenet_box.l_min;
    // }

    if (ego_frenet_box.l_min >= obj_frenet_box.l_min) {
      // if (right_front_conner.ok() && left_rear_conner.ok()) {
      //   lat_overlap = obj_left_rear_l - right_front_l;
      // }
      if (right_front_conner.ok() && left_rear_connerv2.ok()) {
        lat_overlap = obj_left_rear_l_V2 - right_front_l;
      }
    } else {
      // if (left_front_conner.ok() && right_rear_conner.ok()) {
      //   lat_overlap = left_front_l - obj_right_rear_l;
      // }

      if (left_front_conner.ok() && right_rear_connerV2.ok()) {
        lat_overlap = left_front_l - obj_right_rear_l_V2;
      }
    }
  }

  if (lat_overlap == 0.0) return false;
  front_safe_dis =
      ad_byd::planning::math::interp1_inc(lat_threshold, lon_dis, lat_overlap);
  const std::string nearest = absl::StrCat(
      "obj_id: ", obj_id, " lon_dist: ", lon_dist,
      " lat_overlap: ", lat_overlap, " front_safe_dis: ", front_safe_dis,
      " is_nudge: ", is_nudge, " angle_diff: ", angle_diff,
      " ego_left_coner: ", left_front_l, " ego_right_coner: ", right_front_l,
      " obj_left_rear_coner: ", obj_left_rear_l, " obj_left_rear_conerv2 ",
      obj_left_rear_l_V2, " obj_right_rear_coner: ", obj_right_rear_l,
      " obj_right_rear_conerv2 ", obj_right_rear_l_V2,
      " ego_heading_angle: ", ego_heading_angle);
  Log2DDS::LogDataV2("nearest", nearest);

  if (!is_lane_change && !nudge_object_info && lon_dist < front_safe_dis &&
      0.0 < lon_dist /*&& angle_diff < (M_PI / 18)*/)
    return true;
  return false;
}

bool IsObjectBlockingEgoVehicle(const PathSlBoundary& sl_boundary,
                                const FrenetBox& obj_frenet_box,
                                const FrenetBox& ego_frenet_box
                               /* const NudgeObjectInfo* nudge_object_info,
                                const double& nearest_stop_s,const SpacetimeObjectTrajectory& traj_ptr*/) {
  // double lat_cmp = 0.0;
  // if (nudge_object_info) {
  // }
  const double ego_l_offset =
      sl_boundary.QueryReferenceCenterL(ego_frenet_box.center_s());
  const double obj_l_offset =
      sl_boundary.QueryReferenceCenterL(obj_frenet_box.center_s());

  // Check object lateral position.
  if (obj_frenet_box.l_max - obj_l_offset <
          ego_frenet_box.l_min - ego_l_offset ||
      obj_frenet_box.l_min - obj_l_offset >
          ego_frenet_box.l_max - ego_l_offset) {
    return false;
  }
  return true;
}

bool VruSelectLeading(const double& obj_v, const FrenetBox& obj_frenet_box,
                      const ApolloTrajectoryPointProto& plan_start_point,
                      const FrenetBox& ego_frenet_box, const bool pre_unlead,
                      std::string& vru_debug) {
  if (!pre_unlead || obj_v > plan_start_point.v()) return true;
  const double comfor_a = -1.0;
  const double lon_buffer = pre_unlead ? obj_v * 1.0 : 0.0;
  double safe_s =
      (Sqr(obj_v) - Sqr(plan_start_point.v())) / (2 * comfor_a) + lon_buffer;
  vru_debug += absl::StrCat(" safe_s: ", safe_s);
  if (obj_frenet_box.s_min - ego_frenet_box.s_max > safe_s) return true;
  return false;
}

bool VruUnsafe(const FrenetBox& obj_frenet_box,
               const SpacetimeObjectTrajectory& traj_ptr, const bool pre_unlead,
               std::string& vru_debug,
               const ApolloTrajectoryPointProto& plan_start_point,
               const FrenetBox& ego_frenet_box, const std::string& obj_id) {
  const double obj_width = traj_ptr.bounding_box().width();
  const double slow_lead_th = pre_unlead ? -std::fmin(obj_width, 0.5) + 0.5
                                         : -std::fmin(obj_width, 0.5);
  const double normal_lead_th = pre_unlead ? 0.5 : 0.0;
  vru_debug += absl::StrCat(" obj_id: ", obj_id, " pre_unleading", pre_unlead);
  if (traj_ptr.pose().v() < 10.0) {
    if (obj_frenet_box.center_l() > slow_lead_th &&
        VruSelectLeading(traj_ptr.pose().v(), obj_frenet_box, plan_start_point,
                         ego_frenet_box, pre_unlead, vru_debug)) {
      vru_debug += "center_l";
      return true;
    }
  } else if (obj_frenet_box.l_max > normal_lead_th &&
             VruSelectLeading(traj_ptr.pose().v(), obj_frenet_box,
                              plan_start_point, ego_frenet_box, pre_unlead,
                              vru_debug)) {
    vru_debug += "l_max";
    return true;
  }
  return false;
}

std::string find_nearest_stall(
    const SpacetimeTrajectoryManager& st_traj_mgr, const DrivePassage& passage,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const FrenetBox& ego_frenet_box) {
  std::string nearest_obj_id = "";
  if (stalled_objects.empty()) return nearest_obj_id;
  double nearest_stall_s = DBL_MAX;
  for (auto& stall_obj : stalled_objects) {
    const auto temp_nearest_obj = st_traj_mgr.FindObjectByObjectId(stall_obj);
    if (temp_nearest_obj) {
      if (temp_nearest_obj->type() != ObjectType::OT_VEHICLE &&
          temp_nearest_obj->type() != ObjectType::OT_LARGE_VEHICLE)
        continue;
      const auto stall_obj_box =
          passage.QueryFrenetBoxAtContour(temp_nearest_obj->contour());
      if (!stall_obj_box.ok() ||
          stall_obj_box.value().l_max < ego_frenet_box.l_min - 2.0 ||
          stall_obj_box.value().l_min > ego_frenet_box.l_max + 2.0 ||
          stall_obj_box.value().center_s() < ego_frenet_box.center_s() - 20.0)
        continue;
      bool UnOverlap = (stall_obj_box.value().l_min > ego_frenet_box.l_max ||
                        stall_obj_box.value().l_max < ego_frenet_box.l_min);
      if (ego_frenet_box.s_min > stall_obj_box.value().s_max && !UnOverlap)
        continue;

      if (stall_obj_box.value().center_s() < nearest_stall_s) {
        nearest_stall_s = stall_obj_box.value().center_s();
        nearest_obj_id = stall_obj;
      }
    }
  }

  return nearest_obj_id;
}

// const std::string FindNearestStationaryCar(ObjectsOnLane& st_trajs_on_lane,
//                                            const FrenetBox& ego_frenet_box,
//                                            const DrivePassage& passage) {
//   for (const auto& [obj_frenet_box, traj_ptr] : st_trajs_on_lane) {
//     bool JudgeType =
//         ((traj_ptr->planner_object().type() == ObjectType::OT_VEHICLE ||
//           traj_ptr->planner_object().type() == ObjectType::OT_LARGE_VEHICLE)
//           &&
//          traj_ptr->is_stationary());
//     if (!JudgeType) continue;
//     if (obj_frenet_box.center_l() <= 0.0) {
//       if (obj_frenet_box.l_max >= ego_frenet_box.l_min - 0.6)
//         return traj_ptr->planner_object().id();
//     } else {
//       if (obj_frenet_box.l_min >= ego_frenet_box.l_max + 0.6)
//         return traj_ptr->planner_object().id();
//     }
//   }
//   return "";
// }

std::pair<std::string, double> FindNearestStationaryCar(
    const ObjectsOnLane& st_trajs_on_lane, const FrenetBox& ego_frenet_box,
    bool is_vru, const ApolloTrajectoryPointProto& plan_start_point) {
  std::pair<std::string, double> nearest_obj;
  for (const auto& [obj_frenet_box, traj_ptr] : st_trajs_on_lane) {
    const std::string obj_id = traj_ptr->planner_object().id();
    if (traj_ptr->pose().v() > plan_start_point.v() &&
        plan_start_point.v() >= 0.6)
      continue;
    const double comfor_a = -3.0;
    double safe_s = (Sqr(traj_ptr->pose().v()) - Sqr(plan_start_point.v())) /
                    (2 * comfor_a);
    std::string nearest_debug =
        absl::StrCat(" obj_id: ", obj_id, " safe_s: ", safe_s);
    bool is_comf = obj_frenet_box.s_min - ego_frenet_box.s_max > safe_s;
    nearest_debug += absl::StrCat(" is_comf: ", is_comf);
    Log2DDS::LogDataV0("nearest_debug", nearest_debug);
    bool JudgeType =
        ((traj_ptr->planner_object().type() == ObjectType::OT_VEHICLE ||
          traj_ptr->planner_object().type() == ObjectType::OT_LARGE_VEHICLE) &&
         is_comf) /*||is_vru*/;
    bool UnOverlap = (obj_frenet_box.l_min > ego_frenet_box.l_max ||
                      obj_frenet_box.l_max < ego_frenet_box.l_min);
    if (!JudgeType /*|| UnOverlap*/) continue;
    // const std::string obj_id = traj_ptr->planner_object().id();
    if (obj_frenet_box.center_l() <= 0.0) {
      if (obj_frenet_box.l_max >= ego_frenet_box.l_min - 1.4) {
        // double lat_thr = (std::ceil((obj_frenet_box.l_max + 0.6 -
        // ego_frenet_box.l_min)/0.1) + 1)*0.1;
        double lat_thr =
            (RoundToInt((obj_frenet_box.l_max + 1.4 - ego_frenet_box.l_min) /
                        0.1) +
             1) *
            0.1;
        nearest_obj = std::make_pair(obj_id, lat_thr);
        return nearest_obj;
      }
    } else {
      if (obj_frenet_box.l_min <= ego_frenet_box.l_max + 1.4) {
        // double lat_thr = (std::ceil((ego_frenet_box.l_max -
        // obj_frenet_box.l_min + 0.6)/0.1) + 1)*0.1;
        double lat_thr =
            (RoundToInt((ego_frenet_box.l_max - obj_frenet_box.l_min + 1.4) /
                        0.1) +
             1) *
            0.1;
        nearest_obj = std::make_pair(obj_id, lat_thr);
        return nearest_obj;
      }
    }
  }
  return nearest_obj;
}

bool IsStandSide(const st::FrenetBox& obj_frenet_box,
                 const ad_byd::planning::LaneConstPtr& lane_info_ptr,
                 bool traffic_waiting, double lead_threshold, double angle_diff,
                 bool highway_mode, bool is_rightmost_lane,
                 std::string obj_id) {
  if (lane_info_ptr == nullptr) return false;
  constexpr double kEpsilon = 1e-5;
  double lane_width = 0.0;
  double angle_buffer = std::clamp(angle_diff * 5.0, -1.0, 0.3);
  // differentiate rightmost & none rightmost lane buffer
  double offset_buffer = traffic_waiting     ? 0.0 + angle_buffer
                         : highway_mode      ? -0.2 + angle_buffer
                         : is_rightmost_lane ? 0.6 + angle_buffer
                                             : 0.2 + angle_buffer;
  lane_width = lane_info_ptr->GetWidthAtAccumS(obj_frenet_box.center_s());
  std::string stand_side_debug = absl::StrCat(
      " obj_id: ", obj_id, " traffic_waiting: ", traffic_waiting,
      " highway_mode: ", highway_mode, " is_rightmost_lane, ",
      is_rightmost_lane, " angle_buffer: ", angle_buffer, " offset_buffer, ",
      offset_buffer, " lead_threshold: ", lead_threshold,
      " lat_pos: ", obj_frenet_box.center_l() + offset_buffer - lead_threshold,
      " half_width: ", 0.5 * lane_width);
  Log2DDS::LogDataV0("stand_side_debug", stand_side_debug);
  return lane_width < kEpsilon
             ? false
             : obj_frenet_box.center_l() + offset_buffer - lead_threshold >
                       0.5 * lane_width ||
                   obj_frenet_box.center_l() - offset_buffer + lead_threshold <
                       -0.5 * lane_width;
}

}  // namespace

std::vector<ConstraintProto::LeadingObjectProto> FindLeadingObjects(
    const ad_byd::planning::TrafficLightStatusMap& tl_status_map,
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage,
    const PathSlBoundary& sl_boundary, LaneChangeStage lc_stage,
    const SceneOutputProto& scene_reasoning,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const ApolloTrajectoryPointProto& plan_start_point,
    const st::VehicleGeometryParamsProto& vehicle_geometry_params,
    const FrenetBox& ego_frenet_box, bool borrow_lane_boundary,
    const ObjectHistoryManager* obs_his_manager,
    std::map<std::string, bool>& obj_lead,
    const NudgeObjectInfo* nudge_object_info, const bool& is_lane_change,
    const double& nearest_stop_s, bool* is_first_lead, bool& must_borrow) {
  std::vector<ConstraintProto::LeadingObjectProto> leading_objects;

  // Collect traffic waiting objects on current lane path.
  const auto traffic_waiting_objects = CollectTrafficWaitingObjectOnCurrentLane(
      scene_reasoning, passage.lane_path());

  absl::flat_hash_set<std::string> collect_stalled;

  // Find leading objects on current lane.
  const auto st_trajs_on_lane =
      FindFrontObjectsOnLane(passage, sl_boundary, st_traj_mgr.trajectories(),
                             ego_frenet_box, collect_stalled, stalled_objects);
  leading_objects.reserve(st_trajs_on_lane.size());

  const auto& lane_info_ptr =
      psmm.FindCurveLaneByIdOrNull(passage.lane_path().front().lane_id());

  bool is_leftmost_lane = false, is_rightmost_lane = false;
  if (lane_info_ptr != nullptr) {
    is_leftmost_lane = lane_info_ptr->left_lane_id() == 0;
    is_rightmost_lane = lane_info_ptr->right_lane_id() == 0;
  }

  const double front_edge_to_center =
      vehicle_geometry_params.front_edge_to_center();
  const double ego_width = vehicle_geometry_params.width();
  const double ego_half_width = 0.5 * ego_width;
  const bool lc_ongoing = (lc_stage == LaneChangeStage::LCS_EXECUTING ||
                           lc_stage == LaneChangeStage::LCS_RETURN);
  double stdll_s = -20.0;
  std::string stall_obj_debug = "";
  // if (!stalled_objects.empty()) {
  std::string stall_id =
      find_nearest_stall(st_traj_mgr, passage, collect_stalled, ego_frenet_box);
  stall_obj_debug += absl::StrCat(" stall_id: ", stall_id);
  const auto stll_obj = st_traj_mgr.FindObjectByObjectId(stall_id);
  if (stll_obj) {
    const auto stall_obj_box =
        passage.QueryFrenetBoxAtContour(stll_obj->contour());
    const std::string stall_obj = stll_obj->id();
    stall_obj_debug += absl::StrCat(" stall_obj_id: ", stall_obj);
    // stdll_s = stall_obj_box.ok() && stall_obj_box.value().center_s() >
    //                                     ego_frenet_box.center_s()
    //               ? stall_obj_box.value().center_s()
    //               : 0.0;
    stdll_s = stall_obj_box.value().center_s();
    if (stdll_s > 0.0 && stdll_s < 50.0 && is_rightmost_lane &&
        nudge_object_info && is_leftmost_lane) {
      must_borrow = true;
    }

    stall_obj_debug += absl::StrCat(" ", stdll_s);
  }
  // }

  // std::set<std::string> l_set, r_set;
  // IsEnvUnsafe(st_traj_mgr.trajectories(),passage,ego_frenet_box,l_set,r_set,plan_start_point);

  // Create forbidden to nudge leading objects.

  for (const auto& [obj_frenet_box, traj_ptr] : st_trajs_on_lane) {
    const auto& obj_id = traj_ptr->planner_object().id();
    auto obs_histroy_info = obs_his_manager->GetObjLatestFrame(obj_id);
    bool is_stalled = obs_histroy_info && obs_histroy_info->is_stalled;
    double lead_threshold = 0.0;
    bool nudge_obj = obs_histroy_info && obs_histroy_info->is_nudge;
    if (obs_histroy_info && !obs_histroy_info->is_leading)
      lead_threshold = -0.3;
    // calculate angle diff
    auto obj_lane_pt =
        passage.lane_path().ArclengthToLanePoint(obj_frenet_box.center_s());
    double obj_pt_angle =
        ComputeLanePointTangent(psmm, obj_lane_pt).FastAngle();
    double object_heading_angle = traj_ptr->pose().theta();
    double angle_diff = NormalizeAngle(obj_pt_angle - object_heading_angle);
    // left cut in : negative, right cut in : positive
    angle_diff *= obj_frenet_box.center_l() < 0.0 ? 1.0 : -1.0;
    // rightmost lane
    is_rightmost_lane =
        IsRightMostLane(psmm, passage.lane_path(), obj_frenet_box.center_s());
    bool highway_mode = psmm.IsOnHighway() && !is_rightmost_lane;
    // Filter objects by ego heading.
    if (IsOncomingObjectJudgeByEgoHeading(plan_start_point, traj_ptr->pose()) &&
        stalled_objects.find(obj_id) == stalled_objects.end()) {
      obj_lead[obj_id] = false;
      continue;
    }

    bool is_vru_type =
        (traj_ptr->planner_object().type() == ObjectType::OT_CYCLIST ||
         traj_ptr->planner_object().type() == ObjectType::OT_TRICYCLIST)
            ? true
            : false;

    bool JudgeType =
        ((traj_ptr->planner_object().type() == ObjectType::OT_VEHICLE ||
          traj_ptr->planner_object().type() == ObjectType::OT_LARGE_VEHICLE) &&
         traj_ptr->is_stationary());

    bool borrow_consider = false;
    bool is_brake_light = traj_ptr->planner_object()
                              .object_proto()
                              .obstacle_light()
                              .brake_lights() == ObstacleLightType::LIGHT_ON;
    if (nudge_object_info &&
        nudge_object_info->nudge_state == NudgeObjectInfo::NudgeState::BORROW &&
        nudge_object_info->id != obj_id &&
        (is_brake_light || obj_frenet_box.l_max * obj_frenet_box.l_min < 0.0)) {
      borrow_consider = true;
    }

    const auto& nearest_obj = FindNearestStationaryCar(
        st_trajs_on_lane, ego_frenet_box, is_vru_type, plan_start_point);

    if (IsUnsafe(obj_frenet_box, ego_frenet_box, *traj_ptr, plan_start_point,
                 obj_id, nudge_obj, nudge_object_info, is_lane_change,
                 ego_width, vehicle_geometry_params.length(), passage) &&
        !is_vru_type) {
      leading_objects.push_back(CreateLeadingObject(
          *traj_ptr, passage,
          ConstraintProto::LeadingObjectProto::UNABLE_TO_OVERTAKE));
      obj_lead[obj_id] = true;
      continue;
    }

    if (traj_ptr->is_stationary() && stdll_s > -20.0 &&
        obj_frenet_box.center_s() > stdll_s &&
        obj_frenet_box.center_s() < stdll_s + 25.0 && obj_id != stall_id) {
      const auto stall_obj_box =
          passage.QueryFrenetBoxAtContour(stll_obj->contour());
      if (!(stall_obj_box.value().l_max < obj_frenet_box.l_min + 0.5 ||
            stall_obj_box.value().l_min > obj_frenet_box.l_max - 0.5)) {
        obj_lead[obj_id] = false;
        stall_obj_debug += absl::StrCat(
            " obj_id: ", obj_id, " obj_pose_s: ", obj_frenet_box.center_s());
        Log2DDS::LogDataV2("stall_obj_debug", stall_obj_debug);
        continue;
      }
    }
    stall_obj_debug += absl::StrCat(" obj_id: ", obj_id);
    Log2DDS::LogDataV2("stall_obj_debug", stall_obj_debug);
    stall_obj_debug = "";

    // cutin object filter
    if (IsCutInObjectTrajectory(passage, is_lane_change, plan_start_point.v(),
                                ego_frenet_box, *traj_ptr)) {
      continue;
    }

    // Sl boundary check: unavoidable object is leading object.

    // bool slboundary_check = IsObjectAvoidableWithinSlBoundary(
    //     sl_boundary, obj_frenet_box, ego_width, lead_threshold, obj_id,
    //     false, l_set, r_set, nudge_object_info, is_lane_change, JudgeType,
    //     ego_frenet_box);
    bool l_space_enough = false, r_space_enough = false,
         is_emergency_obj = false;
    bool slboundary_check = IsObjectAvoidableWithinSlBoundary(
        sl_boundary, obj_frenet_box, ego_width, lead_threshold, obj_id,
        nudge_object_info, is_lane_change, nearest_obj, passage, ego_frenet_box,
        plan_start_point, st_traj_mgr, l_space_enough, r_space_enough,
        is_stalled, traj_ptr->pose().v(), is_emergency_obj);

    bool is_obj_env = IsObjectAvoidableWithinEnv(
        passage, obj_frenet_box, obj_id, st_traj_mgr.trajectories(),
        l_space_enough, r_space_enough, nudge_object_info, ego_frenet_box,
        traj_ptr->is_stationary());

    if (!is_vru_type && (!slboundary_check || !is_obj_env)) {
      leading_objects.push_back(CreateLeadingObject(
          *traj_ptr, passage,
          ConstraintProto::LeadingObjectProto::UNABLE_TO_OVERTAKE));
      obj_lead[obj_id] = true;
      continue;
    }

    //  Stall object is not leading object.
    if (stalled_objects.contains(obj_id)) {
      obj_lead[obj_id] = false;
      continue;
    }

    // Large Vehicle check:
    bool large_vehicle_check = IsLargeVehicleAvoidableWithinSlBoundary(
        psmm, passage, plan_start_point, sl_boundary, obj_frenet_box, *traj_ptr,
        ego_width);

    if (!is_vru_type && !large_vehicle_check) {
      leading_objects.push_back(CreateLeadingObject(
          *traj_ptr, passage,
          ConstraintProto::LeadingObjectProto::UNABLE_TO_OVERTAKE));
      obj_lead[obj_id] = true;
      continue;
    }

    // Curb check: unavoidable object is leading object.
    if (!is_vru_type &&
        !IsObjectAvoidableWithinCurbBuffer(passage, obj_frenet_box, sl_boundary,
                                           lead_threshold, obj_id, ego_width,
                                           l_space_enough, r_space_enough) &&
        !is_emergency_obj) {
      leading_objects.push_back(CreateLeadingObject(
          *traj_ptr, passage,
          ConstraintProto::LeadingObjectProto::UNABLE_TO_OVERTAKE));
      obj_lead[obj_id] = true;
      continue;
    }

    // Traffic waiting object on current lane path is leading object.
    if (traffic_waiting_objects.contains(obj_id) && !is_vru_type &&
        !IsStandSide(obj_frenet_box, lane_info_ptr, true, lead_threshold,
                     angle_diff, highway_mode, is_rightmost_lane, obj_id)) {
      std::string traffic_debug =
          "obj_id: " + obj_id + " traffic waiting leading";
      Log2DDS::LogDataV0("traffic_debug", traffic_debug);
      leading_objects.push_back(CreateLeadingObject(
          *traj_ptr, passage,
          ConstraintProto::LeadingObjectProto::TRAFFIC_WAITING));
      obj_lead[obj_id] = true;
      continue;
    }

    bool borrow_debug =
        !lc_ongoing &&
        IsObjectBlockingEgoVehicle(sl_boundary, obj_frenet_box, ego_frenet_box);
    std::string borrow_debug_string = absl::StrCat(borrow_debug, " ", obj_id);
    Log2DDS::LogDataV0("borrow_debug", borrow_debug_string);

    if ((lc_ongoing && IsObjectBlockingRefCenter(sl_boundary, obj_frenet_box,
                                                 ego_half_width)) ||
        (!lc_ongoing && IsObjectBlockingEgoVehicle(sl_boundary, obj_frenet_box,
                                                   ego_frenet_box))) {
      // All blocking objects within borrow-lane sl boundary.
      std::string vru_debug = " ";
      if ((borrow_lane_boundary && !is_vru_type) || is_vru_type) {
        if (is_vru_type) {
          const bool pre_unlead =
              obs_histroy_info && !obs_histroy_info->is_leading;
          std::set<std::string> l_set, r_set;
          double lat_thr = obj_frenet_box.l_max - ego_frenet_box.l_min + 1.0;
          IsEnvUnsafe(st_traj_mgr.trajectories(), passage, ego_frenet_box,
                      l_set, r_set, plan_start_point, lat_thr, obj_id);
          if (VruUnsafe(obj_frenet_box, *traj_ptr, pre_unlead, vru_debug,
                        plan_start_point, ego_frenet_box, obj_id) ||
              ((l_set.size() > 1 ||
                (l_set.size() == 1 && l_set.find(obj_id) == l_set.end())) &&
               !nudge_object_info)) {
            leading_objects.push_back(CreateLeadingObject(
                *traj_ptr, passage,
                ConstraintProto::LeadingObjectProto::UNABLE_TO_OVERTAKE));
            vru_debug += absl::StrCat(" obj_id: ", obj_id, "-", pre_unlead);
            Log2DDS::LogDataV0("vru_debug", vru_debug);
            obj_lead[obj_id] = true;
          } else {
            Log2DDS::LogDataV0("vru_debug", vru_debug);
            obj_lead[obj_id] = false;
          }
          continue;
        } else if (!traj_ptr->is_stationary() || !nudge_object_info ||
                   nudge_object_info->nudge_state ==
                       NudgeObjectInfo::NudgeState::NUDGE ||
                   borrow_consider) {
          leading_objects.push_back(CreateLeadingObject(
              *traj_ptr, passage,
              ConstraintProto::LeadingObjectProto::BORROW_BOUNDARY));
          obj_lead[obj_id] = true;
          continue;
        } else {
          Log2DDS::LogDataV0("unleading", obj_id);
        }
      }
      // All blocking objects within a traffic light controlled intersection.
      bool veh_type_condition =
          traj_ptr->planner_object().type() == ObjectType::OT_VEHICLE ||
          traj_ptr->planner_object().type() == ObjectType::OT_LARGE_VEHICLE;
      if (veh_type_condition &&
          IsObjectWithinTlControlledIntersection(
              tl_status_map, psmm, obj_frenet_box, passage.lane_path(),
              passage.lane_path_start_s())) {
        leading_objects.push_back(CreateLeadingObject(
            *traj_ptr, passage,
            ConstraintProto::LeadingObjectProto::INTERSECTION));
        obj_lead[obj_id] = true;
        continue;
      }
    }
    // unstalled obj with low offset to center line is leading obj, regardness
    // of it's avoidable or not
    if (!traffic_waiting_objects.contains(obj_id) && !is_vru_type &&
        !IsStandSide(obj_frenet_box, lane_info_ptr, false, lead_threshold,
                     angle_diff, highway_mode, is_rightmost_lane, obj_id)) {
      std::string offset_debug = "obj_id: " + obj_id + " offset too low ";
      Log2DDS::LogDataV0("offset_debug", offset_debug);
      leading_objects.push_back(CreateLeadingObject(
          *traj_ptr, passage,
          ConstraintProto::LeadingObjectProto::UNABLE_TO_OVERTAKE));
      obj_lead[obj_id] = true;
      continue;
    }

    obj_lead[obj_id] = false;

    // TODO: more strategy to make leading object decision.
  }

  // check whether the first front obj is leading obj
  if (nullptr != is_first_lead && !st_trajs_on_lane.empty()) {
    const auto& first_obj_id =
        st_trajs_on_lane.front().second->planner_object().id();
    *is_first_lead =
        (obj_lead.end() != obj_lead.find(first_obj_id) &&
         (obj_lead[first_obj_id] || st_trajs_on_lane.front().first.s_max +
                                            vehicle_geometry_params.length() >
                                        nearest_stop_s));
    if (*is_first_lead && !obj_lead[first_obj_id] &&
        st_trajs_on_lane.front().first.l_max > ego_frenet_box.l_min &&
        st_trajs_on_lane.front().first.l_min < ego_frenet_box.l_max) {
      leading_objects.push_back(CreateLeadingObject(
          *st_trajs_on_lane.front().second, passage,
          ConstraintProto::LeadingObjectProto::AFTER_STOPLINE));
      obj_lead[first_obj_id] = true;
    }
  }

  return leading_objects;
}

}  // namespace planning
}  // namespace st
