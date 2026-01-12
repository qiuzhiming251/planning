#ifndef _PLAN_COMMON_SERIALIZATION_BASE_SERIALIZATION_H_
#define _PLAN_COMMON_SERIALIZATION_BASE_SERIALIZATION_H_

#include <cereal/cereal.hpp>

#include "absl/container/flat_hash_map.h"
#include "absl/time/time.h"

#include "modules/cnoa_pnc/planning/proto/autonomy_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/optimizer.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/selector_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory.pb.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"
#include "modules/cnoa_pnc/planning/proto/gaming.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "modules/cnoa_pnc/planning/proto/pnp_info.pb.h"
#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer_config.pb.h"

#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/aabox2d.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/line_curve2d.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/vec2d.h"
#include "plan_common/math/vec3d.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/geometry/aabox2d.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/type_def.h"

#include <cereal/types/atomic.hpp>

namespace cereal {

template <typename Archive, typename T>
void serialize(Archive& ar, st::Vec2<T>& vec2) {
  ar(CEREAL_NVP(vec2.x()));
  ar(CEREAL_NVP(vec2.y()));
}

template <typename Archive, typename T>
void serialize(Archive& ar, st::Vec3<T>& vec3) {
  ar(CEREAL_NVP(vec3.x()));
  ar(CEREAL_NVP(vec3.y()));
  ar(CEREAL_NVP(vec3.z()));
}

template <typename Archive>
void serialize(Archive& ar, st::planning::TrajectoryProto& trajectory) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    trajectory.ParseFromString(serialized_proto);
  } else {
    trajectory.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar, st::planning::DeciderStateProto& decider_state) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    decider_state.ParseFromString(serialized_proto);
  } else {
    decider_state.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::InitializerStateProto& initializer_state) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    initializer_state.ParseFromString(serialized_proto);
  } else {
    initializer_state.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::SpacetimePlannerObjectTrajectoriesProto&
                   st_planner_obj_trajs) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    st_planner_obj_trajs.ParseFromString(serialized_proto);
  } else {
    st_planner_obj_trajs.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::SpeedFinderStateProto& speed_finder_state) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    speed_finder_state.ParseFromString(serialized_proto);
  } else {
    speed_finder_state.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::TrajectoryOptimizerStateProto& traj_opt_state) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    traj_opt_state.ParseFromString(serialized_proto);
  } else {
    traj_opt_state.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar, st::FrenetCoordinate& frenet_coordinate) {
  ar(CEREAL_NVP(frenet_coordinate.s));
  ar(CEREAL_NVP(frenet_coordinate.l));
}

template <typename Archive>
void serialize(Archive& ar, st::AutonomyStateProto& autonomy_state) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    autonomy_state.ParseFromString(serialized_proto);
  } else {
    autonomy_state.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar, st::LaneChangeStateProto& lane_change_state) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    lane_change_state.ParseFromString(serialized_proto);
  } else {
    lane_change_state.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar, st::ObjectProto& objectProto) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    objectProto.ParseFromString(serialized_proto);
  } else {
    objectProto.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar, st::PushStatusProto& push_status_proto) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    push_status_proto.ParseFromString(serialized_proto);
  } else {
    push_status_proto.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::PausePushSavedOffsetProto& saved_offset) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    saved_offset.ParseFromString(serialized_proto);
  } else {
    saved_offset.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::LaneChangeStyleDeciderResultProto& lc_style_decider) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    lc_style_decider.ParseFromString(serialized_proto);
  } else {
    lc_style_decider.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(
    Archive& ar,
    st::planning::TaskSafetyEvaluationProto& task_safety_evaluation) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    task_safety_evaluation.ParseFromString(serialized_proto);
  } else {
    task_safety_evaluation.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::DrivelineResultProto& drive_line_result) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    drive_line_result.ParseFromString(serialized_proto);
  } else {
    drive_line_result.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar, st::planning::LargeVehicleAvoidStateProto&
                                large_vehicle_avoid_state_proto) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    large_vehicle_avoid_state_proto.ParseFromString(serialized_proto);
  } else {
    large_vehicle_avoid_state_proto.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar, st::VehicleDriveParamsProto& vehicle_drive_params) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    vehicle_drive_params.ParseFromString(serialized_proto);
  } else {
    vehicle_drive_params.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::VehicleGeometryParamsProto& vehicle_geometry_params) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    vehicle_geometry_params.ParseFromString(serialized_proto);
  } else {
    vehicle_geometry_params.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar, st::VehicleParamsProto& vehicle_params_proto) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    vehicle_params_proto.ParseFromString(serialized_proto);
  } else {
    vehicle_params_proto.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

// abseil time
template <typename Archive>
void serialize(Archive& ar, absl::Time& absl_time) {
  int64_t timestamp = absl::ToUnixSeconds(absl_time);
  ar(cereal::make_nvp("timestamp", timestamp));
  absl_time = absl::FromUnixSeconds(timestamp);
}

template <typename Archive>
void serialize(Archive& ar, st::Segment2d& segment2d) {
  ar(CEREAL_NVP(segment2d.start_));
  ar(CEREAL_NVP(segment2d.end_));
  ar(CEREAL_NVP(segment2d.unit_direction_));
  ar(CEREAL_NVP(segment2d.length_));
  ar(CEREAL_NVP(segment2d.heading_));
}

template <typename Archive>
void serialize(Archive& ar, st::Box2d& box2d) {
  ar(CEREAL_NVP(box2d.kEpsilon));
  ar(CEREAL_NVP(box2d.center_));
  ar(CEREAL_NVP(box2d.half_length_));
  ar(CEREAL_NVP(box2d.half_width_));
  ar(CEREAL_NVP(box2d.heading_));
  ar(CEREAL_NVP(box2d.cos_heading_));
  ar(CEREAL_NVP(box2d.sin_heading_));
}

template <typename Archive>
void serialize(Archive& ar, st::ObjectStopTimeProto& object_stop_time) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    object_stop_time.ParseFromString(serialized_proto);
  } else {
    object_stop_time.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar, st::PNPInfos& pnp_infos_proto) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    pnp_infos_proto.ParseFromString(serialized_proto);
  } else {
    pnp_infos_proto.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar, st::planning::SceneOutputProto& scene_output) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    scene_output.ParseFromString(serialized_proto);
  } else {
    scene_output.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::ConstraintProto::SpeedRegionProto& speed_region) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    speed_region.ParseFromString(serialized_proto);
  } else {
    speed_region.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::ConstraintProto::StopLineProto& stop_line) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    stop_line.ParseFromString(serialized_proto);
  } else {
    stop_line.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(
    Archive& ar,
    st::planning::ConstraintProto::PathStopLineProto& path_stop_line) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    path_stop_line.ParseFromString(serialized_proto);
  } else {
    path_stop_line.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(
    Archive& ar,
    st::planning::ConstraintProto::PathSpeedRegionProto& path_speed_region) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    path_speed_region.ParseFromString(serialized_proto);
  } else {
    path_speed_region.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::ConstraintProto::AvoidLineProto& avoid_line) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    avoid_line.ParseFromString(serialized_proto);
  } else {
    avoid_line.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(
    Archive& ar,
    st::planning::ConstraintProto::SpeedProfileProto& speed_profile) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    speed_profile.ParseFromString(serialized_proto);
  } else {
    speed_profile.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::DecisionConstraintConfigProto& decision_constraint_config) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    decision_constraint_config.ParseFromString(serialized_proto);
  } else {
    decision_constraint_config.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::planning::InitializerConfig& initializer_config) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    initializer_config.ParseFromString(serialized_proto);
  } else {
    initializer_config.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(
    Archive& ar,
    st::TrajectoryOptimizerParamsProto& trajectory_optimizer_params) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    trajectory_optimizer_params.ParseFromString(serialized_proto);
  } else {
    trajectory_optimizer_params.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::MotionConstraintParamsProto& motion_constraint_params) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    motion_constraint_params.ParseFromString(serialized_proto);
  } else {
    motion_constraint_params.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::PlannerFunctionsParamsProto& planner_functions_params) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    planner_functions_params.ParseFromString(serialized_proto);
  } else {
    planner_functions_params.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(
    Archive& ar,
    st::PlannerVehicleModelParamsProto& planner_vehicle_model_params) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    planner_vehicle_model_params.ParseFromString(serialized_proto);
  } else {
    planner_vehicle_model_params.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar, st::Behavior& behavior) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    behavior.ParseFromString(serialized_proto);
  } else {
    behavior.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               st::ApolloTrajectoryPointProto& apollo_trajectory_point) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    apollo_trajectory_point.ParseFromString(serialized_proto);
  } else {
    apollo_trajectory_point.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar, st::PathPoint& path_point) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    path_point.ParseFromString(serialized_proto);
  } else {
    path_point.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar, st::AABox2d& aabox2d) {
  ar(CEREAL_NVP(aabox2d.center_));
  ar(CEREAL_NVP(aabox2d.half_length_));
  ar(CEREAL_NVP(aabox2d.half_width_));
}

template <typename Archive>
void serialize(Archive& ar, st::Polygon2d& polygon2d) {
  ar(CEREAL_NVP(polygon2d.points_));
  ar(CEREAL_NVP(polygon2d.num_points_));
  ar(CEREAL_NVP(polygon2d.line_segments_));
  ar(CEREAL_NVP(polygon2d.is_convex_));
  ar(CEREAL_NVP(polygon2d.area_));
  ar(CEREAL_NVP(polygon2d.aabox_));
}

}  // namespace cereal

namespace ad_byd::planning::math {
template <typename Archive>
void serialize(Archive& ar, LineCurve2d& line_curve) {
  ar(CEREAL_NVP(line_curve.accumulated_s_));
  ar(CEREAL_NVP(line_curve.headings_));
  ar(CEREAL_NVP(line_curve.points_));
}
}  // namespace ad_byd::planning::math

#endif