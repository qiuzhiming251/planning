#pragma once
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/math/discretized_path.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

namespace st::planning {
enum class ObjBehavior {
  OPPO_LEFT_TURN = 1,
  OPPO_GO_STRAIGHT = 2,
  SYNC_LEFT_TURN = 3,
};
enum class ObjType {
  NORMAL_VEHICLE = 1,
  LARGE_VEHICLE = 2,
  CYCLIST = 3,
  PEDESTRIAN = 4,
};
enum class ObjDecisionType {
  YIELD = 1,
  PASS = 2,
};

enum class ObjectDecisionType {
  OBJECT_DECISION_TYPE_NONE = 1,
  OBJECT_DECISION_TYPE_LEFT_NUDGE = 2,
  OBJECT_DECISION_TYPE_RIGHT_NUDGE = 3,
};

struct RiskFieldCollisionArea {
  PathPoint obj_yield_point;
  PathPoint obj_pass_point;
  PathPoint ego_yield_point;
  PathPoint ego_pass_point;

  std::string DebugInfo() const {
    std::string debug = "collision area:\n";
    debug += "obj_yield_point.x: " + std::to_string(obj_yield_point.x()) +
             " , y: " + std::to_string(obj_yield_point.y()) + "\n";
    debug += "obj_pass_point.x: " + std::to_string(obj_pass_point.x()) +
             " , y: " + std::to_string(obj_pass_point.y()) + "\n";
    debug += "ego_yield_point.x: " + std::to_string(ego_yield_point.x()) +
             " , y: " + std::to_string(ego_yield_point.y()) + "\n";
    debug += "ego_pass_point.x: " + std::to_string(ego_pass_point.x()) +
             " , y: " + std::to_string(ego_pass_point.y()) + "\n";
    return debug;
  }
};
struct RiskFieldKeyobj {
  std::string id;
  std::string traj_id;
  ObjDecisionType obj_decision_type;
  std::optional<double> agent_hwt;
  std::optional<RiskFieldCollisionArea> collision_area;
  double ego_hwt;
  double speed;
  ObjType obj_type;
  double collision_dist;
  ObjBehavior obj_behavior;
  const SpacetimeObjectTrajectory* object_ptr;
  ObjectDecisionType obj_lat_decision;
};

}  // namespace st::planning