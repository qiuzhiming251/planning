#pragma once
#include <vector>
#include "plan_common/math/vec.h"
#include <string>
#include "drf_keyobj_decider/keyobj_decider.h"
#include "vehicle_state.h"

namespace st::planning {
struct AgentPoseInfo {
  double heading_diff = 0.0;
  double cos_heading_diff = 0.0;
  double sin_heading_diff = 0.0;
  double agent_x = 0.0;
  double agent_y = 0.0;
  double agent_length = 0.0;
  double agent_width = 0.0;
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
  std::vector<Vec2d> agent_box_points;

  // Two edges of upper edge and lower edge in av's view.
  Vec2d upper_conflict_edge_head_point;
  Vec2d upper_conflict_edge_tail_point;
  Vec2d lower_conflict_edge_head_point;
  Vec2d lower_conflict_edge_tail_point;

  // Quantities involving longitudinal distances:
  double distance_to_conflict_zone = 0.0;
  double av_distance_to_conflict_zone = 0.0;
  double av_distance_to_leave_conflict_zone = 0.0;
  double agent_distance_to_conflict_zone = 0.0;
  double agent_distance_to_leave_conflict_zone = 0.0;

  // For the case of parallel position, is_parallel is true,
  // we only have two parallel conflict edges
  bool is_parallel = false;
  bool is_vertical = false;
  Vec2d parallel_left_edge_head_point;
  Vec2d parallel_left_edge_tail_point;
  Vec2d parallel_right_edge_head_point;
  Vec2d parallel_right_edge_tail_point;
};

struct EnvInfo {
  // Vector (start_point_start, start_point_end), (go_point_start, go_point_end)
  // is unit vector
  Vec2d go_point_start;
  Vec2d go_point_end;
  Vec2d start_point_start;
  Vec2d start_point_end;
  double go_point_heading = 0.0;
  double start_point_heading = 0.0;
  double cos_go_point_heading = 0.0;
  double sin_go_point_heading = 0.0;
  double sin_start_point_heading = 0.0;
  double cos_start_point_heading = 0.0;
  bool start_point_is_valid = false;
};

struct RoadInfo {
  Vec2d go_point;
  double go_point_heading;
  Vec2d start_point;
  double start_point_heading;
  bool start_point_is_valid;
};

struct Risk {
  // The value of risk
  double risk = 0.0;

  // The partial derivate of risk w.r.t. x
  double risk_x = 0.0;
  // The partial derivate of risk w.r.t. v
  double risk_v = 0.0;
  // The partial derivate of risk w.r.t. theta
  double risk_theta = 0.0;
  // The partial derivate of risk w.r.t. t
  double risk_t = 0.0;

  // virtual time to become non-risk
  double virtual_time = 0.0;

  double shrinkage_coeff = 1.0;

  double lat_distance_T;

  std::string DebugString() const {
    return "risk: " + std::to_string(risk) +
           " risk_x: " + std::to_string(risk_x) +
           " risk_v: " + std::to_string(risk_v) +
           " risk_theta: " + std::to_string(risk_theta) +
           " risk_t: " + std::to_string(risk_t) +
           " virtual_time: " + std::to_string(virtual_time) +
           " shrinkage_coeff: " + std::to_string(shrinkage_coeff);
  }
};

namespace RiskField {
std::optional<Vec2d> GetLineInsertion(const Vec2d& A, const Vec2d& B,
                                      const Vec2d& C, const Vec2d& D);
AgentPoseInfo CalcAgentPoseInfo(const Box2d& av_box, const Box2d& agent_box);

EnvInfo CalEnvInfo(const RoadInfo& road_info);

Risk CalGoPointEnvRisk(const Box2d& av_box, double av_speed,
                       const EnvInfo& enm_info_origin,
                       const ObjectDecisionType& lateral_decision,
                       double virtual_time, std::string* debug);

Risk CalStartPointEnvRisk(const Box2d& av_box, double av_speed,
                          const EnvInfo& enm_info,
                          const ObjectDecisionType& lateral_decision,
                          double virtual_time, std::string* debug);

Risk CalcAgentLongitudinalRiskForCrossYield(
    const Box2d& av_box, double av_speed, const AgentPoseInfo& agent_pose_info,
    double agent_speed, std::string* debug);

Risk CalcAgentLateralRisk(const Box2d& av_box, double av_speed,
                          const AgentPoseInfo& agent_pose_info,
                          double agent_speed,
                          const ObjectDecisionType& lateral_decision,
                          const ObjBehavior& obj_behavior,
                          bool use_lat_virtual_time, std::string* debug);

VehicleState CalcNextVehicleStateWithConstKappa(
    const VehicleState& vehicle_state, double dt);

AgentPoseInfo CalcLookForwardPoseInfo(const VehicleState& vehicle_state,
                                      const Box2d& av_box,
                                      const Box2d& agent_box,
                                      double cur_simu_time,
                                      double go_point_heading);

Risk CalcAgentLookForwardLongitudinalRiskForCrossYield(
    const Box2d& av_box, double av_speed, double lookforward_kappa,
    const AgentPoseInfo& agent_pose_info, double agent_speed,
    std::string* debug);
}  // namespace RiskField

}  // namespace st::planning
