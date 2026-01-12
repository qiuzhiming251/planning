#pragma once

#include <string>
#include <vector>

#include "plan_common/speed/st_speed/speed_vector.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/st_boundary.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/path_approx_overlap.h"
#include "plan_common/trajectory.h"
#include "plan_common/trajectory_point.h"

namespace st::planning {
struct SpeedGamingParams  // speedgaming全局参数, TODO by zx:从proto参数文件读取
{
  double planning_horizon = 8.0;         // unit:s
  double time_step = 0.2;                // unit:s
  double min_vel = 0.0;                  // unit:m/s
  double max_vel = 33.0;                 // unit:m/s
  double min_acc = -5.0;                 // unit:m/s^2
  double max_acc = 1.5;                  // unit:m/s^2
  double min_jerk = -5.0;                // unit:m/s^3
  double max_jerk = 2.0;                 // unit:m/s^3
  double max_friction_circle_acc = 2.0;  // unit:m/s^2
};
struct GamingConflictZone {
  double ego_cutin_time = 0;
  double ego_cutin_s = 0;
  double ego_cutout_time = 0;
  double ego_cutout_s = 0;
  double ego_cross_angle = 0;
  double agent_cutin_time = 0;
  double agent_cutin_s = 0;
  double agent_cutout_time = 0;
  double agent_cutout_s = 0;
  double agent_cross_angle = 0;
  double risk_field_lat_ratio = 0;
  bool is_path_conflict = false;
  bool is_risk_field_conflict = false;
};

struct RiskObjInfo {
  GamingConflictZone risk_field_zoneinfo;  //存储风险场障碍物信息
  const PlannerObject *planner_obj;
};
enum InteractionType {
  KNone = 0,
  kCross = 1,
  kStraightMerge = 2,
  kTurnMerge = 3,
  kStaticOccupy = 4,
};

struct IdmSimulateState {
  bool is_agent_state = false;
  double agent_ideal_acc_in_yield_mode = 0.0;
  double agent_ideal_acc_in_pass_mode = 0.0;
};

enum IntersectionTurnType {
  None = 0,
  Left = 1,
  Straight = 2,
  Right = 3,
  Uturn = 4,
};

struct GamingConflictZoneInfo {
  std::string obj_traj_id = "";
  bool is_pre_gaming = false;
  bool is_follow = false;
  bool is_ignore = false;
  bool is_lc_pass = false;
  InteractionType interaction_type;
  GamingConflictZone conflict_zone_in_ego_view;
  GamingConflictZone conflict_zone_in_agent_view;

#ifdef ALTERNATIVE_GAMING_UNIT_TEST
  std::vector<std::pair<double, double>> ego_left_risk_edge;
  std::vector<std::pair<double, double>> ego_right_risk_edge;
  std::vector<std::pair<double, double>> agent_left_risk_edge;
  std::vector<std::pair<double, double>> agent_right_risk_edge;
#endif
};

struct SpeedIdmState {
  SpeedIdmState() = default;
  SpeedIdmState(double _ego_s, double _ego_v, double _ego_a, double _ego_j,
                double _agent_s, double _agent_v, double _agent_a,
                double _agent_j)
      : ego_s(_ego_s),
        ego_v(_ego_v),
        ego_a(_ego_a),
        ego_j(_ego_j),
        agent_s(_agent_s),
        agent_v(_agent_v),
        agent_a(_agent_a),
        agent_j(_agent_j) {}
  double ego_s;
  double ego_v;
  double ego_a;
  double ego_j;
  double agent_s;
  double agent_v;
  double agent_a;
  double agent_j;
};

struct SpeedIdmUpdateParams {
  double alpha = 0.3;  // 相对速度系数
  double beta = 0.3;   // 理想距离差P控制系数
  double d0 = 2.0;     // 最小驱车距离
  double k0 = 1.0;     // 驱车时距
};

struct InfoAfterSimulation {
  SpeedIdmUpdateParams idm_param;
  std::vector<double> target_acc;
  std::vector<double> fast_decel_to_accel_pass_traj;
  std::vector<double> fast_decel_to_accel_yield_traj;
  double ideal_idm_dist_ego_pass = 0.0;
  double agent_dist_to_conflict_time_ego_cutout = 0.0;
  double agent_dist_to_conflict_time_ego_cutin = 0.0;
  void Clear() {
    idm_param = {};
    target_acc.clear();
    fast_decel_to_accel_pass_traj.clear();
    fast_decel_to_accel_yield_traj.clear();
    agent_dist_to_conflict_time_ego_cutout = 0.0;
    agent_dist_to_conflict_time_ego_cutin = 0.0;
    ideal_idm_dist_ego_pass = 0.0;
  }
};

namespace SpeedIdmCommon {
double CalcYieldIdmAcc(const SpeedIdmState &cur_state,
                       const SpeedIdmUpdateParams &idm_params);

double CalcPassIdmAcc(const SpeedIdmState &cur_state,
                      SpeedIdmUpdateParams &idm_params, double ego_cutin_angle,
                      double interactive_time, double interactive_dist);

double CalMaxFrictionCircleBoundTargetAcc(double max_friction_acc, double cur_v,
                                          double cur_kappa);

double CalcSpeedLimitAcc(
    double cur_s, double cur_vel, double virtual_time,
    const PiecewiseLinearFunction<double> &speed_limit_plf);

double CalcSpeedProfileYieldIdmAcc(const SpeedIdmState &cur_state,
                                   const SpeedIdmUpdateParams &idm_params);
}  // namespace SpeedIdmCommon
}  // namespace st::planning