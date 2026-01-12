

#ifndef ONBOARD_PLANNER_SPEED_GAME_THEORY_COST_H_
#define ONBOARD_PLANNER_SPEED_GAME_THEORY_COST_H_

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "absl/status/status.h"
#include "plan_common/async/thread_pool.h"
#include "game_theory_common.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/speed/st_speed/speed_limit_provider.h"
#include "plan_common/speed/st_speed/speed_vector.h"
#include "planner/speed_optimizer/st_graph.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st {
namespace planning {

typedef enum LongitudinalDirection {
  LD_FRONT = 0,
  LD_REAR = 1,
  LD_NUM
} lon_dir_t;

typedef enum LateralDirection {
  LAT_LEFT = 0,
  LAT_RIGHT = 1,
  LAT_NUM,
} lat_dir_t;

typedef enum LongitudinalViolateType {
  LON_VIO_LEGAL = 0,
  LON_VIO_TOOFAST = 1,
  LON_VIO_TOOSLOW = 2,
  LON_VIO_NUM,
} lon_violate_type_t;

typedef struct PolygonCollisionInfo {
  bool is_front_collision;
  bool is_left_collision;
  bool is_rear_collision;
  bool is_right_collision;
  bool is_valid;
} polygon_collision_info_t;

typedef struct RssParams {
  double ego_response_time;
  double ego_longitudinal_acc_max;
  double ego_longitudinal_brake_min;
  double ego_longitudinal_brake_max;
  double ego_lateral_acc_max;
  double ego_lateral_brake_min;
  double ego_lateral_brake_max;
  double ego_lateral_miu;

  double obj_response_time;
  double obj_longitudinal_acc_max;
  double obj_longitudinal_brake_min;
  double obj_longitudinal_brake_max;
  double obj_lateral_acc_max;
  double obj_lateral_brake_min;
  double obj_lateral_brake_max;
  double obj_lateral_miu;

  double over_speed_linear_coeff;
  double over_speed_power_coeff;
  double lack_speed_linear_coeff;
  double lack_speed_power_coeff;
} rss_params_t;

typedef struct GameTheoryCostParams {
  rss_params_t rss_params;
  double traffic_rule_cost_overtake_linear_coeff;
  double traffic_rule_cost_overtake_power_coeff;
  double traffic_rule_cost_follow_linear_coeff;
  double traffic_rule_cost_follow_power_coeff;
  double max_step_cost_value;
} game_theory_cost_params_t;

class GameTheoryCost {
 public:
  GameTheoryCost();
  ~GameTheoryCost();

  virtual void GameTheoryCostUpdate(
      gt_ego_state_t *ego_state, gt_agent_state_t *obj_state,
      const double ego_length, const double ego_width, const double obj_length,
      const double obj_width, const double ego_dist_to_enter_intersection,
      const double ego_dist_to_leave_intersection,
      const double obj_dist_to_enter_intersection,
      const double obj_dist_to_leave_intersection,
      const StBoundaryProto::DecisionType decision_type,
      const double ego_init_acc, const double obj_init_acc,
      const double ego_sampling_acc, const double obj_sampling_acc,
      const int step, const bool is_debug_cost,
      const game_theory_scenario_t game_theory_scenario) = 0;
  virtual void EgoSafetyCost(gt_ego_state_t *ego_state,
                             const gt_agent_state_t *obj_state,
                             const double ego_width, const double obj_width,
                             const bool is_debug_cost) = 0;
  virtual void EgoTrafficRuleCost(
      gt_ego_state_t *ego_state, const gt_agent_state_t *obj_state,
      const StBoundaryProto::DecisionType decision_type,
      const bool is_ego_responsibility, const bool is_collision,
      const bool is_debug_cost, const double ego_dist_to_enter_intersection,
      const double obj_dist_to_enter_intersection) = 0;
  virtual void EgoAccelerateCost(
      gt_ego_state_t *ego_state, const gt_agent_state_t *obj_state,
      const StBoundaryProto::DecisionType decision_type,
      const double ego_init_acc, const double ego_sampling_acc,
      const bool is_debug_cost, const double obj_init_acc,
      const double ego_dist_to_enter_intersection,
      const double obj_dist_to_enter_intersection) = 0;
  virtual void ObjSafetyCost(gt_agent_state_t *obj_state,
                             const gt_ego_state_t *ego_state,
                             const double ego_width, const double obj_width,
                             const bool is_debug_cost,
                             const double obj_dist_to_enter_intersection) = 0;
  virtual void ObjTrafficRuleCost(
      gt_agent_state_t *obj_state, const gt_ego_state_t *ego_state,
      const StBoundaryProto::DecisionType decision_type,
      const bool is_obj_responsibility, const bool is_collision,
      const bool is_debug_cost,
      const double obj_dist_to_enter_intersection) = 0;
  virtual void ObjAccelerateCost(
      gt_agent_state_t *obj_state, const double obj_init_acc,
      const double obj_sampling_acc, const bool is_debug_cost,
      const double ego_dist_to_enter_intersection,
      const double obj_dist_to_enter_intersection) = 0;

  lon_dir_t CalAgentsRelaLonDir(const FrenetBox &frenet_box_a,
                                const FrenetBox &frenet_box_b,
                                const double ego_theta, const double obj_theta);

  lat_dir_t CalAgentsRelaLatDir(const FrenetBox &frenet_box_a,
                                const FrenetBox &frenet_box_b);

  void CalRssLateralSafeDist(
      double *safe_distance, const double ego_vel, const double agent_vel,
      const lat_dir_t lat_direct, const double ego_lateral_brake_max,
      const double ego_lateral_brake_min, const double ego_lateral_acc_max,
      const double agent_lateral_brake_max,
      const double agent_lateral_brake_min, const double agent_lateral_acc_max,
      const double response_time, const double lateral_miu);
  void CalRssLonSafeVelBoundary(double *ego_vel_high, double *ego_vel_low,
                                const double lon_relative_distance,
                                const lon_dir_t lon_direct,
                                const double agent_vel,
                                const double response_time,
                                const double longitudinal_acc_max,
                                const double longitudinal_brake_min,
                                const double longitudinal_brake_max);

  void GameTheoryRssSafeCheckEgo(bool *is_safe,
                                 lon_violate_type_t *lon_violate_type,
                                 double *rss_vel_low, double *rss_vel_high,
                                 const gt_ego_state_t *ego_state,
                                 const gt_agent_state_t *obj_state,
                                 const double ego_width, const double obj_width,
                                 const bool is_debug_cost);

  void GameTheoryRssSafeCheckObj(bool *is_safe,
                                 lon_violate_type_t *lon_violate_type,
                                 double *rss_vel_low, double *rss_vel_high,
                                 const gt_ego_state_t *ego_state,
                                 const gt_agent_state_t *obj_state,
                                 const double ego_width, const double obj_width,
                                 const bool is_debug_cost);

  void SetRssStaticParams();

  void CheckTwoPolygonCollision(polygon_collision_info_t *collision_info,
                                bool *is_collision,
                                const Polygon2d &base_polygon,
                                const Box2d &target_box,
                                const double lat_buffer,
                                const double lon_buffer);

  void GetTwoPolygonCollisionInfo(
      bool *is_collision, polygon_collision_info_t *ego_polygon_collision_info,
      polygon_collision_info_t *obj_polygon_collision_info,
      const Polygon2d &ego_polygon, const Polygon2d &obj_polygon,
      const double ego_length, const double ego_width, const double obj_length,
      const double obj_width, const double ego_theta, const double obj_theta,
      const double lat_buffer, const double lon_buffer);
  void judge_ego_obj_responsibility(
      bool *is_ego_responsibility, bool *is_obj_responsibility,
      const bool is_collision,
      const game_theory_scenario_t game_theory_scenario,
      const polygon_collision_info_t &ego_polygon_collision_info,
      const polygon_collision_info_t &obj_polygon_collision_info,
      const SecondOrderTrajectoryPoint &ego_center_pose,
      const SecondOrderTrajectoryPoint &obj_center_pose,
      const double half_ego_length, const bool is_debug_cost);
  game_theory_cost_params_t static_params;
};

class TurnLeftCrossCost : public GameTheoryCost {
 public:
  TurnLeftCrossCost();
  ~TurnLeftCrossCost();

  void GameTheoryCostUpdate(
      gt_ego_state_t *ego_state, gt_agent_state_t *obj_state,
      const double ego_length, const double ego_width, const double obj_length,
      const double obj_width, const double ego_dist_to_enter_intersection,
      const double ego_dist_to_leave_intersection,
      const double obj_dist_to_enter_intersection,
      const double obj_dist_to_leave_intersection,
      const StBoundaryProto::DecisionType decision_type,
      const double ego_init_acc, const double obj_init_acc,
      const double ego_sampling_acc, const double obj_sampling_acc,
      const int step, const bool is_debug_cost,
      const game_theory_scenario_t game_theory_scenario) override;

  void CalcTimeToCollision(const gt_ego_state_t *ego_state,
                           const gt_agent_state_t *obj_state,
                           const double ego_dist_to_enter_intersection,
                           const double ego_dist_to_leave_intersection,
                           const double obj_dist_to_enter_intersection,
                           const double obj_dist_to_leave_intersection,
                           const bool is_debug_cost);

  void EgoSafetyCost(gt_ego_state_t *ego_state,
                     const gt_agent_state_t *obj_state, const double ego_width,
                     const double obj_width, const bool is_debug_cost) override;
  void EgoTrafficRuleCost(gt_ego_state_t *ego_state,
                          const gt_agent_state_t *obj_state,
                          const StBoundaryProto::DecisionType decision_type,
                          const bool is_ego_responsibility,
                          const bool is_collision, const bool is_debug_cost,
                          const double ego_dist_to_enter_intersection,
                          const double obj_dist_to_enter_intersection) override;
  void EgoAccelerateCost(gt_ego_state_t *ego_state,
                         const gt_agent_state_t *obj_state,
                         const StBoundaryProto::DecisionType decision_type,
                         const double ego_init_acc,
                         const double ego_sampling_acc,
                         const bool is_debug_cost, const double obj_init_acc,
                         const double ego_dist_to_enter_intersection,
                         const double obj_dist_to_enter_intersection) override;
  void ObjSafetyCost(gt_agent_state_t *obj_state,
                     const gt_ego_state_t *ego_state, const double ego_width,
                     const double obj_width, const bool is_debug_cost,
                     const double obj_dist_to_enter_intersection) override;
  void ObjTrafficRuleCost(gt_agent_state_t *obj_state,
                          const gt_ego_state_t *ego_state,
                          const StBoundaryProto::DecisionType decision_type,
                          const bool is_obj_responsibility,
                          const bool is_collision, const bool is_debug_cost,
                          const double obj_dist_to_enter_intersection) override;
  void ObjAccelerateCost(gt_agent_state_t *obj_state, const double obj_init_acc,
                         const double obj_sampling_acc,
                         const bool is_debug_cost,
                         const double ego_dist_to_enter_intersection,
                         const double obj_dist_to_enter_intersection) override;

  double ego_time_to_intersection;
  double ego_time_to_leave_intersection;
  double obj_time_to_intersection;
  double obj_time_to_leave_intersection;
};

}  // namespace planning
}  // namespace st
#endif  // ONBOARD_PLANNER_SPEED_INTERACTIVE_SPEED_DECISION_H_
