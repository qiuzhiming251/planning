

#ifndef ONBOARD_PLANNER_SPEED_INTERACTIVE_AGENT_H_
#define ONBOARD_PLANNER_SPEED_INTERACTIVE_AGENT_H_

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "absl/status/status.h"
#include "plan_common/async/thread_pool.h"
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

#define LX_EPSILON (0.00001)
#define lx_fabs(x) std::fabs(x)
#define lx_fequal(x, y) (lx_fabs((x) - (y)) < LX_EPSILON)
#define lx_fgreater(x, y) (((x) > (y)) && !lx_fequal(x, y))
#define lx_fless(x, y) (((x) < (y)) && !lx_fequal(x, y))

extern void CvtPoseGlobalToLocal(SecondOrderTrajectoryPoint *local_pose,
                                 const SecondOrderTrajectoryPoint *global_pose,
                                 const SecondOrderTrajectoryPoint *base_pose);

extern void CvtPoseLocalToGlobal(SecondOrderTrajectoryPoint *global_pose,
                                 const SecondOrderTrajectoryPoint *local_pose,
                                 const SecondOrderTrajectoryPoint *base_pose);

extern void JudgeTurnInfo(bool *is_turn_left, bool *is_turn_right,
                          bool *is_uturn,
                          const SecondOrderTrajectoryPoint *global_start,
                          const SecondOrderTrajectoryPoint *global_end);

typedef struct Frenet_Velocity {
  double s_dt;
  double l_dt;
} frenet_vel_t;

struct State {
  bool is_rectangular;
  bool is_backstep = false;
  int traj_idx{0};
  double dist{0.0}; /* dist based on init pose */
  double time_stamp{0.0};
  double target_vel{0.0};
  double velocity{0.0};
  double acc{0.0};
  double heading{0.0};
  SecondOrderTrajectoryPoint sl_center_pose; /* drive_passage frenet */
  FrenetBox frenet_box;
  frenet_vel_t sl_vel; /* drive_passage frenet */
  SecondOrderTrajectoryPoint global_center_pose;
  Polygon2d global_polygon;

  void print() const {
    printf("State:");
    printf(" ts: %.1f", time_stamp);
    printf(" vel: %.2f", velocity * 3.6);
    printf(" t_v: %.2f", target_vel * 3.6);
    printf(" acc: %.2f", acc);
    printf(" heading: %.2f", heading * 180 / M_PI);
    printf(" idx: %d", traj_idx);
    printf(" dist: %.1f", dist);
    printf(" sl_c_pos x: %.2f, y: %.2f theta: %.2f", sl_center_pose.pos().x(),
           sl_center_pose.pos().y(), sl_center_pose.theta() * 180 / M_PI);
    printf(" sl_v: s_dt: %.2f, l_dt: %.2f", sl_vel.s_dt * 3.6,
           sl_vel.l_dt * 3.6);
    printf(" sl_box: max s: %.2f, min s: %.2f, max l: %.2f, min l: %.2f",
           frenet_box.s_max, frenet_box.s_min, frenet_box.l_max,
           frenet_box.l_min);
    if (is_backstep) {
      printf(" BS");
    }
    printf("\n");
  }

  void print_polygon() const {
    printf("Poly State:");
    printf(" ts: %.2f", time_stamp);
    printf(" vel: %.2f", velocity * 3.6);
    printf(" acc: %.2f", acc);
    printf(" heading: %.2f", heading * 180 / M_PI);
    printf(" idx: %d", traj_idx);
    printf(" sl center x: %.2f, y: %.2f theta: %.2f", sl_center_pose.pos().x(),
           sl_center_pose.pos().y(), sl_center_pose.theta() * 180 / M_PI);
    printf(" sl v: s_dt: %.2f, l_dt: %.2f", sl_vel.s_dt * 3.6,
           sl_vel.l_dt * 3.6);
    printf(" sl_box: max s: %.2f, min s: %.2f, max l: %.2f, min l: %.2f",
           frenet_box.s_max, frenet_box.s_min, frenet_box.l_max,
           frenet_box.l_min);
    printf(" rads: %.1f", global_polygon.CircleRadius());
    printf(" global center x: %.2f, y: %.2f theta: %.2f",
           global_center_pose.pos().x(), global_center_pose.pos().y(),
           global_center_pose.theta() * 180 / M_PI);
    if (is_backstep) {
      printf(" BS");
    }
    printf("\n");
  }
};

class AgentParam {
 public:
  friend class InteractiveAgent;

  inline double width() const { return width_; }
  inline double length() const { return length_; }
  inline double height() const { return height_; }
  inline double wheel_base() const { return wheel_base_; }
  inline double max_acc() const { return max_acc_; }
  inline double min_acc() const { return min_acc_; }
  inline double inter_max_acc() const { return inter_max_acc_; }
  inline double inter_min_acc() const { return inter_min_acc_; }
  inline double max_vel() const { return max_vel_; }
  inline double min_safe_dist() const { return min_safe_dist_; }
  inline double back_edge_to_center() const { return back_edge_to_center_; }
  inline const VehicleGeometryParamsProto &vehicle_geom() const {
    return vehicle_geom_;
  }

  inline void SetWidth(const double &width) { width_ = width; }
  inline void SetLength(const double &length) { length_ = length; }
  inline void SetBackEdgeToCenter(const double &back_edge_to_center) {
    back_edge_to_center_ = back_edge_to_center;
  }
  inline void SetVehicleGeometryParams(
      const VehicleGeometryParamsProto &vehicle_geom) {
    vehicle_geom_ = vehicle_geom;
  }
  void SetMaxAcc(const ObjectType kind);
  void SetMinAcc(const ObjectType kind);
  void SetInterMaxAcc(const ObjectType kind);
  void SetInterMinAcc(const ObjectType kind);
  void SetMaxVel(const ObjectType kind);
  void SetMinSafeDist(const ObjectType kind);

  AgentParam();
  void ResetAgentParam();
  AgentParam(double length, double width, double height, double min_acc,
             double max_acc, double inter_min_acc, double inter_max_acc,
             double min_vel, double max_vel, double min_safe_dist,
             double back_edge_to_center,
             const VehicleGeometryParamsProto vehicle_geom);
  AgentParam(const AgentParam &param);
  AgentParam &operator=(const AgentParam &param);

  void print() const {
    printf("Params:");
    printf(" width: %.2f", width_);
    printf(" length: %.2f", length_);
    printf(" wheel_base: %.2f", wheel_base_);
    printf(" max_acc: %.2f", max_acc_);
    printf(" min_acc: %.2f", min_acc_);
    printf(" inter_max: %.2f", inter_max_acc_);
    printf(" inter_min: %.2f", inter_min_acc_);
    printf(" max_vel: %.2f", max_vel_ * 3.6);
    printf(" min_safe_dist: %.2f\n", min_safe_dist_);
    printf(" back_edge_to_center: %.2f\n", back_edge_to_center_);
  }

 private:
  double length_;        /* m */
  double width_;         /* m */
  double height_;        /* m */
  double wheel_base_;    /* m for ego: front dist to rear axle, for obs: half
                            length */
  double min_acc_;       /* m/s^2 */
  double max_acc_;       /* m/s^2 */
  double inter_min_acc_; /* m/s^2 */
  double inter_max_acc_; /* m/s^2 */
  double min_vel_;       /* m/s */
  double max_vel_;       /* m/s */
  double min_safe_dist_; /* m */
  double back_edge_to_center_; /* m */
  VehicleGeometryParamsProto vehicle_geom_;
};

enum InteractiveAgentType {
  INTERACTIVE_AGENT_EGO = 0,
  INTERACTIVE_AGENT_MOV_AGENT = 1,
  INTERACTIVE_AGENT_VIRTUAL_AGENT = 2,
  INTERACTIVE_AGENT_TYPE_NUM
};

typedef struct CollisionInfo {
  double ego_dist_to_enter_collision;
  double ego_dist_to_leave_collision;
  double obj_dist_to_enter_collision;
  double obj_dist_to_leave_collision;
} collision_info_t;

struct AgentInitInfo {
  Polygon2d init_polygon_;
  SecondOrderTrajectoryPoint init_pose_;
  collision_info_t collision_info;
};

typedef enum GameTheoryScenario {
  GAME_THEORY_SCENARIO_UNKNOWN = 0,
  GAME_THEORY_SCENARIO_CROSS_STR_STR = 1, /* ego go straight, obj go straight */
  GAME_THEORY_SCENARIO_CROSS_STR_TL = 2,  /* ego go straight, obj turn left */
  GAME_THEORY_SCENARIO_CROSS_TL_STR = 3,  /* ego turn left, obj go straight */
  GAME_THEORY_SCENARIO_CROSS_STR_TR = 4,  /* ego go straight, obj turn right */
  GAME_THEORY_SCENARIO_CROSS_TR_STR = 5,  /* ego turn right, obj go straight */
  GAME_THEORY_SCENARIO_CROSS_TL_TL = 6,   /* ego turn left, obj turn left */
  GAME_THEORY_SCENARIO_CROSS_TR_TR = 7,   /* ego turn right, obj turn right */
  GAME_THEORY_SCENARIO_CROSS_UTURN_STR = 8, /* ego uturn, obj go straight */
  GAME_THEORY_SCENARIO_CROSS_STR_UTURN = 9, /* ego go straight, obj uturn */
  GAME_THEORY_SCENARIO_MERGE_PARALLEL = 10, /* parallel merge */
  GAME_THEORY_SCENARIO_MERGE_STR_TL =
      11, /* ego go straight, obj turn left merge */
  GAME_THEORY_SCENARIO_MERGE_TL_STR =
      12, /* ego turn left, obj go straight merge */
  GAME_THEORY_SCENARIO_MERGE_STR_TR =
      13, /* ego go straight, obj turn right merge */
  GAME_THEORY_SCENARIO_MERGE_TR_STR =
      14, /* ego turn right merge, obj go straight: vertical merge */
  GAME_THEORY_SCENARIO_MERGE_TL_TL = 15, /* ego turn left, obj turn left */
  GAME_THEORY_SCENARIO_MERGE_TR_TR = 16, /* ego turn right, obj turn right */
  GAME_THEORY_SCENARIO_MERGE_UTURN_STR =
      17, /* ego uturn merge, obj go straight */
  GAME_THEORY_SCENARIO_MERGE_STR_UTURN =
      18, /* ego go straight, obj uturn merge */
  GAME_THEORY_SCENARIO_MERGE_LANE_CHANGE = 19, /* ego lane change */
  GAME_THEORY_SCENARIO_ONCOMMING = 20,         /* oncomming scene */
  GAME_THEORY_SCENARIO_FOLLOW = 21,            /* follow scene */
  GAME_THEORY_SCENARIO_NUM,
} game_theory_scenario_t;

class InteractiveAgent {
 public:
  InteractiveAgent();
  void ResetInteractiveAgent();
  inline void SetAgentParam(const AgentParam &param) { param_ = param; }
  inline void SetAgentID(std::string id) { id_ = std::move(id); }
  inline void SetAgentType(const InteractiveAgentType type) { type_ = type; }
  void SetMovAgentParams(const SpacetimeObjectTrajectory *st_traj);
  void SetEgoParams(const VehicleGeometryParamsProto &vehicle_geom);

  inline void SetDrivePassage(const DrivePassage *drive_passage) {
    drive_passage_ = drive_passage;
  }

  inline void SetGameTheoryScenario(
      const game_theory_scenario_t game_theory_scenario) {
    game_theory_scenario_ = game_theory_scenario;
  }

  void SetEgoInitState(const VehicleGeometryParamsProto &vehicle_geom,
                       const DiscretizedPath &path, double ego_init_vel,
                       double ego_init_acc,
                       const std::vector<VehicleShapeBasePtr> &av_shapes,
                       const DrivePassage *drive_passage, const bool is_debug);

  void TransformGlobalToSlCenterPose(
      SecondOrderTrajectoryPoint *sl_center_pose,
      const SecondOrderTrajectoryPoint &global_center_pose,
      const DrivePassage *drive_passage, const bool is_debug);

  void SetAgentInitState(const SpacetimeObjectTrajectory *obstacle,
                         const DrivePassage *drive_passage,
                         StBoundaryWithDecision *st_boundary_decision,
                         const bool ego_turn_left, const bool ego_turn_right,
                         const bool ego_uturn, const bool is_debug);

  void SetInitInfo(const double init_vel, const double init_acc,
                   const double init_theta, const Polygon2d init_polygon,
                   const Vec2d pos);
  void GetLowerUpperIdxAccordingToDist(int *lower_idx, int *upper_idx,
                                       const double dist, const int start_idx);
  void GetEgoPolygonAccordingToDist(
      Polygon2d *ego_polygon, SecondOrderTrajectoryPoint *global_center_pose,
      int *traj_point_idx, const double dist,
      /* default 0 */ int start_idx);

  void SetStBoundaryWithDecision(StBoundaryWithDecision *st_boundary_decision);

  void GetEgoStateInfoAccordingToDist(
      FrenetBox *frenet_box, Polygon2d *obj_polygon,
      SecondOrderTrajectoryPoint *global_center_pose, int *traj_point_idx,
      const double dist,
      /* default 0 */ int start_idx);

  void CvtGlobalPoseToSlCenterPose(
      SecondOrderTrajectoryPoint *sl_center_pose,
      const SecondOrderTrajectoryPoint *global_center_pose,
      const bool is_debug);

  void GetAgentPolygonAccordingToDist(
      Polygon2d *obj_polygon, SecondOrderTrajectoryPoint *global_center_pose,
      int *traj_point_idx, const double dist,
      /* default 0 */ int start_idx);
  void GetAgentStateInfoAccordingToDist(
      FrenetBox *frenet_box, Polygon2d *ego_polygon,
      SecondOrderTrajectoryPoint *global_center_pose, int *traj_point_idx,
      const double dist,
      /* default 0 */ int start_idx);

  void UpdateGameTheoryScenario(const bool ego_turn_left,
                                const bool ego_turn_right,
                                const bool ego_uturn);

  void UpdateCollisionInfo();

  inline const Polygon2d &init_polygon() const {
    return init_info_.init_polygon_;
  }

  inline const std::string &id() const { return id_; }
  inline const AgentParam *param() const { return &param_; }
  inline const InteractiveAgentType type() const { return type_; }
  inline const double init_vel() const { return init_info_.init_pose_.v(); }
  inline const double init_theta() const {
    return init_info_.init_pose_.theta();
  }
  inline const double init_acc() const { return init_info_.init_pose_.a(); }
  inline const std::vector<State> &state() const { return state_; }
  inline std::vector<State> *mutable_state() { return &state_; }
  inline const SpacetimeObjectTrajectory *obstacle_ptr() const {
    return obstacle_;
  }

  inline const DrivePassage *drive_passage() const { return drive_passage_; }
  inline const collision_info_t *collision_info() const {
    return &init_info_.collision_info;
  }
  inline const game_theory_scenario_t &game_theory_scenario() const {
    return game_theory_scenario_;
  }
  inline const StBoundaryWithDecision *st_boundary_decision() const {
    return st_boundary_decision_;
  }

  ~InteractiveAgent() = default;

 private:
  std::string id_; /* obs perception id*/
  AgentParam param_;
  InteractiveAgentType type_;
  std::vector<State> state_;
  AgentInitInfo init_info_;
  const SpacetimeObjectTrajectory *obstacle_; /* only valid for mov obj */
  StBoundaryWithDecision *st_boundary_decision_;
  bool is_valid_;
  const DrivePassage *drive_passage_;
  game_theory_scenario_t game_theory_scenario_;
};

}  // namespace planning
}  // namespace st
#endif  // ONBOARD_PLANNER_SPEED_INTERACTIVE_SPEED_DECISION_H_
