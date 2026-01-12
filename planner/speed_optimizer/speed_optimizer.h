

#ifndef ST_PLANNING_SPEED_SPEED_OPTIMIZER
#define ST_PLANNING_SPEED_SPEED_OPTIMIZER

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "plan_common/math/piecewise_jerk_qp_solver/piecewise_jerk_qp_solver.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/speed/st_speed/speed_vector.h"
#include "planner/speed_optimizer/speed_bound.h"
#include "planner/speed_optimizer/speed_optimizer_constraint_manager.h"
#include "planner/speed_optimizer/speed_optimizer_object.h"
#include "planner/speed_optimizer/speed_optimizer_object_manager.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"

namespace st::planning {

class SpeedOptimizer {
 public:
  SpeedOptimizer(std::string_view base_name, double init_v, double init_a,
                 const MotionConstraintParamsProto* motion_constraint_params,
                 const SpeedFinderParamsProto* speed_finder_params,
                 double path_length, double default_speed_limit,
                 double delta_t);

  absl::Status Optimize(
      const SpeedOptimizerObjectManager& opt_obj_mgr,
      const SpeedBoundMapType& speed_bound_map,
      const SpeedVector& reference_speed,
      const std::vector<AccelBounds>& accel_bounds,
      const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj,
      SpeedVector* optimized_speed,
      SpeedFinderDebugProto* speed_finder_debug_proto);

 private:
  bool AddConstarints(double init_v, double init_a,
                      const SpeedOptimizerConstraintManager& constraint_mgr);

  bool AddKernel(const SpeedOptimizerConstraintManager& constraint_mgr,
                 const SpeedVector& reference_speed);

  absl::Status Solve();

  void MakeSConstraint(int knot_idx,
                       const SpeedOptimizerObjectManager& opt_obj_mgr,
                       const std::optional<double>& min_stationary_upper_bound,
                       SpeedOptimizerConstraintManager* constraint_mgr,
                       SpeedFinderDebugProto* speed_finder_debug_proto);

  void MakeMovingObjectFollowConstraint(
      int knot_idx, const ObjectOverlapState& overlap_state, std::string id,
      double follow_standstill,
      const std::optional<double>& min_stationary_upper_bound,
      double time_att_gain,
      const PiecewiseLinearFunction<double>&
          comfortable_brake_bound_violation_rel_speed_plf,
      SpeedOptimizerConstraintManager* constraint_mgr,
      SpeedFinderDebugProto* speed_finder_debug_proto);

  void MakeStationaryObjectFollowConstraint(
      int knot_idx, const ObjectOverlapState& overlap_state, std::string id,
      double time_gain, SpeedOptimizerConstraintManager* constraint_mgr,
      SpeedFinderDebugProto* speed_finder_debug_proto);

  void MakeMovingObjectLeadConstraint(
      int knot_idx, const ObjectOverlapState& overlap_state, std::string id,
      double time_att_gain, SpeedOptimizerConstraintManager* constraint_mgr,
      SpeedFinderDebugProto* speed_finder_debug_proto);

  void MakeSpeedConstraint(
      int knot_idx,
      const std::map<SpeedLimitTypeProto::Type,
                     std::vector<SpeedBoundWithInfo>>& speed_limit_map,
      SpeedOptimizerConstraintManager* constraint_mgr,
      SpeedFinderDebugProto* speed_finder_debug_proto) const;

  void MakeAccelConstraint(
      int knot_idx, const std::vector<AccelBounds>& accel_bound,
      SpeedOptimizerConstraintManager* constraint_mgr) const;

  void MakeJerkConstraint(
      const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj,
      SpeedOptimizerConstraintManager* constraint_mgr) const;

  std::optional<double> ComputeMinStationaryUpperBound(
      const SpeedOptimizerObjectManager& opt_obj_mgr);

  void FilterQpSpeedPoints(std::vector<SpeedPoint>& qp_speed_points);

 private:
  std::string base_name_;
  double init_v_;
  double init_a_;

  const MotionConstraintParamsProto* motion_constraint_params_;
  const SpeedFinderParamsProto* speed_finder_params_;
  const SpeedFinderParamsProto::SpeedOptimizerParamsProto*
      speed_optimizer_params_;

  double delta_t_ = 0.0;
  int knot_num_ = 0;
  double total_time_ = 0.0;
  double allowed_max_speed_ = 0.0;
  double max_path_length_ = 0.0;
  PiecewiseLinearFunction<double> probability_gain_plf_;
  PiecewiseLinearFunction<double> accel_weight_gain_plf_;
  std::vector<double> piecewise_time_range_;
  std::vector<double> moving_obj_time_gain_;
  std::vector<double> static_obj_time_gain_;
  PiecewiseLinearFunction<double> accel_lower_bound_plf_;
  PiecewiseLinearFunction<double> ref_speed_time_gain_;
  PiecewiseLinearFunction<double> ttc_stop_weight_gain_plf_;

  SpeedVector comfortable_brake_speed_;
  SpeedVector max_brake_speed_;

  std::unique_ptr<PiecewiseJerkQpSolver> solver_;

  bool use_soft_jerk_constraint_ = false;

  std::vector<PiecewiseLinearFunction<double>> slack_coeff_plf_;
};

}  // namespace st::planning

#endif  // ST_PLANNING_SPEED_SPEED_OPTIMIZER
