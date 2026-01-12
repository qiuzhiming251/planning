

#ifndef ONBOARD_PLANNER_INITIALIZER_DP_COST_FEATURE_H_
#define ONBOARD_PLANNER_INITIALIZER_DP_COST_FEATURE_H_

#include <algorithm>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include "absl/types/span.h"
#include "plan_common/path_sl_boundary.h"
#include "decider/initializer/collision_checker.h"
#include "decider/initializer/cost_feature.h"
#include "decider/initializer/ref_speed_table.h"
#include "plan_common/math/piecewise_linear_function.h"
//#include "ml/captain_net/captain_net.h"
//#include "ml/captain_net/utils/cost_utils.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/drive_passage.h"

namespace st::planning {

constexpr double kStopConstraintMargin = 0.25;  // m.

class DpAccelerationFeatureCost : public FeatureCost {
 public:
  explicit DpAccelerationFeatureCost(
      const MotionConstraintParamsProto& motion_constraint_params,
      const InitializerSceneType init_scene_type)
      : FeatureCost("dp_acceleration"),
        init_scene_type_(init_scene_type),
        max_accel_constraint_(motion_constraint_params.max_acceleration()),
        max_decel_constraint_(motion_constraint_params.max_deceleration()) {}
  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  const InitializerSceneType init_scene_type_;
  double max_accel_constraint_;
  double max_decel_constraint_;
};

class DpLaneBoundaryFeatureCost : public FeatureCost {
 public:
  explicit DpLaneBoundaryFeatureCost(const PathSlBoundary* path_sl,
                                     const InitializerSceneType init_scene_type,
                                     double sdc_half_width)
      : FeatureCost("dp_lane_boundary"),
        path_sl_(path_sl),
        init_scene_type_(init_scene_type),
        sdc_half_width_(sdc_half_width) {}

  void ComputeCost(const MotionEdgeInfo&,
                   absl::Span<double> cost) const override;

 private:
  // Not owned.
  const PathSlBoundary* path_sl_;
  const InitializerSceneType init_scene_type_;
  double sdc_half_width_;
};

class DpCurvatureFeatureCost : public FeatureCost {
 public:
  DpCurvatureFeatureCost() : FeatureCost("dp_curvature") {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;
};

class DpLateralAccelerationFeatureCost : public FeatureCost {
 public:
  explicit DpLateralAccelerationFeatureCost(
      const InitializerSceneType init_scene_type, bool is_lane_change)
      : FeatureCost("dp_lateral_acceleration"),
        init_scene_type_(init_scene_type),
        is_lane_change_(is_lane_change) {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  const InitializerSceneType init_scene_type_;
  bool is_lane_change_;
};

class DpStopConstraintFeatureCost : public FeatureCost {
 public:
  explicit DpStopConstraintFeatureCost(const std::vector<double>& stop_s)
      : FeatureCost("dp_stop_constraint"),
        nearest_stop_s_(stop_s.empty()
                            ? std::numeric_limits<double>::max()
                            : *std::min_element(stop_s.begin(), stop_s.end()) +
                                  kStopConstraintMargin) {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  double nearest_stop_s_;
};

class DpRefSpeedFeatureCost : public FeatureCost {
 public:
  explicit DpRefSpeedFeatureCost(const RefSpeedTable* ref_speed_table,
                                 const InitializerSceneType init_scene_type)
      : FeatureCost("dp_ref_speed"),
        ref_speed_table_(ref_speed_table),
        init_scene_type_(init_scene_type) {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  const RefSpeedTable* ref_speed_table_;
  const InitializerSceneType init_scene_type_;
};

class DpDynamicCollisionFeatureCost : public FeatureCost {
 public:
  explicit DpDynamicCollisionFeatureCost(
      const CollisionChecker* collision_checker);

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

  // Provide & Update a set of ignorable trajectory ids.
  IgnoreTrajMap ComputeInteractiveCost(const MotionEdgeInfo& edge_info,
                                       const IgnoreTrajMap& ignored_trajs,
                                       absl::Span<double> cost) const;

 private:
  // Not owned.
  const CollisionChecker* cc_;
};

class DpLeadingObjectFeatureCost : public FeatureCost {
 public:
  // Construct the DpLeadingObjectFeatureCost without specifying the leading
  // objects.
  explicit DpLeadingObjectFeatureCost(
      const DrivePassage& drive_passage,
      const SpacetimeTrajectoryManager& st_traj_mgr,
      const std::vector<std::string>& leading_trajs, double ego_front_to_ra);

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

 private:
  double ego_front_to_ra_;
  PiecewiseLinearFunction<double> max_s_t_;
};

// class DpRefTrajectoryFeatureCost : public FeatureCost {
//  public:
//   explicit DpRefTrajectoryFeatureCost(
//       const ml::captain_net::CaptainNetOutput* captain_net_output)
//       : FeatureCost("dp_ref_traj"), captain_net_output_(captain_net_output) {
//     if (!captain_net_output_->traj_points.empty()) {
//       ref_traj_weight_ =
//           std::vector<double>(captain_net_output_->traj_points.size(), 1.0);
//       // TODO: Move gamma to planner default params after
//       // testing.
//       constexpr double kTimeDecayRate = 0.99;
//       ml::CalculateTimeDecayWeight(kTimeDecayRate, &ref_traj_weight_);
//       reg_factor_ = std::max(std::accumulate(ref_traj_weight_.begin(),
//                                              ref_traj_weight_.end(), 0.0),
//                              1e-6);
//     }
//   }

//   void ComputeCost(const MotionEdgeInfo& edge_info,
//                    absl::Span<double> cost) const override;

//  private:
//   const ml::captain_net::CaptainNetOutput* captain_net_output_;
//   std::vector<double> ref_traj_weight_;
//   double reg_factor_ = 1.0;
// };

class DpFinalProgressFeatureCost : public FeatureCost {
 public:
  explicit DpFinalProgressFeatureCost(
      const PathSlBoundary* path_sl, const InitializerSceneType init_scene_type,
      double start_accumulated_s, double max_accumulated_s)
      : FeatureCost("dp_final_progress"),
        path_sl_(path_sl),
        init_scene_type_(init_scene_type),
        start_accumulated_s_(start_accumulated_s),
        max_accumulated_s_(max_accumulated_s),
        max_rel_s_(max_accumulated_s - start_accumulated_s) {}

  void ComputeCost(const MotionEdgeInfo& edge_info,
                   absl::Span<double> cost) const override;

  // void ComputeCostForAStar(const MotionEdgeInfo& edge_info,
  //                          absl::Span<double> cost) const;

 private:
  const PathSlBoundary* path_sl_;
  const InitializerSceneType init_scene_type_;
  double start_accumulated_s_;
  double max_accumulated_s_;
  double max_rel_s_;
};

}  // namespace st::planning
#endif  // ONBOARD_PLANNER_INITIALIZER_DP_COST_FEATURE_H_
