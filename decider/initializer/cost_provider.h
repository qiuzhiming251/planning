

#ifndef ONBOARD_PLANNER_INITIALIZER_COST_PROVIDER_H_
#define ONBOARD_PLANNER_INITIALIZER_COST_PROVIDER_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/types/span.h"
#include "plan_common/path_sl_boundary.h"
#include "decider/initializer/collision_checker.h"
#include "decider/initializer/cost_feature.h"
#include "decider/initializer/geometry/geometry_form.h"
#include "decider/initializer/motion_form.h"
#include "decider/initializer/ref_speed_table.h"
//#include "ml/captain_net/captain_net.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_planner_object_trajectories.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/drive_passage.h"
namespace st::planning {

class CostProviderBase {
 public:
  absl::Span<const std::string> cost_names() const { return cost_names_; }

  absl::Span<const double> weights() const { return weights_; }

  void ComputeDpCost(double start_t, const MotionForm* motion_form,
                     absl::Span<double> cost) const;
  IgnoreTrajMap ComputeInteractiveDpCost(double start_t,
                                         const MotionForm* motion_form,
                                         const IgnoreTrajMap& ignored_trajs,
                                         absl::Span<double> cost) const;
  void ComputeDpLeadingObjCost(double start_t, const MotionForm* motion_form,
                               absl::Span<double> cost) const;
  void ComputeRefLineCost(const GeometryForm* geometry_form, bool terminating,
                          absl::Span<double> cost) const;

 protected:
  template <typename Config>
  void BuildWeightTable(const Config& cost_config);

  std::vector<std::unique_ptr<FeatureCost>> features_;

  InitializerConfig initializer_params_;

 private:
  // The name of each feature cost.
  std::vector<std::string> cost_names_;

  // The weight of each cost feature.
  std::vector<double> weights_;

  std::vector<int> feature_size_;
};

class CostProvider : public CostProviderBase {
 public:
  CostProvider(const DrivePassage& drive_passage,
               const InitializerConfig& initializer_params,
               const MotionConstraintParamsProto& motion_constraint_params,
               const std::vector<double>& stop_s_vec,
               const SpacetimeTrajectoryManager& st_traj_mgr,
               const std::vector<std::string>& leading_trajs,
               const VehicleGeometryParamsProto& vehicle_geom,
               const CollisionChecker* collision_checker,
               const PathSlBoundary* path_sl,
               const RefSpeedTable* ref_speed_table,
               //  const ml::captain_net::CaptainNetOutput* captain_net_output,
               const InitializerSceneType init_scene_type, bool is_lane_change,
               double start_accumulated_s, double max_accumulated_s,
               bool is_post_evaluation = false);
};

class RefLineCostProvider : public CostProviderBase {
 public:
  RefLineCostProvider(
      const SpacetimePlannerObjectTrajectories* st_planner_object_traj,
      const DrivePassage* drive_passage, const PathSlBoundary* path_sl,
      double geom_graph_mac_accum_s, double relaxed_center_max_curvature,
      const InitializerConfig& initializer_params);
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_INITIALIZER_COST_PROVIDER_H_
