

#ifndef ONBOARD_PLANNER_SPEED_GRIDDED_SVT_GRAPH_H_
#define ONBOARD_PLANNER_SPEED_GRIDDED_SVT_GRAPH_H_

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "boost/container_hash/extensions.hpp"
#include "plan_common/async/thread_pool.h"
#include "gflags/gflags.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "plan_common/speed/st_speed/speed_limit_provider.h"
#include "plan_common/speed/st_speed/speed_vector.h"
#include "planner/speed_optimizer/dp_svt_cost.h"
#include "planner/speed_optimizer/st_graph_data.h"
#include "planner/speed_optimizer/svt_graph_point.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"

DECLARE_bool(enable_sampling_dp_reference_speed);

namespace st::planning {

struct SvGridIndex {
  int grid_index_s_ = 0;
  int grid_index_v_ = 0;

  SvGridIndex(int grid_index_s, int grid_index_v)
      : grid_index_s_(grid_index_s), grid_index_v_(grid_index_v) {}

  bool operator==(const SvGridIndex& other) const {
    return grid_index_s_ == other.grid_index_s_ &&
           grid_index_v_ == other.grid_index_v_;
  }
  bool operator!=(const SvGridIndex& other) const { return !(*this == other); }
  bool operator<(const SvGridIndex& other) const {
    return grid_index_s_ < other.grid_index_s_ ||
           (grid_index_s_ == other.grid_index_s_ &&
            grid_index_v_ < other.grid_index_v_);
  }

  struct HashFunction {
    std::size_t operator()(const SvGridIndex& index) const {
      std::size_t seed = 0;
      boost::hash_combine(seed, index.grid_index_s_);
      boost::hash_combine(seed, index.grid_index_v_);
      return seed;
    }
  };

  std::string DebugString() const {
    return absl::StrCat("(", grid_index_s_, ", ", grid_index_v_, ")");
  }
};

using SvGridIndexSet =
    absl::flat_hash_set<SvGridIndex, SvGridIndex::HashFunction>;

struct PreliminarySpeedWithCost {
  PreliminarySpeedWithCost() = default;
  PreliminarySpeedWithCost(double c, SpeedVector sv)
      : cost(c), preliminary_speed(std::move(sv)) {}
  double cost = 0.0;
  SpeedVector preliminary_speed;
};

class GriddedSvtGraph {
 public:
  GriddedSvtGraph(const StGraphData* st_graph_data, double init_v,
                  double init_a, double max_acceleration,
                  double max_deceleration,
                  const SpeedFinderParamsProto* speed_finder_params,
                  double speed_cap,
                  std::vector<StBoundaryWithDecision*> st_boundaries_wd,
                  const SpacetimeTrajectoryManager& st_traj_mgr);

  void SwapStBoundariesWithDecision(
      std::vector<StBoundaryWithDecision*>* st_boundaries_with_decision) {
    sorted_st_boundaries_with_decision_.swap(*st_boundaries_with_decision);
  }

  absl::Status FindOptimalPreliminarySpeed(
      SpeedVector* preliminary_speed, SamplingDpDebugProto* sampling_dp_debug,
      ThreadPool* thread_pool);

  absl::Status FindOptimalPreliminarySpeedWithCost(
      PreliminarySpeedWithCost* preliminary_speed_with_cost,
      SamplingDpDebugProto* sampling_dp_debug, ThreadPool* thread_pool);

  absl::Status GenerateSamplingDpSpeedProfileCandidateSet(
      std::vector<PreliminarySpeedWithCost>* candidate_speed_profiles,
      SamplingDpDebugProto* sampling_dp_debug,
      InteractiveSpeedDebugProto::CandidateSet* candidate_set_debug,
      ThreadPool* thread_pool);

 private:
  // Return last layer sv grid indices that had points located on when search
  // finished.
  absl::StatusOr<SvGridIndexSet> SearchAndReturnFinalLayerPoints(
      ThreadPool* thread_pool);

  absl::StatusOr<SvGridIndexSet> SearchByHybridDijkstra();

  // Discretize sampling space and set the grid size of S,V,T respectively.
  absl::Status InitLayers();

  std::vector<SvtGraphPointRef> ExpandByConstAccModel(
      int cur_layer_index, double cur_t, double next_t, int next_point_index_t,
      const SpeedLimitProvider& speed_limit_provider, double cruise_speed,
      double init_speed, SvtGraphPoint* cur_point);

  void ExpandToNextLayer(
      int cur_layer_index, double cur_t, double next_t, int next_point_index_t,
      const SpeedLimitProvider& speed_limit_provider, double cruise_speed,
      double init_speed, SvtGraphPoint* cur_point,
      std::vector<std::vector<std::vector<SvtGraphPointRef>>>*
          next_layer_candidate_points,
      SvGridIndexSet* index_range);

  absl::StatusOr<PreliminarySpeedWithCost>
  GetSpeedProfileAndCompleteStBoundariesWithDecision(
      const SvGridIndexSet& final_layer_index_range);

  std::vector<PreliminarySpeedWithCost> SampleSpeedProfilesFromSamplingDp(
      const SvGridIndexSet& final_layer_indices);

  void AddAllSpeedProfilesToDebug(const SvGridIndexSet& final_layer_index_range,
                                  SamplingDpDebugProto* sampling_dp_debug);

 private:
  using SamplingDpSpeedParamsProto =
      SpeedFinderParamsProto::SamplingDpSpeedParamsProto;

  const StGraphData* st_graph_data_;

  // This vector is sorted. The protective st-boundaries are on the front, and
  // the other st-boundaries are on the back.
  std::vector<StBoundaryWithDecision*> sorted_st_boundaries_with_decision_;

  double init_v_;
  double init_a_;
  const SpeedFinderParamsProto* speed_finder_params_;
  const SamplingDpSpeedParamsProto* dp_params_;

  DpSvtCost dp_svt_cost_;

  double total_duration_t_ = 0.0;  // Default total_duration_t = 10 s.
  double unit_t_ = 0.0;
  double unit_inv_t_ = 0.0;
  int dimension_t_ = 0;  // Dimension represents number of grid.
  int min_early_stop_index_t_ = 0;

  double total_length_s_ = 0.0;  // Total_length_s depends on extended path.
  double unit_s_ = 0.0;
  double unit_inv_s_ = 0.0;
  int dimension_grid_s_ = 0;

  double total_length_v_ = 0.0;  // Default total_length_v = 45.0 mph.
  double unit_v_ = 0.0;
  double unit_inv_v_ = 0.0;
  int dimension_grid_v_ = 0;
  double max_acceleration_ = 0.0;
  double max_deceleration_ = 0.0;

  std::vector<std::vector<double>> acc_matrix_;

  // Knots represent all of the index of grid (s,v).
  std::vector<double> t_knots_;
  std::vector<double> v_knots_;
  std::vector<double> s_knots_;

  // Layers_ represents the grid of (s, v) for every layer(t), such as
  // layers_[t][s][v].
  std::vector<std::vector<std::vector<SvtGraphPointRef>>> layers_;
  const SpacetimeTrajectoryManager& st_traj_mgr_;
};
}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SPEED_GRIDDED_SVT_GRAPH_H_
