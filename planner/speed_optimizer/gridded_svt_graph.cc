

#include "planner/speed_optimizer/gridded_svt_graph.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <ostream>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "boost/heap/fibonacci_heap.hpp"
#include "boost/heap/policies.hpp"
#include "boost/intrusive/link_mode.hpp"
#include "boost/intrusive/list.hpp"
#include "gflags/gflags.h"
#include "plan_common/log.h"
#include "plan_common/timer.h"
//#include "global/logging.h"
//#include "global/trace.h"
//#include "lite/check.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/speed/st_speed/speed_point.h"
#include "planner/speed_optimizer/svt_point.h"
#include "plan_common/util/status_macros.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "planner/speed_optimizer/speed_finder_flags.h"

DEFINE_bool(enable_sampling_dp_reference_speed, false,
            "True to penalize sampling dp result towards default cruise speed");

namespace st::planning {

namespace {
constexpr double kMaxEarlyStopTime = 1.0;  // s.
constexpr double kEps = 1.0e-6;
constexpr double kInf = std::numeric_limits<double>::infinity();

struct BestEndPoint {
  // If this struct used as value for absl::flat_hash_map, to support operator
  // `[]`, default constructor should be retained.
  BestEndPoint() = default;
  BestEndPoint(const SvtGraphPoint* p, double c) : point(p), cost(c) {}
  const SvtGraphPoint* point = nullptr;
  double cost = kInf;
};

// Find the index of the minimum cost svt graph point.
absl::StatusOr<int> FindOptimalCandidate(
    absl::Span<const SvtGraphPointRef> candidate_points) {
  if (candidate_points.empty()) {
    return absl::FailedPreconditionError("Candidate points empty.");
  }
  const auto optimal_point = std::min_element(
      candidate_points.begin(), candidate_points.end(),
      [](const SvtGraphPointRef& point1, const SvtGraphPointRef& point2) {
        return point1->total_cost() < point2->total_cost();
      });
  return optimal_point - candidate_points.begin();
}

std::vector<std::vector<double>> GenerateAccMatrix(
    double max_acceleration, double max_deceleration,
    double coarse_sampling_decel_thres, double coarse_sampling_num,
    double unit_acc, double total_length_s, int dimension_t, int unit_t,
    double cur_v) {
  constexpr int kKeepUnitAccLayerIndex = 2;
  constexpr int kNoNeedToAccelerateRapidlyLayerIndex = 3;
  constexpr int kNoNeedToDecelerateHardLayerIndex = 5;
  constexpr double kGentleAccelerateAcc = 1.0;     // m/ss
  constexpr double kGentleDecelerateAcc = -2.0;    // m/ss
  constexpr double kMaxAccStep = 1.0;              // m/ss
  constexpr double kAccStepGain = 0.1;             // m/ss
  constexpr double kMaxSampleGapToMaxValue = 0.3;  // m/ss

  std::vector<std::vector<double>> acc_matrix;
  acc_matrix.reserve(dimension_t);
  const int acc_sample_max_size =
      FloorToInt((max_acceleration - max_deceleration) / unit_acc) + 2;
  for (int i = 0; i < dimension_t - 1; ++i) {
    if (i > kNoNeedToAccelerateRapidlyLayerIndex) {
      max_acceleration = kGentleAccelerateAcc;
    }
    if (i > kNoNeedToDecelerateHardLayerIndex) {
      max_deceleration = kGentleDecelerateAcc;
    }

    std::vector<double> cur_layer_acc;
    cur_layer_acc.reserve(acc_sample_max_size);
    double acc_step = unit_acc;
    if (i > kKeepUnitAccLayerIndex) {
      acc_step = std::min(
          acc_step + (i - kKeepUnitAccLayerIndex) * kAccStepGain, kMaxAccStep);
    }
    // Make sure acc = 0.0 can be sampled.
    for (double acc = 0.0; acc <= max_acceleration; acc += acc_step) {
      cur_layer_acc.push_back(acc);
    }
    if (max_acceleration - cur_layer_acc.back() > kMaxSampleGapToMaxValue) {
      cur_layer_acc.push_back(max_acceleration);
    } else {
      cur_layer_acc.back() = max_acceleration;
    }
    const double acc_coarse_step = std::max(
        (coarse_sampling_decel_thres - max_deceleration) / coarse_sampling_num,
        acc_step);
    for (double acc = -acc_step, acc_thres = coarse_sampling_decel_thres + kEps;
         acc >= max_deceleration; acc -= acc_step) {
      if (acc < acc_thres) {
        acc_step = acc_coarse_step;
      }
      cur_layer_acc.push_back(acc);
    }
    if (cur_layer_acc.back() - max_deceleration > kMaxSampleGapToMaxValue) {
      cur_layer_acc.push_back(max_deceleration);
    } else {
      cur_layer_acc.back() = max_deceleration;
    }
    acc_matrix.push_back(std::move(cur_layer_acc));
  }
  // Make sure stop speed profile can be sampled.
  const double stoppable_decel = -cur_v / unit_t;
  if (stoppable_decel > max_deceleration) {
    acc_matrix.front().push_back(stoppable_decel);
  }

  return acc_matrix;
}
}  // namespace

GriddedSvtGraph::GriddedSvtGraph(
    const StGraphData* st_graph_data, double init_v, double init_a,
    double max_acceleration, double max_deceleration,
    const SpeedFinderParamsProto* speed_finder_params, double speed_cap,
    std::vector<StBoundaryWithDecision*> st_boundaries_wd,
    const SpacetimeTrajectoryManager& st_traj_mgr)
    : st_graph_data_(st_graph_data),
      sorted_st_boundaries_with_decision_(std::move(st_boundaries_wd)),
      init_v_(init_v),
      init_a_(init_a),
      speed_finder_params_(CHECK_NOTNULL(speed_finder_params)),
      dp_params_(&speed_finder_params->sampling_dp_speed_params()),
      dp_svt_cost_(speed_finder_params_, st_graph_data_->total_time(),
                   st_graph_data_->path_length(), init_v,
                   &sorted_st_boundaries_with_decision_, st_traj_mgr),
      total_duration_t_(st_graph_data_->total_time()),
      unit_t_(dp_params_->unit_t()),
      total_length_s_(st_graph_data_->path_length()),
      total_length_v_(std::max(init_v_, speed_cap) +
                      dp_params_->speed_exceeding_cap()),
      unit_v_(dp_params_->unit_v()),
      max_acceleration_(max_acceleration),
      max_deceleration_(max_deceleration),
      st_traj_mgr_(st_traj_mgr) {
  int protective_idx = 0;
  for (int i = 0; i < sorted_st_boundaries_with_decision_.size(); ++i) {
    if (sorted_st_boundaries_with_decision_[i]
            ->raw_st_boundary()
            ->is_protective()) {
      if (protective_idx != i) {
        std::swap(sorted_st_boundaries_with_decision_[i],
                  sorted_st_boundaries_with_decision_[protective_idx]);
      }
      ++protective_idx;
    }
  }
  dp_svt_cost_.SetFollowDistanceRelSpeedPlf(PiecewiseLinearFunctionFromProto(
      speed_finder_params->follow_distance_rel_speed_plf()));
}

absl::Status GriddedSvtGraph::InitLayers() {
  // Total t and v are always fixed value, but total s is dynamic, so sample t
  // and v with fixed step, but change s step length according to total s.
  unit_s_ = std::max(total_length_s_ / dp_params_->s_dimension_size(), 0.15);
  unit_inv_t_ = 1.0 / unit_t_;
  unit_inv_s_ = 1.0 / unit_s_;
  unit_inv_v_ = 1.0 / unit_v_;

  // Calculate dimension size.
  dimension_t_ = CeilToInt(total_duration_t_ * unit_inv_t_) + 1;
  dimension_grid_v_ = CeilToInt(total_length_v_ * unit_inv_v_);
  dimension_grid_s_ = CeilToInt(total_length_s_ * unit_inv_s_);

  min_early_stop_index_t_ = std::max(
      dimension_t_ - RoundToInt(kMaxEarlyStopTime * unit_inv_t_) - 1, 0);

  // Sanity check for numerical stability
  if (unit_t_ < kEps) {
    return absl::InternalError("unit_t is smaller than the kEps.");
  }
  if (unit_v_ < kEps) {
    return absl::InternalError("unit_v is smaller than the kEps.");
  }
  if (unit_s_ < kEps) {
    return absl::InternalError("unit_s is smaller than the kEps.");
  }

  // Sanity check on s,v,t dimension size.
  if (dimension_grid_s_ < 1 || dimension_grid_v_ < 1 || dimension_t_ < 1) {
    return absl::InternalError(absl::StrFormat(
        "Sampling-dp discretized space dimension error, s:%d, v:%d, t:%d",
        dimension_grid_s_, dimension_grid_v_, dimension_t_));
  }

  // Set nullptr.
  layers_ =
      std::vector<std::vector<std::vector<SvtGraphPointRef>>>(dimension_t_);
  for (auto& layer : layers_) {
    layer.resize(dimension_grid_s_);
    for (auto& s : layer) {
      s.resize(dimension_grid_v_);
    }
  }

  s_knots_.clear();
  s_knots_.reserve(dimension_grid_s_ + 1);
  v_knots_.clear();
  v_knots_.reserve(dimension_grid_v_ + 1);
  t_knots_.clear();
  t_knots_.reserve(dimension_t_);

  // TODO: Sampling by density.
  double cur_s = 0.0;
  for (int i = 0; i < dimension_grid_s_; ++i, cur_s += unit_s_) {
    s_knots_.push_back(cur_s);
  }
  s_knots_.push_back(total_length_s_);

  double cur_v = 0.0;
  for (int i = 0; i < dimension_grid_v_; ++i, cur_v += unit_v_) {
    v_knots_.push_back(cur_v);
  }
  v_knots_.push_back(total_length_v_);

  double cur_t = 0.0;
  for (int i = 0; i < dimension_t_ - 1; ++i, cur_t += unit_t_) {
    t_knots_.push_back(cur_t);
  }
  t_knots_.push_back(total_duration_t_);

  acc_matrix_ = GenerateAccMatrix(max_acceleration_, max_deceleration_,
                                  dp_params_->coarse_sampling_decel_thres(),
                                  dp_params_->coarse_sampling_num(),
                                  dp_params_->unit_acc(), total_length_s_,
                                  dimension_t_, unit_t_, init_v_);

  return absl::OkStatus();
}
struct Cmp {
  bool operator()(const SvtGraphPoint* point1,
                  const SvtGraphPoint* point2) const {
    return point1->total_cost() > point2->total_cost();
  }
};

absl::StatusOr<SvGridIndexSet> GriddedSvtGraph::SearchByHybridDijkstra() {
  Timer timer(__FUNCTION__);

  boost::heap::fibonacci_heap<SvtGraphPoint*, boost::heap::compare<Cmp>> pq;
  absl::flat_hash_map<int, decltype(pq)::handle_type> handle_map;

  const double cruise_speed = st_graph_data_->cruise_speed();
  const auto& speed_limit_provider = st_graph_data_->speed_limit_provider();

  RETURN_IF_ERROR(InitLayers());

  const int init_point_grid_index_v = FloorToInt(init_v_ * unit_inv_v_);
  auto& start_point = layers_[0][0][init_point_grid_index_v];
  start_point = std::make_unique<SvtGraphPoint>(
      /*grid_index_s=*/0, init_point_grid_index_v, /*index_t=*/0,
      SvtPoint(/*s=*/0.0, init_v_, /*t=*/0.0));
  start_point->set_total_cost(0.0);
  handle_map.emplace(start_point->id(), pq.emplace(start_point.get()));
  const auto st_boundary_decisions =
      dp_svt_cost_.GetStBoundaryDecisionsForInitPoint(*start_point);
  start_point->UpdateStBoundaryDecisions(st_boundary_decisions);

  SvGridIndexSet indices_result;

  while (!pq.empty()) {
    SvtGraphPoint& current_svt_point = *pq.top();
    pq.pop();

    const double cur_t = current_svt_point.index_t() * unit_t_;
    const int next_index_t = current_svt_point.index_t() + 1;
    const double next_t = next_index_t * unit_t_;

    const bool reach_destination_t = (next_index_t == dimension_t_);
    if (reach_destination_t) {
      indices_result.emplace(current_svt_point.grid_index_s(),
                             current_svt_point.grid_index_v());
      return indices_result;
    }

    auto next_svt_points = ExpandByConstAccModel(
        current_svt_point.index_t(), cur_t, next_t, next_index_t,
        speed_limit_provider, cruise_speed, init_v_, &current_svt_point);

    const bool reach_destination_s =
        next_svt_points.empty() &&
        current_svt_point.index_t() >= min_early_stop_index_t_ &&
        current_svt_point.grid_index_s() == dimension_grid_s_ - 1;
    if (reach_destination_s) {
      return indices_result;
    }

    for (SvtGraphPointRef& next_svt_point : next_svt_points) {
      SvtGraphPointRef& svt_point_in_layer =
          layers_[next_svt_point->index_t()][next_svt_point->grid_index_s()]
                 [next_svt_point->grid_index_v()];
      if (svt_point_in_layer != nullptr && svt_point_in_layer->is_closed()) {
        continue;
      }
      if (svt_point_in_layer == nullptr) {
        svt_point_in_layer = std::move(next_svt_point);
        handle_map.emplace(svt_point_in_layer->id(),
                           pq.emplace(svt_point_in_layer.get()));
      } else if (svt_point_in_layer->total_cost() >
                 next_svt_point->total_cost()) {
        const auto handle = FindOrNull(handle_map, svt_point_in_layer->id());
        CHECK_NOTNULL(handle);
        pq.update(*handle, next_svt_point.get());
        svt_point_in_layer = std::move(next_svt_point);
      }
    }
    current_svt_point.Close();
  }
  return absl::NotFoundError(
      "Hybrid dijkstra search failed to reach destination");
}

// Return last layer sv grid indices that had points located on when search
// finished.
absl::StatusOr<SvGridIndexSet> GriddedSvtGraph::SearchAndReturnFinalLayerPoints(
    ThreadPool* thread_pool) {
  // ("SearchAndReturnFinalLayerPoints");
  const double cruise_speed = st_graph_data_->cruise_speed();
  const auto& speed_limit_provider = st_graph_data_->speed_limit_provider();

  // step 1: Init.
  RETURN_IF_ERROR(InitLayers());

  SvGridIndexSet cur_layer_indices;
  SvGridIndexSet next_layer_indices;
  next_layer_indices.reserve(dimension_grid_s_ * dimension_grid_v_);

  std::vector<std::vector<std::vector<SvtGraphPointRef>>>
      next_layer_candidate_points(dimension_grid_s_);
  for (auto& layer : next_layer_candidate_points) {
    layer.resize(dimension_grid_v_);
  }
  // This is an experimental value.
  constexpr int kPointsNumPerGrid = 15;
  for (auto& row : next_layer_candidate_points) {
    for (auto& grid : row) {
      grid.reserve(kPointsNumPerGrid);
    }
  }

  double cur_t = 0.0;
  // t-level.
  for (size_t i = 0; i + 1 < layers_.size(); ++i, cur_t += unit_t_) {
    auto& cur_layer = layers_[i];
    auto& next_layer = layers_[i + 1];
    const double next_t = cur_t + unit_t_;
    const int next_point_index_t = i + 1;
    next_layer_indices.clear();

    // Only one point need to expand in the first layer(t).
    if (i == 0) {
      if (init_v_ > total_length_v_) {
        return absl::InternalError(
            "Init point speed exceeds total length speed.");
      }
      const int init_point_grid_index_v = FloorToInt(init_v_ * unit_inv_v_);
      layers_[0][0][init_point_grid_index_v] = std::make_unique<SvtGraphPoint>(
          /*grid_index_s=*/0, init_point_grid_index_v, /*index_t=*/0,
          SvtPoint(/*s=*/0.0, init_v_, /*t=*/0.0));
      layers_[0][0][init_point_grid_index_v]->set_total_cost(0.0);

      cur_layer_indices.emplace(/*grid_index_s=*/0, init_point_grid_index_v);

      const auto st_boundary_decisions =
          dp_svt_cost_.GetStBoundaryDecisionsForInitPoint(
              *layers_[0][0][init_point_grid_index_v]);
      layers_[0][0][init_point_grid_index_v]->UpdateStBoundaryDecisions(
          st_boundary_decisions);
    }

    // step 2: Expand by sampling method.

    for (auto& row : next_layer_candidate_points) {
      for (auto& grid : row) {
        grid.clear();
      }
    }

    for (const auto& index : cur_layer_indices) {
      // SvGrid is a virtual GriddedSvtPoint, we can find all SvGrid by
      // a fixed step, and update params.
      auto& sv_grid = cur_layer[index.grid_index_s_][index.grid_index_v_];
      if (sv_grid == nullptr) {
        return absl::InternalError("Current sv grid not existing.");
      }
      ExpandToNextLayer(i, cur_t, next_t, next_point_index_t,
                        speed_limit_provider, cruise_speed, init_v_,
                        sv_grid.get(), &next_layer_candidate_points,
                        &next_layer_indices);
    }

    // step 3: Update SvGrids.
    // TODO: Parallel for.
    for (const auto& next_index : next_layer_indices) {
      auto& grid_candidate_points =
          next_layer_candidate_points[next_index.grid_index_s_]
                                     [next_index.grid_index_v_];
      auto& optimal_candidate =
          next_layer[next_index.grid_index_s_][next_index.grid_index_v_];
      ASSIGN_OR_RETURN(auto find_res,
                       FindOptimalCandidate(grid_candidate_points));
      optimal_candidate = std::move(grid_candidate_points[find_res]);
      optimal_candidate->pre_point()->set_next_point(&*optimal_candidate);
    }

    // Update indices.
    cur_layer_indices.swap(next_layer_indices);
  }

  return cur_layer_indices;
}

absl::Status GriddedSvtGraph::FindOptimalPreliminarySpeed(
    SpeedVector* preliminary_speed, SamplingDpDebugProto* sampling_dp_debug,
    ThreadPool* thread_pool) {
  // ("FindOptimalPreliminarySpeed");

  ASSIGN_OR_RETURN(const auto final_layer_indices,
                   SearchAndReturnFinalLayerPoints(thread_pool));

  auto preliminary_speed_with_cost =
      GetSpeedProfileAndCompleteStBoundariesWithDecision(final_layer_indices);
  if (!preliminary_speed_with_cost.ok()) {
    return preliminary_speed_with_cost.status();
  } else {
    *preliminary_speed = preliminary_speed_with_cost->preliminary_speed;
  }
  return absl::OkStatus();
}

absl::Status GriddedSvtGraph::FindOptimalPreliminarySpeedWithCost(
    PreliminarySpeedWithCost* preliminary_speed_with_cost,
    SamplingDpDebugProto* sampling_dp_debug, ThreadPool* thread_pool) {
  // ("FindOptimalPreliminarySpeedWithCost");

  SvGridIndexSet final_layer_indices;
  if (FLAGS_planner_use_hybrid_dijkstra_to_search_dp_speed) {
    ASSIGN_OR_RETURN(final_layer_indices, SearchByHybridDijkstra());
  } else {
    ASSIGN_OR_RETURN(final_layer_indices,
                     SearchAndReturnFinalLayerPoints(thread_pool));
  }
  ASSIGN_OR_RETURN(
      *preliminary_speed_with_cost,
      GetSpeedProfileAndCompleteStBoundariesWithDecision(final_layer_indices));
  return absl::OkStatus();
}

absl::Status GriddedSvtGraph::GenerateSamplingDpSpeedProfileCandidateSet(
    std::vector<PreliminarySpeedWithCost>* candidate_speed_profiles,
    SamplingDpDebugProto* sampling_dp_debug,
    InteractiveSpeedDebugProto::CandidateSet* candidate_set_debug,
    ThreadPool* thread_pool) {
  // ("GenerateSamplingDpSpeedProfileCandidateSet");

  ASSIGN_OR_RETURN(const SvGridIndexSet final_layer_indices,
                   SearchAndReturnFinalLayerPoints(thread_pool));
  *candidate_speed_profiles =
      SampleSpeedProfilesFromSamplingDp(final_layer_indices);
  return absl::OkStatus();
}

std::vector<SvtGraphPointRef> GriddedSvtGraph::ExpandByConstAccModel(
    int cur_layer_index, double cur_t, double next_t, int next_point_index_t,
    const SpeedLimitProvider& speed_limit_provider, double cruise_speed,
    double init_speed, SvtGraphPoint* cur_point) {
  using StBoundaryDecision = SvtGraphPoint::StBoundaryDecision;

  std::vector<SvtGraphPointRef> candidate_points;
  candidate_points.reserve(acc_matrix_[cur_layer_index].size());
  const double cur_s = cur_point->point().s();
  const double cur_v = cur_point->point().v();
  const auto prev_speed_limit_or =
      speed_limit_provider.GetSpeedLimitByTimeAndS(cur_t, cur_s);
  double prev_speed_limit = std::numeric_limits<double>::max();
  if (prev_speed_limit_or.has_value()) {
    prev_speed_limit = *prev_speed_limit_or;
  }
  // Increase the cost for lane merge speed limit
  auto prev_lane_merge_speed_limit_or =
      speed_limit_provider.GetLaneMergeSpeedLimitByTime(cur_t);

  for (const double acc : acc_matrix_[cur_layer_index]) {
    const double next_s = cur_s + cur_v * unit_t_ + 0.5 * acc * Sqr(unit_t_);
    const double next_v = cur_v + acc * unit_t_;

    if (next_v > (total_length_v_ - kEps) ||
        next_s > (total_length_s_ - kEps) || next_v < 0.0 || next_s < 0.0 ||
        next_s < cur_s) {
      continue;
    }

    const int next_point_grid_index_s = FloorToInt(next_s * unit_inv_s_);
    const int next_point_grid_index_v = FloorToInt(next_v * unit_inv_v_);

    auto& next_point =
        candidate_points.emplace_back(std::make_unique<SvtGraphPoint>(
            next_point_grid_index_s, next_point_grid_index_v,
            next_point_index_t, SvtPoint(next_s, next_v, next_t)));
    next_point->set_spatial_potential_cost(
        dp_svt_cost_.GetSpatialPotentialCost(next_s));
    next_point->set_vertex_cost(next_point->spatial_potential_cost());

    const auto curr_speed_limit_or =
        speed_limit_provider.GetSpeedLimitByTimeAndS(next_t, next_s);
    double curr_speed_limit = std::numeric_limits<double>::max();
    if (curr_speed_limit_or.has_value()) {
      curr_speed_limit = *curr_speed_limit_or;
    }
    prev_speed_limit = next_point_index_t == 1
                           ? curr_speed_limit
                           : std::min(prev_speed_limit, curr_speed_limit);

    // Increase the cost for lane merge speed limit
    const auto curr_lane_merge_speed_limit_or =
        speed_limit_provider.GetLaneMergeSpeedLimitByTime(next_t);
    if (next_point_index_t == 1) {
      prev_lane_merge_speed_limit_or = curr_lane_merge_speed_limit_or;
    }
    if (curr_lane_merge_speed_limit_or.has_value()) {
      prev_lane_merge_speed_limit_or =
          prev_lane_merge_speed_limit_or.has_value()
              ? std::min(*prev_lane_merge_speed_limit_or,
                         *curr_lane_merge_speed_limit_or)
              : curr_lane_merge_speed_limit_or;
    }

    const double average_speed = 0.5 * (cur_v + next_v);
    const double speed_limit_cost = dp_svt_cost_.GetSpeedLimitCost(
        average_speed, prev_speed_limit, prev_lane_merge_speed_limit_or);
    const double reference_speed_cost =
        dp_svt_cost_.GetReferenceSpeedCost(average_speed, cruise_speed);
    const double accel_cost = dp_svt_cost_.GetAccelCost(acc);
    double object_cost = 0.0;
    std::vector<StBoundaryDecision> st_boundary_decisions;
    st_boundary_decisions.reserve(sorted_st_boundaries_with_decision_.size());
    dp_svt_cost_.GetStBoundaryCostAndDecisions(*cur_point, *next_point,
                                               init_speed, &object_cost,
                                               &st_boundary_decisions);
    const double edge_cost =
        speed_limit_cost + reference_speed_cost + accel_cost + object_cost;
    const double total_cost =
        next_point->vertex_cost() + edge_cost + cur_point->total_cost();

    next_point->set_speed_limit_cost(speed_limit_cost);
    next_point->set_reference_speed_cost(reference_speed_cost);
    next_point->set_accel_cost(accel_cost);
    next_point->set_object_cost(object_cost);
    next_point->set_edge_cost(edge_cost);
    next_point->set_total_cost(total_cost);
    next_point->set_pre_point(cur_point);
    next_point->set_acc_from_pre_point(acc);
    next_point->UpdateStBoundaryDecisions(st_boundary_decisions);
  }

  return candidate_points;
}

void GriddedSvtGraph::ExpandToNextLayer(
    int cur_layer_index, double cur_t, double next_t, int next_point_index_t,
    const SpeedLimitProvider& speed_limit_provider, double cruise_speed,
    double init_speed, SvtGraphPoint* cur_point,
    std::vector<std::vector<std::vector<SvtGraphPointRef>>>*
        next_layer_candidate_points,
    SvGridIndexSet* index_range) {
  // Note: next_layer_candidate_points and index_range must be guaranteed to be
  // empty by the call site.
  auto candidate_points = ExpandByConstAccModel(
      cur_layer_index, cur_t, next_t, next_point_index_t, speed_limit_provider,
      cruise_speed, init_speed, cur_point);

  for (auto& candidate_point : candidate_points) {
    const int index_s = candidate_point->grid_index_s();
    const int index_v = candidate_point->grid_index_v();
    index_range->emplace(index_s, index_v);
    (*next_layer_candidate_points)[index_s][index_v].push_back(
        std::move(candidate_point));
  }
}

void GriddedSvtGraph::AddAllSpeedProfilesToDebug(
    const SvGridIndexSet& final_layer_index_range,
    SamplingDpDebugProto* sampling_dp_debug) {
  // ("AddAllSpeedProfilesToDebug");

  const auto get_speed_profile = [](const SvtGraphPoint* cur_point) {
    SpeedProfileDebugProto speed_profile;
    // Backtracking to find the optimal speed profile.
    while (cur_point != nullptr) {
      auto* point = speed_profile.add_svt_graph_points();
      auto* speed_point = point->mutable_speed_point();
      speed_point->set_s(cur_point->point().s());
      speed_point->set_v(cur_point->point().v());
      speed_point->set_a(cur_point->acc_from_pre_point());
      speed_point->set_t(cur_point->point().t());
      point->set_speed_limit_cost(cur_point->speed_limit_cost());
      point->set_reference_speed_cost(cur_point->reference_speed_cost());
      point->set_accel_cost(cur_point->accel_cost());
      point->set_object_cost(cur_point->object_cost());
      point->set_spatial_potential_cost(cur_point->spatial_potential_cost());
      point->set_total_cost(cur_point->total_cost());
      cur_point = cur_point->pre_point();
    }
    // If debug only, no need to reverse.
    return speed_profile;
  };

  for (const auto& final_layer_index : final_layer_index_range) {
    const auto& end_point = layers_.back()[final_layer_index.grid_index_s_]
                                          [final_layer_index.grid_index_v_];
    *sampling_dp_debug->add_speed_profiles() = get_speed_profile(&*end_point);
  }

  for (const auto& layer : layers_) {
    const auto& end_s_row = layer.back();
    for (const auto& end_point : end_s_row) {
      if (end_point == nullptr || !end_point->next_point().empty()) {
        continue;
      }
      *sampling_dp_debug->add_speed_profiles() = get_speed_profile(&*end_point);
    }
  }
}

absl::StatusOr<PreliminarySpeedWithCost>
GriddedSvtGraph::GetSpeedProfileAndCompleteStBoundariesWithDecision(
    const SvGridIndexSet& final_layer_index_range) {
  // ("GetSpeedProfileAndCompleteStBoundariesWithDecision");

  double min_cost = kInf;
  const SvtGraphPoint* best_end_point = nullptr;

  for (const auto& final_layer_index : final_layer_index_range) {
    const auto& end_point = layers_.back()[final_layer_index.grid_index_s_]
                                          [final_layer_index.grid_index_v_];
    if (end_point->total_cost() < min_cost) {
      best_end_point = &*end_point;
      min_cost = end_point->total_cost();
    }
  }

  for (const auto& layer : layers_) {
    const auto& end_s_row = layer.back();
    for (const auto& end_point : end_s_row) {
      if (end_point == nullptr || !end_point->next_point().empty()) {
        continue;
      }
      if (end_point->total_cost() < min_cost) {
        best_end_point = &*end_point;
        min_cost = end_point->total_cost();
      }
    }
  }

  // In some extreme cases, sampling dp cannot find a feasible trajectory, see a
  // typical
  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    return absl::InternalError(msg);
  }

  VLOG(2) << "Best end point cost: " << best_end_point->total_cost();
  std::vector<SpeedPoint> speed_profile;
  speed_profile.reserve(dimension_t_);
  const SvtGraphPoint* cur_point = best_end_point;

  // Backtracking to find the optimal speed profile.
  while (cur_point != nullptr) {
    VLOG(3) << "Time: " << cur_point->point().t();
    VLOG(3) << "S: " << cur_point->point().s();
    VLOG(3) << "V: " << cur_point->point().v();
    // Get speed point.
    SpeedPoint speed_point;
    speed_point.set_s(cur_point->point().s());
    speed_point.set_v(cur_point->point().v());
    speed_point.set_a(cur_point->acc_from_pre_point());
    speed_point.set_t(cur_point->point().t());
    speed_profile.push_back(std::move(speed_point));
    for (auto* st_boundary_with_decision :
         sorted_st_boundaries_with_decision_) {
      if (st_boundary_with_decision->decision_type() !=
          StBoundaryProto::UNKNOWN) {
        continue;
      }
      const auto decision =
          cur_point->GetStBoundaryDecision(st_boundary_with_decision->id());
      if (decision) {
        if (*decision == StBoundaryProto::UNKNOWN) {
          return absl::InternalError(
              "Current point decision should not be UNKNOWN.");
        }
        // Modify decision in place.
        st_boundary_with_decision->set_decision_type(*decision);
        st_boundary_with_decision->set_decision_reason(
            StBoundaryProto::SAMPLING_DP);
        st_boundary_with_decision->set_decision_info("decided by sampling dp");
      } else {
        continue;
      }
    }
    cur_point = cur_point->pre_point();
  }
  std::reverse(speed_profile.begin(), speed_profile.end());
  // Set next point `acc_from_pre_point` as current point acc. Keep last point
  // acc to avoid sharp jerk.
  for (int i = 1; i < speed_profile.size() - 1; ++i) {
    speed_profile[i].set_a(speed_profile[i + 1].a());
  }
  speed_profile.front().set_a(init_a_);

  return PreliminarySpeedWithCost(best_end_point->total_cost(),
                                  SpeedVector(std::move(speed_profile)));
}

std::vector<PreliminarySpeedWithCost>
GriddedSvtGraph::SampleSpeedProfilesFromSamplingDp(
    const SvGridIndexSet& final_layer_indices) {
  // ("SampleSpeedProfilesFromSamplingDp");

  // Sample minimal cost speed profile in s dimension at final layer.
  // <s_dimension, BestEndPoint>
  absl::flat_hash_map<int, BestEndPoint> final_layer_points;
  constexpr int kSDimensionSampleInterval = 4;
  for (const auto& index : final_layer_indices) {
    const auto& end_point =
        layers_.back()[index.grid_index_s_][index.grid_index_v_];
    const int sample_index = index.grid_index_s_ / kSDimensionSampleInterval;
    if (final_layer_points.find(sample_index) != final_layer_points.end()) {
      if (end_point->total_cost() < final_layer_points[sample_index].cost) {
        final_layer_points[sample_index] =
            BestEndPoint(&*end_point, end_point->total_cost());
      }
    } else {
      final_layer_points[sample_index] =
          BestEndPoint(&*end_point, end_point->total_cost());
    }
  }

  // Select minimal cost speed profile for last row(s) of every layer(t).
  // <t_dimension, BestEndPoint>
  absl::flat_hash_map<int, BestEndPoint> layers_final_point;
  for (int i = 0; i < layers_.size() - 1; ++i) {
    for (const auto& end_point : layers_[i].back()) {
      if (end_point == nullptr || !end_point->next_point().empty()) {
        continue;
      }
      if (layers_final_point.find(i) != layers_final_point.end()) {
        if (end_point->total_cost() < layers_final_point[i].cost) {
          layers_final_point[i] =
              BestEndPoint(&*end_point, end_point->total_cost());
        }
      } else {
        layers_final_point[i] =
            BestEndPoint(&*end_point, end_point->total_cost());
      }
    }
  }

  const auto get_speed_profile = [](double init_point_a,
                                    const SvtGraphPoint* cur_point) {
    std::vector<SpeedPoint> speed_profile;
    speed_profile.reserve(cur_point->index_t() + 1);
    // Backtracking to find the optimal speed profile.
    while (cur_point != nullptr) {
      SpeedPoint speed_point;
      speed_point.set_s(cur_point->point().s());
      speed_point.set_v(cur_point->point().v());
      speed_point.set_a(cur_point->acc_from_pre_point());
      speed_point.set_t(cur_point->point().t());
      speed_profile.push_back(std::move(speed_point));
      cur_point = cur_point->pre_point();
    }
    std::reverse(speed_profile.begin(), speed_profile.end());

    // Set next point `acc_from_pre_point` as current point acc. Keep last point
    // acc to avoid sharp jerk.
    for (int i = 1; i < speed_profile.size() - 1; ++i) {
      speed_profile[i].set_a(speed_profile[i + 1].a());
    }
    speed_profile.front().set_a(init_point_a);

    return speed_profile;
  };

  std::vector<PreliminarySpeedWithCost> speed_profiles;
  speed_profiles.reserve(layers_final_point.size() + final_layer_points.size());

  for (const auto& [_, best_end_point] : final_layer_points) {
    speed_profiles.emplace_back(
        best_end_point.cost,
        SpeedVector(get_speed_profile(init_a_, best_end_point.point)));
  }
  for (const auto& [_, best_end_point] : layers_final_point) {
    speed_profiles.emplace_back(
        best_end_point.cost,
        SpeedVector(get_speed_profile(init_a_, best_end_point.point)));
  }

  return speed_profiles;
}

}  // namespace st::planning
