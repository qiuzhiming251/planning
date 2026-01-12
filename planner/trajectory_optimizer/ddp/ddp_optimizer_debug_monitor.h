

#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_DEBUG_MONITOR_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_DEBUG_MONITOR_H_

#include <regex>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "plan_common/math/eigen.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/planner_object_manager.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "ddp_optimizer_monitor.h"

namespace st {
namespace planning {

constexpr double kTrajVisZInc =
    kSpaceTimeVisualizationDefaultTimeScale * kTrajectoryTimeStep;

template <typename PROB>
class IterationVisualizerMonitor : public DdpOptimizerMonitor<PROB> {
 public:
  IterationVisualizerMonitor(int horizon, const std::string& base_name)
      : DdpOptimizerMonitor<PROB>(horizon), base_name_(base_name) {}

  using OptimizerInspector =
      typename DdpOptimizerMonitor<PROB>::OptimizerInspector;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  void OnSolveStart(const StatesType& xs, const ControlsType& us,
                    const OptimizerInspector& oi) override {}

  void OnIterationEnd(int iteration, const StatesType& xs,
                      const ControlsType& us,
                      const OptimizerInspector& oi) override {}

  void OnSolveEnd(const StatesType& xs, const ControlsType& us,
                  const OptimizerInspector& oi) override {}

 private:
  const std::string base_name_;
};

template <typename PROB>
class IterationAvModelVisualizerMonitor : public DdpOptimizerMonitor<PROB> {
 public:
  struct circle {
    // Distances from circle center to RAC.
    double l = 0.0;
    // Angle between RAC->circle_center and av heading.
    double theta = 0.0;
    double r = 0.0;
  };

  IterationAvModelVisualizerMonitor(
      int horizon, const std::string& base_name,
      const std::vector<circle>& circles,
      const VehicleGeometryParamsProto* veh_geo_params)
      : DdpOptimizerMonitor<PROB>(horizon),
        base_name_(base_name),
        circles_(circles),
        veh_geo_params_(CHECK_NOTNULL(veh_geo_params)) {}

  using OptimizerInspector =
      typename DdpOptimizerMonitor<PROB>::OptimizerInspector;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  void OnSolveStart(const StatesType& xs, const ControlsType& us,
                    const OptimizerInspector& oi) override {
    // DrawAvModels(xs, us, "init_av_model", vis::Color(0.8, 0.4, 0.4));
  }

  void OnSolveEnd(const StatesType& xs, const ControlsType& us,
                  const OptimizerInspector& oi) override {
    // DrawAvModels(xs, us, "final_av_model", vis::Color(0.4, 0.4, 0.8));
  }

 private:
  const std::string base_name_;
  std::vector<circle> circles_;
  const VehicleGeometryParamsProto* veh_geo_params_;
};

template <typename PROB>
struct OptimizerSolverDebugMonitor : public DdpOptimizerMonitor<PROB> {
 public:
  using ObjectResponseProto =
      TrajectoryOptimizerDebugProto::ObjectResponseProto;

  OptimizerSolverDebugMonitor(int horizon, const TrajectoryPoint& av_pose,
                              const SpacetimeTrajectoryManager& st_traj_mgr)
      : DdpOptimizerMonitor<PROB>(horizon), av_pose_(av_pose) {
    for (const auto& traj : st_traj_mgr.trajectories()) {
      object_map[traj.traj_id()] = &traj;
    }
  }

  using OptimizerInspector =
      typename DdpOptimizerMonitor<PROB>::OptimizerInspector;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  void OnSolveStart(const StatesType& xs, const ControlsType& us,
                    const OptimizerInspector& oi) override {
    solve_start_xs = xs;
    solve_start_us = us;
    iterations.clear();
    object_responses.clear();
    init_costs.ddp_costs.clear();
    final_costs.ddp_costs.clear();
  }

  void OnSolveEnd(const StatesType& xs, const ControlsType& us,
                  const OptimizerInspector& oi) override {
    solve_end_xs = xs;
    solve_end_us = us;
    // Example cost names:
    //  - Object (R): for Agent1-idx0
    //  - Object (F): for 12354-1
    //  - Emeraude Object (R): for Phantom 1-idx0
    //
    // Regex tokens:
    //  - object ID.
    //  - prediction ID.
    static const std::regex obj_response_cost_regex =
        std::regex(".+for (.+)-(\\w+)");

    // Analyze the costs to deduce the objects we're responding to.
    for (const auto& [cost_name, cost, is_soft] : oi.named_costs) {
      std::smatch obj_response_cost_match;
      if (!std::regex_match(cost_name, obj_response_cost_match,
                            obj_response_cost_regex)) {
        continue;
      }

      // Costs below this threshold are considered too weak to correspond to an
      // active "response" to an object.
      constexpr double kCostThresholdResponse = 1.0;

      if (cost < kCostThresholdResponse) continue;

      CHECK_EQ(obj_response_cost_match.size(), 3);
      const std::string obj_id = obj_response_cost_match[1];
      const std::string obj_pred_index = obj_response_cost_match[2];
      // VLOG(3) << "Object response: cost name [" << cost_name
      //         << "] cost: " << cost << " obj_id: " << obj_id
      //         << " obj_pred_index: " << obj_pred_index
      //         << ", is_soft: " << is_soft;

      const std::string obj_traj_name = obj_id + "-" + obj_pred_index;
      const auto it = object_map.find(obj_traj_name);
      if (it == object_map.end()) {
        LOG_ERROR << "Unknown object ID " << obj_id << " from cost name "
                  << cost_name;
        continue;
      }
      const SpacetimeObjectTrajectory& st_obj = *(it->second);
      ObjectResponseProto& response = object_responses.emplace_back();
      response.set_object_id(obj_traj_name);
      response.set_cost(cost);
      response.set_cost_name(cost_name);

      // TODO: look for a way to find out the time of the peak of
      // the response, or a time range of the response. We may be responding to
      // an object (or its predicted trajectory) at a future time, and the
      // visualization should reflect that.
      response.set_peak_step(0);
      Vec2dToProto(st_obj.pose().pos(),
                   response.mutable_peak_object_position());
      const Vec2d force_dir =
          (st_obj.pose().pos() - av_pose_.pos()).normalized();
      Vec2dToProto(force_dir, response.mutable_peak_force_direction());
    }

    final_costs.cost = oi.cost;
    final_costs.ddp_costs.clear();
    for (const auto& named_cost : oi.named_costs) {
      // Don't log cost below this threshold.
      constexpr double kCostThresholdForLog = 1.0;
      if (named_cost.value < kCostThresholdForLog) continue;
      final_costs.ddp_costs.push_back(named_cost);
    }
  }

  void OnIterationStart(int iteration, const StatesType& xs,
                        const ControlsType& us,
                        const OptimizerInspector& oi) override {
    if (iteration >= iterations.size()) {
      iterations.emplace_back();
    }
    if (iteration == 0) {
      init_costs.cost = oi.cost;
      init_costs.ddp_costs.clear();
      for (const auto& named_cost : oi.named_costs) {
        // Don't log cost below this threshold.
        constexpr double kCostThresholdForLog = 1.0;
        if (named_cost.value < kCostThresholdForLog) continue;
        init_costs.ddp_costs.push_back(named_cost);
      }
    }
  }
  void OnIterationEnd(int iteration, const StatesType& xs,
                      const ControlsType& us,
                      const OptimizerInspector& oi) override {
    CHECK_EQ(iteration + 1, iterations.size());
    iterations.back().final_cost = oi.cost;
    iterations.back().js0 = oi.js0;
  }
  void OnLineSearchIterationEnd(int iteration, const StatesType& xs,
                                const ControlsType& us,
                                const StatesType& full_dxs,
                                const ControlsType& full_dus, double alpha,
                                double cost,
                                const OptimizerInspector& oi) override {
    CHECK_EQ(iteration + 1, iterations.size());
    auto& iter = iterations.back();
    iter.alphas.push_back(alpha);
    iter.line_search_costs.push_back(cost);
  }

  void OnStepSizeAdjustmentIterationEnd(int iteration, const StatesType& xs,
                                        const ControlsType& us, int k_stepsize,
                                        double cost,
                                        const OptimizerInspector& oi) override {
    CHECK_EQ(iteration + 1, iterations.size());
    auto& iter = iterations.back();
    iter.k_s.push_back(k_stepsize);
    iter.stepsize_adjustment_costs.push_back(cost);
  }

  TrajectoryPoint av_pose_;
  absl::flat_hash_map<std::string, const SpacetimeObjectTrajectory*> object_map;

  StatesType solve_start_xs;
  ControlsType solve_start_us;

  StatesType solve_end_xs;
  ControlsType solve_end_us;

  struct CostDebugInfo {
    double cost;
    std::vector<NamedCostEntry> ddp_costs;
  };
  CostDebugInfo init_costs;
  CostDebugInfo final_costs;

  struct IterationDebugInfo {
    std::vector<double> alphas;
    std::vector<double> line_search_costs;
    std::vector<int> k_s;
    std::vector<double> stepsize_adjustment_costs;
    // The actual final cost.
    double final_cost;
    // The desired final cost.
    double js0;
  };
  std::vector<IterationDebugInfo> iterations;

  std::vector<ObjectResponseProto> object_responses;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_DEBUG_MONITOR_H_
