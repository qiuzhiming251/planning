

#ifndef ONBOARD_PLANNER_OPTIMIZATION_IPOPT_OPTIMIZER_DEBUG_MONITOR_H_
#define ONBOARD_PLANNER_OPTIMIZATION_IPOPT_OPTIMIZER_DEBUG_MONITOR_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "plan_common/math/eigen.h"
#include "plan_common/math/vec.h"

namespace st {
namespace planning {

template <typename PROB>
class IpoptOptimizerDebugMonitor {
 public:
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  struct OptimizerInspector {
    double cost;  // Updated after iterations.
    absl::flat_hash_map<std::string, double>
        cost_map;  // Updated after iterations.
  };

  struct CostDebugInfo {
    double cost;
    std::vector<std::pair<std::string, double>> costs;
  };
  struct IterationDebugInfo {
    StatesType final_xs;
    ControlsType final_us;
    CostDebugInfo cost_info;
  };

  ~IpoptOptimizerDebugMonitor() {}
  void OnSolveStart(const StatesType& xs, const ControlsType& us,
                    const OptimizerInspector& oi) {
    solve_start_xs_ = xs;
    solve_start_us_ = us;
    iterations_.clear();
    init_costs_.costs.clear();
    final_costs_.costs.clear();
  }
  void OnSolveEnd(const StatesType& xs, const ControlsType& us,
                  const OptimizerInspector& oi) {
    solve_end_xs_ = xs;
    solve_end_us_ = us;

    final_costs_.cost = oi.cost;
    final_costs_.costs.clear();
    for (const auto& [cost_name, cost] : oi.cost_map) {
      // Don't log cost below this threshold.
      constexpr double kCostThresholdForLog = 1.0;
      if (cost < kCostThresholdForLog) continue;
      final_costs_.costs.emplace_back(cost_name, cost);
    }
  }
  void OnIterationEnd(int iter, const StatesType& xs, const ControlsType& us,
                      const OptimizerInspector& oi) {
    if (iter >= iterations_.size()) {
      iterations_.emplace_back();
    }
    iterations_.back().final_xs = xs;
    iterations_.back().final_us = us;
    iterations_.back().cost_info.cost = oi.cost;
    for (const auto& [cost_name, cost] : oi.cost_map) {
      // Don't log cost below this threshold.
      constexpr double kCostThresholdForLog = 1.0;
      if (cost < kCostThresholdForLog) continue;
      iterations_.back().cost_info.costs.emplace_back(cost_name, cost);
    }
  }

  const StatesType& solve_start_xs() const { return solve_start_xs_; }
  const ControlsType& solve_start_us() const { return solve_start_us_; }
  const StatesType& solve_end_xs() const { return solve_end_xs_; }
  const ControlsType& solve_end_us() const { return solve_end_us_; }
  const CostDebugInfo& init_costs() const { return init_costs_; }
  const CostDebugInfo& final_costs() const { return final_costs_; }
  const std::vector<IterationDebugInfo>& iterations() const {
    return iterations_;
  }

 private:
  StatesType solve_start_xs_;
  ControlsType solve_start_us_;

  StatesType solve_end_xs_;
  ControlsType solve_end_us_;

  CostDebugInfo init_costs_;
  CostDebugInfo final_costs_;

  std::vector<IterationDebugInfo> iterations_;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OPTIMIZATION_IPOPT_OPTIMIZER_DEBUG_MONITOR_H_
