

#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_COST_MANAGER_MONITOR_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_COST_MANAGER_MONITOR_H_

#include <vector>

#include "plan_common/math/eigen.h"
#include "ddp_optimizer_monitor.h"
#include "planner/trajectory_optimizer/problem/cost.h"
#include "planner/trajectory_optimizer/problem/cost_helper.h"

namespace st {
namespace planning {

template <typename PROB>
class DdpCostManagerMonitor : public DdpOptimizerMonitor<PROB> {
 public:
  using OptimizerInspector =
      typename DdpOptimizerMonitor<PROB>::OptimizerInspector;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  explicit DdpCostManagerMonitor(int horizon)
      : DdpOptimizerMonitor<PROB>(horizon) {}

  void OnSolveStart(const StatesType& xs, const ControlsType& us,
                    const OptimizerInspector& oi) override {
    for (auto* helper : helpers_) {
      helper->Update(xs, us);
    }
    for (auto* cost : costs_) {
      cost->Update(xs, us, DdpOptimizerMonitor<PROB>::horizon());
    }
  }

  void OnIterationStart(int iter, const StatesType& xs, const ControlsType& us,
                        const OptimizerInspector& oi) override {
    for (auto* cost : costs_) {
      cost->UpdateDerivatives(xs, us, DdpOptimizerMonitor<PROB>::horizon());
    }
  }

  void OnLineSearchIterationStart(int iter, const StatesType& xs,
                                  const ControlsType& us,
                                  const StatesType& full_dxs,
                                  const ControlsType& full_dus, double alpha,
                                  const OptimizerInspector& oi) override {
    for (auto* helper : helpers_) {
      helper->Update(xs, us);
    }
    for (auto* cost : costs_) {
      cost->Update(xs, us, DdpOptimizerMonitor<PROB>::horizon());
    }
  }

  void OnStepSizeAdjustmentIterationStart(
      int iter, const StatesType& xs, const ControlsType& us, int k_stepsize,
      const OptimizerInspector& oi) override {
    for (auto* helper : helpers_) {
      helper->Update(xs, us);
    }
    for (auto* cost : costs_) {
      cost->Update(xs, us, DdpOptimizerMonitor<PROB>::horizon());
    }
  }

  void AddCost(Cost<PROB>* cost) {
    CHECK(cost != nullptr);
    costs_.push_back(cost);
  }
  void AddCostHelper(CostHelper<PROB>* helper) {
    CHECK(helper != nullptr);
    helpers_.push_back(helper);
  }

 private:
  std::vector<Cost<PROB>*> costs_;
  std::vector<CostHelper<PROB>*> helpers_;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_COST_MANAGER_MONITOR_H_
