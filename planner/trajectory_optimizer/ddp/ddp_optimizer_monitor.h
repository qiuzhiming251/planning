

#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_MONITOR_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_MONITOR_H_

#include <string>
#include <vector>

#include "plan_common/math/eigen.h"
#include "plan_common/math/vec.h"
#include "planner/trajectory_optimizer/problem/cost.h"

namespace st {
namespace planning {

template <typename PROB>
class DdpOptimizerMonitor {
 public:
  using StateType = typename PROB::StateType;
  using ControlType = typename PROB::ControlType;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  struct OptimizerInspector {
    double cost;  // Updated before OnLineSearchIterationEnd().
    double js0;   // Updated in OnIterationEnd().
    // Stores the name and value of costs.
    std::vector<NamedCostEntry> named_costs;
  };

  explicit DdpOptimizerMonitor(int horizon) : horizon_(horizon) {
    CHECK_GT(horizon_, 0);
  }

  virtual ~DdpOptimizerMonitor() {}
  virtual void OnSolveStart(const StatesType& xs, const ControlsType& us,
                            const OptimizerInspector& oi) {}
  virtual void OnSolveEnd(const StatesType& xs, const ControlsType& us,
                          const OptimizerInspector& oi) {}
  virtual void OnIterationStart(int iter, const StatesType& xs,
                                const ControlsType& us,
                                const OptimizerInspector& oi) {}
  virtual void OnIterationEnd(int iter, const StatesType& xs,
                              const ControlsType& us,
                              const OptimizerInspector& oi) {}
  virtual void OnLineSearchIterationStart(int iter, const StatesType& xs,
                                          const ControlsType& us,
                                          const StatesType& full_dxs,
                                          const ControlsType& full_dus,
                                          double alpha,
                                          const OptimizerInspector& oi) {}
  virtual void OnLineSearchIterationEnd(int iter, const StatesType& xs,
                                        const ControlsType& us,
                                        const StatesType& full_dxs,
                                        const ControlsType& full_dus,
                                        double alpha, double cost,
                                        const OptimizerInspector& oi) {}
  virtual void OnStepSizeAdjustmentIterationStart(
      int iter, const StatesType& xs, const ControlsType& us, int k_stepsize,
      const OptimizerInspector& oi) {}
  virtual void OnStepSizeAdjustmentIterationEnd(int iter, const StatesType& xs,
                                                const ControlsType& us,
                                                int k_stepsize, double cost,
                                                const OptimizerInspector& oi) {}

  int horizon() const { return horizon_; }

 private:
  int horizon_ = 0;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_MONITOR_H_
