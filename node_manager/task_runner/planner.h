#ifndef AD_BYD_PLANNING_NODES_PLANNER_PLANNER_H
#define AD_BYD_PLANNING_NODES_PLANNER_PLANNER_H
#include "plan_common/input_frame.h"
#include "plan_common/planning_macros.h"

namespace ad_byd {
namespace planning {

class Planner {
 public:
  DECLARE_PTR(Planner);

  virtual ~Planner() = default;

  virtual void Run(const PlanningInputFrame* input_frame) = 0;
  virtual void Reset() = 0;
  virtual std::shared_ptr<planning_result_type> GetPlanningResult() const = 0;
  virtual std::shared_ptr<debug_frame_type> GetDebugFrame() const = 0;
  virtual void ResetforAccEnterandExit() = 0;
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_NODES_PLANNER_PLANNER_H
