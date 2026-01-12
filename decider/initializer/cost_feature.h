

#ifndef ONBOARD_PLANNER_INITIALIZER_COST_FEATURE_H_
#define ONBOARD_PLANNER_INITIALIZER_COST_FEATURE_H_

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "plan_common/constraint_manager.h"
#include "decider/initializer/geometry/geometry_graph.h"
#include "decider/initializer/initializer_input.h"
#include "decider/initializer/motion_graph.h"
#include "decider/initializer/ref_speed_table.h"
#include "modules/cnoa_pnc/planning/proto/initializer_config.pb.h"

namespace st::planning {

struct GeometryEdgeInfo {
  const GeometryForm* geometry_form = nullptr;
  std::vector<GeometryState> states;
  bool terminating = false;
};

struct MotionEdgeInfo {
  double start_t = 0.0;  // Start time of the motion.
  const MotionForm* motion_form = nullptr;
  std::vector<MotionState> const_interval_states{};
  std::vector<MotionState> equal_interval_states{};
};

class FeatureCost {
 public:
  explicit FeatureCost(std::string name) : name_(std::move(name)) {}

  virtual void ComputeCost(const MotionEdgeInfo& edge_info,
                           absl::Span<double> cost) const {}

  virtual void ComputeCost(const GeometryEdgeInfo& edge_info,
                           absl::Span<double> cost) const {}

  const std::string& name() const { return name_; }

  virtual ~FeatureCost() {}

 private:
  std::string name_ = "";
};

}  // namespace st::planning
#endif  // ONBOARD_PLANNER_INITIALIZER_COST_FEATURE_H_
