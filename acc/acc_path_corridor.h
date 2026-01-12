#pragma once

#include <memory>

#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/planner_status.h"

#include "modules/cnoa_pnc/planning/proto/acc.pb.h"

namespace st::planning {

// Much like Drivepassage
struct AccPathCorridor {
  PlannerStatus build_status{};
  AccPathCorridorType type = AccPathCorridorType::MAP;
  PathSlBoundary boundary{};
  std::unique_ptr<FrenetFrame> frenet_frame = nullptr;
  DiscretizedPath path{};  // smoothed
  double loaded_map_dist = 0.0;
  PiecewiseLinearFunction<double> kappa_s{};

  std::string DebugString() const;
};

AccPathCorridor BuildErrorAccPathCorridorResult(std::string_view error_msg);

}  // namespace st::planning
