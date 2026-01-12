#include "planner/speed_optimizer/decider/speed_limit_decider.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "plan_common/log_data.h"
#include "plan_common/log.h"

//#include "global/buffered_logger.h"
//#include "lite/check.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/path_util.h"

namespace st {
namespace planning {

namespace {}  // namespace

void LaneChangeSpeedDecider(const double& av_speed, const double& max_acc,
                            OpenLoopSpeedLimit* open_loop_speed_limit) {
  if (av_speed < Kph2Mps(30.0)) {
    if (av_speed < Kph2Mps(20.0)) {
      double a_limit = Lerp(0.5, Kph2Mps(10.0), std::fmin(max_acc, 1.0),
                            Kph2Mps(20.0), av_speed, true);
      if (av_speed < 1e-2) {
        a_limit = std::fmax(a_limit, 1.0);
      }
      open_loop_speed_limit->AddALimit(a_limit, std::nullopt,
                                       "20.0 low speed lane change");
    } else {
      double a_limit = Lerp(std::fmin(max_acc, 1.0), Kph2Mps(20.0), max_acc,
                            Kph2Mps(30.0), av_speed, true);
      open_loop_speed_limit->AddALimit(a_limit, std::nullopt,
                                       "30.0 low speed lane change");
    }
  }
  return;
}

}  // namespace planning
}  // namespace st
