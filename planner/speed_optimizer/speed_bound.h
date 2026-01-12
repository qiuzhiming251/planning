

#ifndef ONBOARD_PLANNER_SPEED_SPEED_BOUND_H_
#define ONBOARD_PLANNER_SPEED_SPEED_BOUND_H_

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"

namespace st::planning {

struct SpeedBoundWithInfo {
  double bound = 0.0;
  std::string info;
};

struct AccelBounds {
  double lower_bound = 0.0;
  double upper_bound = 0.0;
  std::optional<double> soft_lower_bound;
  std::optional<double> soft_upper_bound;
};

using SpeedBoundMapType =
    std::map<SpeedLimitTypeProto::Type, std::vector<SpeedBoundWithInfo>>;

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SPEED_SPEED_BOUND_H_
