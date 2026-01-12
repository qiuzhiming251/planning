

#ifndef ONBOARD_PLANNER_DECISION_DECIDER_OUTPUT_H_
#define ONBOARD_PLANNER_DECISION_DECIDER_OUTPUT_H_

#include <optional>
#include <vector>

#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/traffic_light_info.pb.h"

#include "plan_common/type_def.h"
#include "plan_common/constraint_manager.h"

namespace st::planning {

struct DeciderOutput {
  ConstraintManager constraint_manager;
  DeciderStateProto decider_state;
  std::optional<double> distance_to_traffic_light_stop_line = std::nullopt;
  int tl_stop_interface = 0;
  ad_byd::planning::SpeedState speed_state;
  st::planning::TrafficLightIndicationInfoProto tl_ind_info;
};

}  // namespace st::planning

#endif
