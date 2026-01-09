

#ifndef ONBOARD_PLANNER_DECISION_CONSTRAINT_BUILDER_H_
#define ONBOARD_PLANNER_DECISION_CONSTRAINT_BUILDER_H_

#include "absl/status/statusor.h"
#include "object_manager/st_inference/decider_input.h"
#include "object_manager/st_inference/decider_output.h"

namespace st::planning {
absl::StatusOr<DeciderOutput> BuildConstraints(
    const DeciderInput& decider_input);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_DECISION_CONSTRAINT_BUILDER_H_
