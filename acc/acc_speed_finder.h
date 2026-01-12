#pragma once

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "acc/acc_speed_finder_input.h"
#include "acc/acc_speed_finder_output.h"

namespace st::planning {

absl::StatusOr<AccSpeedFinderOutput> FindAccSpeed(
    const AccSpeedFinderInput& input);

}  // namespace st::planning
