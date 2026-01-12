#pragma once

#include <optional>

#include "absl/types/span.h"

#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "object_manager/object_history.h"
#include "plan_common/drive_passage.h"
#include "predictor/prediction_defs.h"

namespace st::planning {

ObjectProto LerpObject(const ObjectProto& o1, const ObjectProto& o2,
                       double factor);

std::optional<prediction::ObjectMotionHistory> ConvertToMotionHistory(
    const ObjectHistory& object_history);

double FitLateralSpeedByMotionHistory(
    absl::Span<const prediction::ObjectMotionState> history_states,
    const DrivePassage& drive_passage, int min_fit_num);

double FitLonAccelByMotionHistory(
    absl::Span<const prediction::ObjectMotionState> history_states,
    const DrivePassage& drive_passage, int min_fit_num);

}  // namespace st::planning
