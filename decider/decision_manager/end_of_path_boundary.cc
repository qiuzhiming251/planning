

#include <algorithm>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "decider/decision_manager/end_of_path_boundary.h"
#include "plan_common/math/geometry/halfplane.h"

namespace st {
namespace planning {

// This function returns end of path boundary constraint.
absl::StatusOr<ConstraintProto::StopLineProto> BuildEndOfPathBoundaryConstraint(
    const DrivePassage& passage, const PathSlBoundary& path_boundary) {
  const double end_of_path_boundary =
      std::min(passage.end_s(), path_boundary.end_s());
  const auto curbs = passage.QueryCurbPointAtS(end_of_path_boundary);
  if (!curbs.ok()) {
    return absl::NotFoundError(absl::StrFormat(
        "Curb boundaries not found at s=%.2f.", end_of_path_boundary));
  }
  const HalfPlane halfplane(curbs->first, curbs->second);

  ConstraintProto::StopLineProto end_of_path_boundary_constraint;
  end_of_path_boundary_constraint.set_s(end_of_path_boundary);
  end_of_path_boundary_constraint.set_standoff(0.0);
  end_of_path_boundary_constraint.set_time(0.0);
  halfplane.ToProto(end_of_path_boundary_constraint.mutable_half_plane());
  end_of_path_boundary_constraint.set_id("end_of_path_boundary");
  end_of_path_boundary_constraint.mutable_source()
      ->mutable_end_of_path_boundary()
      ->set_id("");
  return end_of_path_boundary_constraint;
}

}  // namespace planning
}  // namespace st
