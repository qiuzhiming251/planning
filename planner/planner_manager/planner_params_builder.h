

#ifndef ST_PLANNING_PLANNER_PARAMS_BUILDER
#define ST_PLANNING_PLANNER_PARAMS_BUILDER

#include "absl/status/statusor.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st {
namespace planning {

absl::StatusOr<PlannerParamsProto> BuildPlannerParams(
    const std::string& params_dir,
    const VehicleGeometryParamsProto& vehicle_geo_params);

}  // namespace planning
}  // namespace st

#endif  // ST_PLANNING_PLANNER_PARAMS_BUILDER
