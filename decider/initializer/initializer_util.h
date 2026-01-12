

#ifndef ONBOARD_PLANNER_INITIALIZER_INITIALIZER_UTIL_H_
#define ONBOARD_PLANNER_INITIALIZER_INITIALIZER_UTIL_H_

#include <map>
#include <string>
#include <vector>

#include "absl/types/span.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"

#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

#include "plan_common/drive_passage.h"

#include "object_manager/st_inference/initializer_output.h"

#include "decider/initializer/ref_speed_table.h"
#include "decider/initializer/motion_search_output.h"
#include "decider/initializer/geometry/geometry_graph.h"

namespace st::planning {

absl::StatusOr<std::vector<ApolloTrajectoryPointProto>>
GenerateConstLateralAccelConstSpeedTraj(
    const DrivePassage& drive_passage, double ego_front_to_ra, double target_l,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    absl::Span<const ConstraintProto::StopLineProto> stop_line,
    const ApolloTrajectoryPointProto& plan_start_point, int traj_steps);

absl::StatusOr<std::vector<ApolloTrajectoryPointProto>>
GenerateLatQuinticLonConstAccTrajToRefL(
    const DrivePassage& drive_passage, const PathSlBoundary& sl_boundary,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const std::vector<std::string>& leading_trajs,
    const ApolloTrajectoryPointProto& plan_start_point,
    const double ego_front_to_center,
    const std::optional<double> nearest_stop_s, const int plan_id,
    const std::string fake_reason);
void ParseMotionSearchOutputToMotionSearchDebugProto(
    const MotionSearchOutput& search_output, MotionSearchDebugProto* proto);

void ParseMotionSearchOutputToMultiTrajDebugProto(
    const MotionSearchOutput& search_output, MultiTrajDebugProto* proto);

void ParseMotionSearchOutputToInitializerResult(
    const MotionSearchOutput& search_output, InitializerDebugProto* proto);

InitializerOutput MakeAebInitializerOutput(
    ApolloTrajectoryPointProto plan_start_point,
    InitializerStateProto new_state, std::string message,
    InitializerDebugProto* debug_proto);

/**
 * @brief: Dumping expert trajectory raw feature cost and all searched DP
 * trajectories's raw feature costs.
 * **/
void ParseFeaturesDumpingProto(
    const MotionSearchOutput& search_output,
    ExpertEvaluationProto* expert_proto,
    SampledDpMotionEvaluationProto* candidates_proto);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_INITIALIZER_INITIALIZER_UTIL_H_
