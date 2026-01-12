

#ifndef ONBOARD_PLANNER_INITIALIZER_MOTION_SEARCH_UTIL_H_
#define ONBOARD_PLANNER_INITIALIZER_MOTION_SEARCH_UTIL_H_

#include <vector>

#include "decider/initializer/motion_form.h"
#include "decider/initializer/motion_graph.h"
#include "decider/initializer/motion_state.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/drive_passage.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

namespace st::planning {

constexpr double kEpsilon = 1e-6;

ApolloTrajectoryPointProto MotionState2TrajPoint(
    const MotionState& motion_state, double s, double current_t);

void ResampleTrajectoryPoints(
    const std::vector<const MotionForm*>& motions,
    std::vector<ApolloTrajectoryPointProto>& traj_points);

std::vector<ApolloTrajectoryPointProto> ConstructStationaryTraj(
    const MotionState& sdc_motion);

std::vector<ApolloTrajectoryPointProto> ConstructTrajFromLastEdge(
    const MotionGraph& motion_graph, MotionEdgeIndex last_edge_index);

double GetLeadingObjectsEndMinS(const SpacetimeTrajectoryManager& st_mgr,
                                const DrivePassage& drive_passage,
                                const std::vector<std::string>& leading_objs,
                                double sdc_length);

MotionState PrepareStartMotionNode(
    const GeometryGraph& geometry,
    const std::vector<GeometryNodeIndex>& first_layer,
    const ApolloTrajectoryPointProto& start_point,
    int* start_node_idx_on_first_layer);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_INITIALIZER_MOTION_SEARCH_UTIL_H_
