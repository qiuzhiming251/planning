#ifndef ONBOARD_PLANNER_SPEED_OBJECT_SCENE_RECOGNITION_H_
#define ONBOARD_PLANNER_SPEED_OBJECT_SCENE_RECOGNITION_H_

#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "absl/status/status.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"
#include "planner/speed_optimizer/st_graph.h"

namespace st {
namespace planning {

struct DrivingProcess {
  double start_s = 0.0;
  double end_s = 0.0;
  ad_byd::planning::MergeTopology merge_topology =
      ad_byd::planning::MergeTopology::TOPOLOGY_MERGE_NONE;
  ad_byd::planning::SplitTopology split_topology =
      ad_byd::planning::SplitTopology::TOPOLOGY_SPLIT_NONE;
  LaneSemantic lane_semantic = LaneSemantic::NONE;
  mapping::ElementId lane_id = 0;
};

void MakeObjectSceneRecognition(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const DiscretizedPath& path, const SpacetimeTrajectoryManager& st_traj_mgr,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    std::vector<StBoundaryRef>* st_boundaries, double av_speed,
    std::vector<DrivingProcess>* driving_process_seq);

}  // namespace planning
}  // namespace st
#endif  // ONBOARD_PLANNER_SPEED_OBJECT_SCENE_RECOGNITION_H_
