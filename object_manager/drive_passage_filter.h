

#ifndef ONBOARD_PLANNER_OBJECT_DRIVE_PASSAGE_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_DRIVE_PASSAGE_FILTER_H_

//#include "lite/check.h"
#include "plan_common/path_sl_boundary.h"
#include "modules/cnoa_pnc/planning/proto/planner_object.pb.h"
#include "planner_object.h"
#include "trajectory_filter.h"
#include "predictor/predicted_trajectory.h"
#include "plan_common/drive_passage.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "plan_common/maps/planner_semantic_map_manager.h"

namespace st {
namespace planning {

double GetDistanceBuffer(const PlannerObject& object);
bool IsContourWithBufferOverlapsBoundary(
    const Polygon2d& contour, const double buffer,
    const PathSlBoundary& sl_boundary, const DrivePassage& drive_passage,
    const Box2d& ego_box, const LaneChangeStateProto& lc_state,
    const bool use_out_boundary = true, const bool use_decay_buffer = false,
    const double* ego_v = nullptr);

class DrivePassageFilter : public TrajectoryFilter {
 public:
  explicit DrivePassageFilter(const DrivePassage* drive_passage,
                              const PathSlBoundary* sl_boundary,
                              const LaneChangeStateProto* lc_state,
                              const Box2d& ego_box, const bool is_on_highway,
                              const PlannerSemanticMapManager* psmm)
      : drive_passage_(CHECK_NOTNULL(drive_passage)),
        sl_boundary_(CHECK_NOTNULL(sl_boundary)),
        lc_state_(CHECK_NOTNULL(lc_state)),
        ego_box_(ego_box),
        is_on_highway_(is_on_highway),
        psmm_(psmm) {}

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;

 private:
  const DrivePassage* drive_passage_;
  const PathSlBoundary* sl_boundary_;
  const LaneChangeStateProto* lc_state_;
  const Box2d ego_box_;
  const bool is_on_highway_ = false;
  const PlannerSemanticMapManager* psmm_;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OBJECT_DRIVE_PASSAGE_FILTER_H_
