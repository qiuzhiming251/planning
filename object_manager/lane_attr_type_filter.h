#ifndef ONBOARD_PLANNER_OBJECT_LANE_ATTR_TYPE_FILTER_H_
#define ONBOARD_PLANNER_OBJECT_LANE_ATTR_TYPE_FILTER_H_

//#include "lite/check.h"
#include "plan_common/path_sl_boundary.h"
#include "modules/cnoa_pnc/planning/proto/planner_object.pb.h"
#include "planner_object.h"
#include "trajectory_filter.h"
#include "predictor/predicted_trajectory.h"
#include "plan_common/drive_passage.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st {
namespace planning {

class LaneAttrTypeFilter : public TrajectoryFilter {
 public:
  explicit LaneAttrTypeFilter(const PoseProto& pose,
                              const VehicleGeometryParamsProto& vehicle_geom,
                              const LaneChangeStateProto pre_lc_state,
                              const bool& is_on_highway)
      : lc_state_(pre_lc_state), is_on_highway_(is_on_highway) {
    pos_ = Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y());
    tangent_ = Vec2d::FastUnitFromAngle(pose.yaw());
    back_pos_ = pos_ - tangent_ * vehicle_geom.back_edge_to_center();
  }

  FilterReason::Type Filter(
      const PlannerObject& object,
      const prediction::PredictedTrajectory& traj) const override;

 private:
  LaneChangeStateProto lc_state_;
  bool is_on_highway_;
  Vec2d pos_;
  Vec2d tangent_;
  Vec2d back_pos_;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OBJECT_DRIVE_PASSAGE_FILTER_H_
