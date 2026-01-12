

#include <algorithm>

#include "predictor/prediction_object_state.h"
#include "predictor/prediction_util.h"

namespace st {
namespace prediction {

std::vector<PredictionObjectState> SampleTrajectoryStates(
    const PredictedTrajectory& pred_traj, const Vec2d& init_pos,
    const Polygon2d& init_contour, const Box2d& init_box) {
  const auto& traj_points = pred_traj.points();
  if (traj_points.empty()) return {};
  const int num_traj_points = traj_points.size();

  std::vector<PredictionObjectState> states;
  states.reserve(num_traj_points);

  if (IsStationaryTrajectory(pred_traj)) {
    // Spare the spatial transformations below.
    for (int i = 0; i < num_traj_points; ++i) {
      states.push_back(
          std::move(PredictionObjectState{.traj_point = &traj_points[i],
                                          .box = init_box,
                                          .contour = init_contour}));
    }
    return states;
  }

  // Note that besides transforming the object contour polygon, we also need to
  // transform the object's bounding box center as box's center may not be at
  // the object's pos.
  const Vec2d box_pos_shift = init_box.center() - init_pos;
  for (int i = 0; i < num_traj_points; ++i) {
    PredictionObjectState& obj_state = states.emplace_back();
    const auto& pt = traj_points[i];
    obj_state.traj_point = &pt;
    const Vec2d rotation =
        Vec2d::FastUnitFromAngle(pt.theta() - init_box.heading());
    obj_state.contour = init_contour.Transform(
        init_pos, rotation.x(), rotation.y(), pt.pos() - init_pos);

    obj_state.box = Box2d(pt.pos() + box_pos_shift, pt.theta(),
                          init_box.length(), init_box.width());
  }

  return states;
}

bool IsVulnerableRoadUserType(ObjectType type) {
  return type == OT_PEDESTRIAN || type == OT_CYCLIST || type == OT_TRICYCLIST ||
         type == OT_MOTORCYCLIST;
}

bool IsStaticObjectType(ObjectType type) {
  return type == OT_UNKNOWN_STATIC || type == OT_VEGETATION || type == OT_FOD ||
         type == OT_BARRIER || type == OT_CONE;
}

}  // namespace prediction
}  // namespace st
