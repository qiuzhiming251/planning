

#ifndef ST_PLANNING_OBJECT_PLANNER_OBJECT
#define ST_PLANNING_OBJECT_PLANNER_OBJECT

#include <optional>
#include <string>
#include <vector>

#include "plan_common/math/geometry/aabox2d.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_object.pb.h"
#include "modules/cnoa_pnc/planning/proto/prediction.pb.h"
#include "plan_common/second_order_trajectory_point.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction.h"

namespace st {
namespace planning {

class PlannerObject {
 public:
  PlannerObject() = default;

  explicit PlannerObject(prediction::ObjectPrediction object_prediction);

  void ToPlannerObjectProto(PlannerObjectProto* planner_object_proto) const;

  const std::string& id() const { return object_proto_.id(); }

  int num_trajs() const { return prediction_.trajectories().size(); }

  const prediction::PredictedTrajectory& traj(int i) const {
    return prediction_.trajectories()[i];
  }

  const ObjectStopTimeProto& stop_time_info() const {
    return prediction_.stop_time();
  }

  const prediction::ObjectLongTermBehavior& long_term_behavior() const {
    return prediction_.long_term_behavior();
  }

  // Returns the most likely trajectory. Returns std::nullopt if the object has
  // no trajectory.
  std::optional<int> MostLikelyTrajectory() const;

  ObjectType type() const { return object_proto_.type(); }
  void set_type(ObjectType otype) { object_proto_.set_type(otype); }

  const ObjectProto& object_proto() const { return object_proto_; }

  bool is_frame_dropped() const { return object_proto_.is_frame_dropped(); }

  // TODO: Delete this API.
  // For object type OT_VEHICLE, OT_MOTORCYCLIST, OT_CYCLIST, and OT_TRICYCLIST,
  // its pose v and a stand for longtiduinal quantities (may be negative); for
  // other types, its pose v and a stand for absoluate values.
  const SecondOrderTrajectoryPoint& pose() const { return pose_; }

  Vec2d velocity() const { return velocity_; }

  const Polygon2d& contour() const { return contour_; }

  const AABox2d& aabox() const { return aabox_; }

  // TODO: Remove this mutable function.
  void set_stationary(bool stationary) { is_stationary_ = stationary; }

  bool is_stationary() const { return is_stationary_; }

  const prediction::ObjectPrediction& prediction() const { return prediction_; }
  prediction::ObjectPrediction* mutable_prediction() { return &prediction_; }

  const Box2d& bounding_box() const { return bounding_box_; }
  // The bounding box returned by perception. If not exist, it returns
  // bounding_box().
  const Box2d& perception_bbox() const { return perception_bbox_; }

  double proto_timestamp() const { return object_proto_.timestamp(); }

  bool is_sim_agent() const { return is_sim_agent_; }

  bool is_large_vehicle() const { return is_large_vehicle_; }
  // The id of the base object controlled by a simulation agent.
  const std::string& base_id() const { return base_id_; }

 private:
  void FromPrediction(const prediction::ObjectPrediction& prediction);

  bool is_stationary_ = false;
  SecondOrderTrajectoryPoint pose_;
  Vec2d velocity_;
  Polygon2d contour_;
  Box2d bounding_box_;
  Box2d perception_bbox_;
  AABox2d aabox_;
  prediction::ObjectPrediction prediction_;

  ObjectProto object_proto_;

  bool is_sim_agent_ = false;
  bool is_large_vehicle_ = false;
  std::string base_id_;

  friend class PlannerObjectBuilder;

  template <typename Archive>
  friend void serialize(Archive& ar, PlannerObject& planner_object);
};

}  // namespace planning
}  // namespace st

#endif  // ST_PLANNING_OBJECT_PLANNER_OBJECT
