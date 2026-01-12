

#ifndef ONBOARD_PLANNER_UTIL_PERCEPTION_UTIL_H_
#define ONBOARD_PLANNER_UTIL_PERCEPTION_UTIL_H_

#include <string>
#include "absl/status/status.h"

#include "plan_common/math/geometry/polygon2d.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/positioning.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st {
namespace planning {

inline bool IsConsiderMirrorObject(const ObjectProto& object_proto,
                                   double min_mirror_height_avg,
                                   double max_mirror_height_avg) {
  if (!object_proto.has_min_z() || !object_proto.has_max_z() ||
      !object_proto.has_ground_z()) {
    return true;
  }
  const double object_max_height =
      object_proto.max_z() - object_proto.ground_z();
  const double object_min_height =
      object_proto.min_z() - object_proto.ground_z();
  return object_max_height > min_mirror_height_avg &&
         object_min_height < max_mirror_height_avg;
}

inline bool IsVehicle(ObjectType type) {
  return (type == ObjectType::OT_VEHICLE ||
          type == ObjectType::OT_LARGE_VEHICLE);
}

// For recognizing large vehicle (e.g. bus, truck, etc.).
bool IsLargeVehicle(const ObjectProto& object);

//  Returns the contour of a perception object.
Polygon2d ComputeObjectContour(const ObjectProto& object);

// Returns object proto representation of AV.
ObjectProto AvPoseProtoToObjectProto(
    const std::string& object_id,
    const VehicleGeometryParamsProto& vehicle_geom, const PoseProto& pose,
    bool offroad);

// bool IsLidarObject(const ObjectProto& object);

bool IsCameraObject(const ObjectProto& object);

// bool IsOccludedLidarObject(const ObjectProto& object);

// bool IsOccludedCameraObject(const ObjectProto& object);

// Align an object's timestamp to current time. It assumes the object moves
// along its current direction with constant acceleration. If the provided
// current time is too different than perception's time, an error message will
// be returned.
absl::Status AlignPerceptionObjectTime(double current_time,
                                       ObjectProto* object);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_UTIL_PERCEPTION_UTIL_H_
