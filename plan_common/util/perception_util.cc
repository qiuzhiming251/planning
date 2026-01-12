

#include "plan_common/util/perception_util.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"

//#include "lite/check.h"
//#include "lite/logging.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/affine_transformation.pb.h"
#include "modules/cnoa_pnc/planning/proto/box2d.pb.h"
#include "plan_common/util/vehicle_geometry_util.h"

namespace st {
namespace planning {

namespace {
std::vector<Vec2d> TransformPoints(
    const ::google::protobuf::RepeatedPtrField<Vec2dProto>& proto_points,
    const Vec2d& origin_center, const Vec2d& shift, double cos_angle,
    double sin_angle) {
  std::vector<Vec2d> points;
  points.reserve(proto_points.size());
  const Vec2d new_center = origin_center + shift;
  for (const auto& p : proto_points) {
    const Vec2d point = Vec2dFromProto(p) - origin_center;
    points.emplace_back(point.Rotate(cos_angle, sin_angle) + new_center);
  }
  return points;
}
}  // namespace

bool IsLargeVehicle(const ObjectProto& object) {
  constexpr double kLargeVehicleLength = 7.0;  // m.
  // constexpr double kTruckLargeVehicleLength = 6.0;  // m.
  constexpr double kLargeVehicleWidth = 3.0;   // m.
  constexpr double kLargeVehicleHeight = 3.0;  // m.

  const double length = object.bounding_box().length();
  const double width = object.bounding_box().width();
  const double height = object.max_z() - object.ground_z();
  return length > kLargeVehicleLength ||
         (width > kLargeVehicleWidth && height > kLargeVehicleHeight) ||
         object.type() == OT_LARGE_VEHICLE;
}

Polygon2d ComputeObjectContour(const ObjectProto& object_proto) {
  int contour_size = object_proto.contour_size();
  std::vector<Vec2d> vertices;
  vertices.reserve(contour_size);
  for (int j = 0; j < contour_size; ++j) {
    vertices.push_back(Vec2dFromProto(object_proto.contour(j)));
  }
  CHECK_GT(vertices.size(), 2);
  return Polygon2d(std::move(vertices), /*is_convex=*/true);
}

ObjectProto AvPoseProtoToObjectProto(
    const std::string& object_id,
    const VehicleGeometryParamsProto& vehicle_geom, const PoseProto& pose,
    bool offroad) {
  ObjectProto object;
  object.set_id(object_id);
  object.set_type(ObjectType::OT_VEHICLE);

  const Box2d box = ComputeAvBox(
      /*av_pos=*/Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()),
      pose.yaw(), vehicle_geom);
  object.mutable_pos()->set_x(box.center().x());
  object.mutable_pos()->set_y(box.center().y());

  object.set_yaw(pose.yaw());
  // Estimate yaw_rate by curvature * speed.
  // object.set_yaw_rate(pose.curvature() * pose.speed());

  object.mutable_vel()->set_x(pose.vel_smooth().x());
  object.mutable_vel()->set_y(pose.vel_smooth().y());

  object.mutable_accel()->set_x(pose.accel_smooth().x());
  object.mutable_accel()->set_y(pose.accel_smooth().y());

  for (const auto& pt : box.GetCornersCounterClockwise()) {
    pt.ToProto(object.add_contour());
  }

  box.ToProto(object.mutable_bounding_box());

  object.set_parked(false);
  // object.set_offroad(offroad);

  // Set a very long life time.
  // constexpr double kAvDefaultLifeTime =
  //     std::numeric_limits<double>::max();  // Seconds.
  // object.set_life_time(kAvDefaultLifeTime);
  // constexpr double kMinSpeed = 0.2;  // Meters per second.
  // object.set_moving_state(std::abs(Vec2d(object.vel()).norm()) < kMinSpeed
  //                             ? ObjectProto::MS_STATIC
  //                             : ObjectProto::MS_MOVING);
  object.set_min_z(pose.pos_smooth().z());
  object.set_max_z(pose.pos_smooth().z() + vehicle_geom.height());

  if (pose.has_timestamp()) {
    object.set_timestamp(pose.timestamp());
  } else {
    LOG_INFO << absl::StrFormat("I do not find any timestamp for the av!");
  }
  return object;
}

// bool IsLidarObject(const ObjectProto& object) {
//   if (!object.has_measurement_source_type()) {
//     return false;
//   }
//   return (TMST_LO == object.measurement_source_type()) ||
//          (TMST_LV == object.measurement_source_type()) ||
//          (TMST_LR == object.measurement_source_type()) ||
//          (TMST_LVR == object.measurement_source_type());
// }

bool IsCameraObject(const ObjectProto& object) {
  return true;
  // if (!object.has_measurement_source_type()) {
  //   return false;
  // }
  // return (TMST_VO == object.measurement_source_type()) ||
  //        (TMST_VR == object.measurement_source_type());
}

// bool IsOccludedLidarObject(const ObjectProto& object) {
//   if (!object.has_measurement_source_type() ||
//       !object.has_observation_state()) {
//     return false;
//   }
//   return (OS_PARTIALLY_OCCLUDED == object.observation_state() ||
//           OS_PARTIALLY_OBSERVED == object.observation_state()) &&
//          IsLidarObject(object);
// }

// bool IsOccludedCameraObject(const ObjectProto& object) {
//   if (!object.has_measurement_source_type() ||
//       !object.has_observation_state()) {
//     return false;
//   }
//   return (OS_PARTIALLY_OCCLUDED == object.observation_state()) &&
//          IsCameraObject(object);
// }

absl::Status AlignPerceptionObjectTime(double current_time,
                                       ObjectProto* object) {
  double time_diff = current_time - object->timestamp();
  constexpr double kMaxTimeDiff = 5.0;  // Seconds.
  if (time_diff < 0.0) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Provided time[%f] is less than perception's time[%f]",
                        current_time, object->timestamp()));
  }

  if (time_diff > kMaxTimeDiff) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Current time[%f] is too different with perception "
                        "time[%f], larger than threshold[%f]",
                        current_time, object->timestamp(), kMaxTimeDiff));
  }

  // set timestamp.
  object->set_timestamp(current_time);
  // NOTE: decide which yaw to use according to speed.
  // Actually should be done by perception...
  // constexpr double kMinSpeedToEnableAlignment = 2.0;  // m/s
  // if (object->moving_state() == ObjectProto::MS_STATIC ||
  //     Vec2dFromProto(object->vel()).squaredNorm() <
  //         Sqr(kMinSpeedToEnableAlignment)) {
  //   object->set_yaw(object->bounding_box().heading());
  //   return absl::OkStatus();
  // }

  const Vec2d vel = Vec2dFromProto(object->vel());
  const Vec2d pos = Vec2dFromProto(object->pos());
  const Vec2d accel = Vec2dFromProto(object->accel());

  // Update values assuming const acceleration.
  Vec2d new_vel = vel + accel * time_diff;
  if (new_vel.Dot(vel) < 0) {
    new_vel = Vec2d(0.0, 0.0);
    time_diff = std::fabs(vel.Length() / accel.Length());  // time to stop
  }
  const Vec2d pos_shift = (vel + 0.5 * accel * time_diff) * time_diff;
  const double yaw_diff = NormalizeAngle(object->yaw_rate() * time_diff);
  const double new_yaw = NormalizeAngle(yaw_diff + object->yaw());
  const Vec2d new_pos = pos_shift + pos;

  // Set pos.
  new_pos.ToProto(object->mutable_pos());

  // Set velocity.
  new_vel.ToProto(object->mutable_vel());

  // Set contour.
  const Vec2d rotation = Vec2d::FastUnitFromAngle(yaw_diff);
  const auto contour_points =
      TransformPoints(object->contour(), /*origin_center=*/pos,
                      /*shift=*/pos_shift, rotation.x(), rotation.y());
  object->clear_contour();
  for (const auto& pt : contour_points) {
    pt.ToProto(object->add_contour());
  }

  // Set bounding box. Angle normalization is done inside transform function.
  object->mutable_bounding_box()->set_x(new_pos.x());
  object->mutable_bounding_box()->set_y(new_pos.y());
  object->mutable_bounding_box()->set_heading(new_yaw);

  // Set yaw.
  object->set_yaw(NormalizeAngle(object->yaw() + yaw_diff));

  return absl::OkStatus();
}

}  // namespace planning
}  // namespace st
