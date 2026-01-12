#include "object_manager/object_history_util.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "plan_common/math/line_fitter.h"
#include "plan_common/util/path_util.h"

namespace st::planning {

constexpr double kMinFitTime = 0.5;  // s

ObjectProto LerpObject(const ObjectProto& o1, const ObjectProto& o2,
                       double factor) {
  const double ts = Lerp(o1.timestamp(), o2.timestamp(), factor);
  const Vec2d pos =
      Lerp(Vec2dFromProto(o1.pos()), Vec2dFromProto(o2.pos()), factor);
  const Vec2d vel =
      Lerp(Vec2dFromProto(o1.vel()), Vec2dFromProto(o2.vel()), factor);
  const Vec2d accel =
      Lerp(Vec2dFromProto(o1.accel()), Vec2dFromProto(o2.accel()), factor);
  const double yaw = NormalizeAngle(LerpAngle(o1.yaw(), o2.yaw(), factor));
  const double yaw_rate =
      NormalizeAngle(LerpAngle(o1.yaw_rate(), o2.yaw_rate(), factor));
  ObjectProto obj = o1;
  obj.set_timestamp(ts);
  pos.ToProto(obj.mutable_pos());
  vel.ToProto(obj.mutable_vel());
  accel.ToProto(obj.mutable_accel());
  obj.set_yaw(yaw);
  obj.set_yaw_rate(yaw_rate);
  return obj;
}

std::optional<prediction::ObjectMotionHistory> ConvertToMotionHistory(
    const ObjectHistory& object_history) {
  std::vector<ObjectProto> object_proto_history;
  std::vector<double> timestamps;

  const ObjectFrame* latest_object = object_history.GetLatestFrame();
  const auto& frames = object_history.GetFrames();

  if (latest_object == nullptr || frames.size() < 2) {
    return std::nullopt;
  }
  object_proto_history.reserve(frames.size());
  timestamps.reserve(frames.size());
  double prev_timestamp = 0.0;
  for (const auto& frame : frames) {
    const double curr_frame_ts = frame.object_proto.timestamp();
    if (curr_frame_ts - prev_timestamp < 1e-3) {
      continue;
    }
    prev_timestamp = curr_frame_ts;
    object_proto_history.push_back(frame.object_proto);
    timestamps.push_back(curr_frame_ts);
  }
  if (timestamps.size() < 2) {
    return std::nullopt;
  }

  class Lerper {
   public:
    ObjectProto operator()(const ObjectProto& o1, const ObjectProto& o2,
                           double factor) const {
      return LerpObject(o1, o2, factor);
    }
  };
  PiecewiseLinearFunction<ObjectProto, double, Lerper> object_proto_plf(
      timestamps, object_proto_history);
  constexpr double kSampleTimeStep = 0.1;  // s.
  constexpr int kMaxSampleNum = 20;

  const int sample_num = std::clamp(
      FloorToInt((timestamps.back() - timestamps.front()) / kSampleTimeStep) +
          1,
      1, kMaxSampleNum);

  std::vector<ObjectProto> resampled_objects;
  resampled_objects.reserve(sample_num);
  const double current_ts = timestamps.back();
  for (int i = 0; i < sample_num - 1; ++i) {
    const double ts = current_ts + (i - sample_num + 1) * kSampleTimeStep;
    auto resampled_obj = object_proto_plf.EvaluateWithExtrapolation(ts);
    resampled_objects.push_back(std::move(resampled_obj));
  }
  resampled_objects.push_back(latest_object->object_proto);

  prediction::ObjectMotionHistory object_motion_history;
  object_motion_history.id = latest_object->id;
  object_motion_history.type = latest_object->object_proto.type();
  const auto& bbox = latest_object->object_proto.bounding_box();
  for (const auto& obj : resampled_objects) {
    const auto pos = Vec2dFromProto(obj.pos());
    const auto yaw = obj.yaw();
    object_motion_history.states.push_back(
        {.timestamp = obj.timestamp(),
         .pos = pos,
         .heading = yaw,
         .vel = Vec2dFromProto(obj.vel()),
         .bbox = Box2d(pos, yaw, bbox.length(), bbox.width())});
  }
  return object_motion_history;
}

double FitLateralSpeedByMotionHistory(
    absl::Span<const prediction::ObjectMotionState> history_states,
    const DrivePassage& drive_passage, int min_fit_num) {
  if (min_fit_num < 2 || history_states.size() < min_fit_num) {
    return 0.0;
  }
  std::vector<Vec2d> data;
  std::vector<double> weights;
  data.reserve(history_states.size());
  weights.reserve(history_states.size());
  const double latest_ts = history_states.back().timestamp;
  const double fit_time = latest_ts - history_states.front().timestamp;
  for (const auto& state : history_states) {
    const double relative_time = state.timestamp - latest_ts;
    const auto frenet_sl_or = drive_passage.QueryFrenetCoordinateAt(state.pos);
    if (!frenet_sl_or.ok()) continue;
    data.push_back(Vec2d(relative_time, frenet_sl_or->l));
    const double normalized_fit_time = relative_time / fit_time;
    weights.push_back(std::exp(normalized_fit_time));
  }
  if (data.size() < min_fit_num || data.front().x() > -kMinFitTime) {
    return 0.0;
  }
  auto line_fitter = LineFitter(data, weights);
  line_fitter.FitData();
  const auto tangent = line_fitter.tangent();
  if (std::fabs(tangent.x()) < 1e-3) return 0.0;
  return tangent.y() / tangent.x();
}

double FitLonAccelByMotionHistory(
    absl::Span<const prediction::ObjectMotionState> history_states,
    const DrivePassage& drive_passage, int min_fit_num) {
  if (min_fit_num < 2 || history_states.size() < min_fit_num) {
    return 0.0;
  }
  std::vector<Vec2d> data;
  std::vector<double> weights;
  data.reserve(history_states.size());
  weights.reserve(history_states.size());
  const double latest_ts = history_states.back().timestamp;
  const double fit_time = latest_ts - history_states.front().timestamp;
  for (const auto& state : history_states) {
    const double relative_time = state.timestamp - latest_ts;
    const auto tangent = drive_passage.QueryTangentAt(state.pos);
    if (!tangent.ok()) continue;
    const auto lon_vel = tangent->Dot(state.vel);
    data.push_back(Vec2d(relative_time, lon_vel));
    const double normalized_fit_time = relative_time / fit_time;
    weights.emplace_back(std::exp(normalized_fit_time));
  }
  if (data.size() < min_fit_num || data.front().x() > -kMinFitTime) {
    return 0.0;
  }
  auto line_fitter = LineFitter(data, weights);
  line_fitter.FitData();
  const auto tangent = line_fitter.tangent();
  if (std::fabs(tangent.x()) < 1e-3) return 0.0;
  return tangent.y() / tangent.x();
}

}  // namespace st::planning
