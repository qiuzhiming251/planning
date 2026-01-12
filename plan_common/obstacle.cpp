#include "obstacle.h"

#include <memory>
#include <vector>

#include "plan_common/math/angle.h"
#include "plan_common/math/double.h"

namespace ad_byd {
namespace planning {

Obstacle::Obstacle(const int32_t history_num)
    : obs_history_frames_(history_num) {}

void Obstacle::InsertFrame(const DynamicObstacleInfo& dyn_obs,
                           const ObstacleFrame* const adc_frame) {
  // 构造ObstacleFrame, insert 进Obstacle
  ObstacleFrame obs_frame;
  const MathVec2d& pos = dyn_obs.pos;
  double dist_to_adc = 0.0;
  double theta_to_adc = 0.0;
  if (adc_frame != nullptr) {
    obs_frame.SetAdcPos(pos);
    dist_to_adc = (adc_frame->position() - pos).Length();
    theta_to_adc = (pos - adc_frame->position()).Angle();
  } else {
    obs_frame.SetAdcPos(pos);
  }
  obs_frame.SetDistToAdc(dist_to_adc);
  obs_frame.SetThetaToAdc(theta_to_adc);
  obs_frame.SetId(dyn_obs.id);
  obs_frame.SetType(dyn_obs.type);
  obs_frame.SetSuperByType(&obs_frame);
  obs_frame.SetTimestamp(dyn_obs.stamp);
  obs_frame.SetPosition(dyn_obs.pos);
  obs_frame.SetHeading(dyn_obs.heading);
  obs_frame.SetVelocity(dyn_obs.vel);
  obs_frame.SetLength(dyn_obs.length);
  obs_frame.SetWidth(dyn_obs.width);
  obs_frame.SetHeight(dyn_obs.height);
  obs_frame.SetSeqNum(dyn_obs.seq_num);
  obs_frame.SetSpeed(obs_frame.velocity().Length());
  obs_frame.SetVecAcc(Vec2d(dyn_obs.acc.x(), dyn_obs.acc.y()));
  obs_frame.SetIsStatic(false);
  obs_frame.SetFusionType(dyn_obs.fusion_type);
  obs_frame.SetObstacleLight(dyn_obs.obstacle_lights);
  is_static_ = false;
  bool is_motion = true;
  // obs_frame.SetYawRate(dyn_obs.yaw_rate);
  if (dyn_obs.obstacle_state & 0x01 ||
      math::Double::Compare(obs_frame.speed(), 0.0) ==
          math::Double::CompareType::EQUAL) {
    is_motion = false;
  }
  obs_frame.SetIsMotion(is_motion);

  if (obs_history_frames_.size() < 2) {
    VectorPoint vp;
    vp.start_pt = Vec2d(obs_frame.position().x(), obs_frame.position().y());
    vp.end_pt = Vec2d(0.0, 0.0);
    vp.heading = 0.0;
    vp.length = 0.0;
    // (pos, pos, PolylineType::OBSTACLE, "");
    obs_frame.SetVecPoint(vp);
    obs_frame.SetScalarAcc(0.0);
  } else {
    const ObstacleFrame* const last_frame =
        this->GetFrameAt(1);  // skip 1 step, TODO: using gflag
    VectorPoint vp;
    vp.start_pt = Vec2d(last_frame->position().x(), last_frame->position().y());
    vp.end_pt = Vec2d(obs_frame.position().x(), obs_frame.position().y());
    vp.heading = (vp.end_pt - vp.start_pt).Angle();
    vp.length = (vp.end_pt - vp.start_pt).Length();
    obs_frame.SetVecPoint(vp);
    double time_diff =
        std::max(obs_frame.timestamp() - last_frame->timestamp(), 1e-6);
    double scalar_acc = (obs_frame.speed() - last_frame->speed()) / time_diff;
    obs_frame.SetScalarAcc(scalar_acc);
  }

  const double half_length = obs_frame.length() * 0.5;
  const double half_width = obs_frame.width() * 0.5;
  // set anchor point
  AnchorPoints anchor_points;
  const double sin_angle = std::sin(obs_frame.heading());
  const double cos_angle = std::cos(obs_frame.heading());
  anchor_points.left_front_position.set_x(obs_frame.position().x() +
                                          half_length * cos_angle -
                                          half_width * sin_angle);
  anchor_points.left_front_position.set_y(obs_frame.position().y() +
                                          half_length * sin_angle +
                                          half_width * cos_angle);
  anchor_points.right_front_position.set_x(obs_frame.position().x() +
                                           half_length * cos_angle +
                                           half_width * sin_angle);
  anchor_points.right_front_position.set_y(obs_frame.position().y() +
                                           half_length * sin_angle -
                                           half_width * cos_angle);
  anchor_points.left_back_position.set_x(obs_frame.position().x() -
                                         half_length * cos_angle -
                                         half_width * sin_angle);
  anchor_points.left_back_position.set_y(obs_frame.position().y() -
                                         half_length * sin_angle +
                                         half_width * cos_angle);
  anchor_points.right_back_position.set_x(obs_frame.position().x() -
                                          half_length * cos_angle +
                                          half_width * sin_angle);
  anchor_points.right_back_position.set_y(obs_frame.position().y() -
                                          half_length * sin_angle -
                                          half_width * cos_angle);
  obs_frame.SetAnchorPoints(anchor_points);

  if (obs_history_frames_.full()) {
    obs_history_frames_.pop_back();
  }
  obs_history_frames_.emplace_front(std::move(obs_frame));
}

void Obstacle::InsertFrame(const StaticObstacleInfo& stc_obs) {
  ObstacleFrame obs_frame;
  obs_frame.SetId(stc_obs.id);
  obs_frame.SetTimestamp(stc_obs.stamp);
  obs_frame.SetPolygon(stc_obs.polygon);
  obs_frame.SetIsStatic(true);
  obs_frame.SetSeqNum(stc_obs.seq_num);
  obs_frame.SetPosition(stc_obs.pos);
  obs_frame.SetType(stc_obs.type);
  is_static_ = true;
  if (obs_history_frames_.full()) {
    obs_history_frames_.pop_back();
  }
  obs_history_frames_.emplace_front(std::move(obs_frame));
}

ObstacleFrame* Obstacle::GetLatestFrame() {
  if (obs_history_frames_.empty())
    return nullptr;
  else
    return &obs_history_frames_.front();
}

ObstacleFrame* Obstacle::GetFrameAt(int32_t idx) {
  if (idx < 0 || idx >= static_cast<int32_t>(obs_history_frames_.size())) {
    return nullptr;
  }
  auto iter = obs_history_frames_.begin();
  auto target_iter = std::next(iter, idx);

  return &(*target_iter);
}

const CircularQueue<ObstacleFrame>& Obstacle::GetObsHistoryFrames() const {
  return obs_history_frames_;
}

CircularQueue<ObstacleFrame>* Obstacle::GetObsHistoryFramesPtr() {
  return &obs_history_frames_;
}

void Obstacle::ConvertDynamicToFloats(const Vec2d& anchor_point,
                                      const double& rotate_heading,
                                      const int32_t& obs_idx,
                                      const std::size_t& start_idx,
                                      std::vector<float>* const fp) const {
  std::size_t vector_end_idx =
      start_idx + max_vector_size_ * dynamic_vector_dim_;
  if (obs_history_frames_.empty()) {
    return;
  }
  const double sample_time_step = 0.1;
  const double vec_threshold = 0.05;
  const auto& curr_frame = obs_history_frames_.front();
  // float is_fusion_lidar = static_cast<float>((curr_frame.fusion_type() >> 2)
  // & 1); LINFO("Obstacle [%s] is fusion lidar %.f fusin type %d", id_.c_str(),
  // is_fusion_lidar, curr_frame.fusion_type()); LINFO("Obstacle [%s] history
  // size %lu", id().c_str(), obs_history_frames_.size());
  const double& curr_time_stamp = curr_frame.timestamp();
  auto iter = obs_history_frames_.begin();
  for (std::size_t frame_idx = 0;
       frame_idx < obs_history_frames_.size() &&
       iter != obs_history_frames_.end() && vector_end_idx > start_idx;
       frame_idx += 2, vector_end_idx -= dynamic_vector_dim_) {
    Vec2d pt =
        math::RotateVector2d(iter->position() - anchor_point, rotate_heading);
    Vec2d prev_pt = pt;
    auto prev_iter = std::next(iter, 2);
    if (frame_idx + 2 < obs_history_frames_.size() &&
        prev_iter != obs_history_frames_.end() &&
        std::fabs(std::fabs(iter->timestamp() - prev_iter->timestamp()) -
                  sample_time_step) < vec_threshold) {
      prev_pt = math::RotateVector2d(prev_iter->position() - anchor_point,
                                     rotate_heading);
    }
    float is_in_junction = 0.0;
    if ((iter->static_scene() == ObstacleStaticScene::IN_JUNCTION) ||
        (iter->static_scene() == ObstacleStaticScene::ROUNDABOUT_JUNCTION)) {
      is_in_junction = 1.0;
    }
    float super_type = 0.0;
    if (iter->super_type() == ObstacleSuperType::VEHICLE) {
      super_type = 1.0;
    } else if (iter->super_type() == ObstacleSuperType::CYCLIST) {
      super_type = 2.0;
    } else if (iter->super_type() == ObstacleSuperType::PEDESTRIAN) {
      super_type = 3.0;
    }
    double heading = math::NormalizeAngle(iter->heading() + rotate_heading);
    fp->at(vector_end_idx - dynamic_vector_dim_) = prev_pt.x();
    fp->at(vector_end_idx - dynamic_vector_dim_ + 1u) = prev_pt.y();
    fp->at(vector_end_idx - dynamic_vector_dim_ + 2u) = pt.x();
    fp->at(vector_end_idx - dynamic_vector_dim_ + 3u) = pt.y();
    fp->at(vector_end_idx - dynamic_vector_dim_ + 4u) = heading;
    const double cos_angle = std::cos(heading);
    const double sin_angle = std::sin(heading);
    fp->at(vector_end_idx - dynamic_vector_dim_ + 5u) = cos_angle;
    fp->at(vector_end_idx - dynamic_vector_dim_ + 6u) = sin_angle;
    fp->at(vector_end_idx - dynamic_vector_dim_ + 7u) =
        iter->speed() * cos_angle;
    fp->at(vector_end_idx - dynamic_vector_dim_ + 8u) =
        iter->speed() * sin_angle;
    fp->at(vector_end_idx - dynamic_vector_dim_ + 9u) =
        iter->timestamp() - curr_time_stamp;
    fp->at(vector_end_idx - dynamic_vector_dim_ + 10u) = iter->length();
    fp->at(vector_end_idx - dynamic_vector_dim_ + 11u) = iter->width();
    fp->at(vector_end_idx - dynamic_vector_dim_ + 12u) = iter->height();
    fp->at(vector_end_idx - dynamic_vector_dim_ + 13u) = iter->type();
    fp->at(vector_end_idx - dynamic_vector_dim_ + 14u) = super_type;
    fp->at(vector_end_idx - dynamic_vector_dim_ + 15u) = is_in_junction;
    fp->at(vector_end_idx - dynamic_vector_dim_ + 16u) = 0.0;  // no interpolate
    // fp->at(vector_end_idx - dynamic_vector_dim_ + 17u) = 0.0;  //
    // static_cast<float>(obs_idx); fp->at(vector_end_idx - dynamic_vector_dim_
    // + 18u) = 0.0;
    fp->at(vector_end_idx - dynamic_vector_dim_ + 17u) =
        std::max(static_cast<double>(iter->obstacle_lights()[0]) - 1.0, 0.0);
    fp->at(vector_end_idx - dynamic_vector_dim_ + 18u) =
        std::max(static_cast<double>(iter->obstacle_lights()[1]) - 1.0, 0.0);

    fp->at(vector_end_idx - dynamic_vector_dim_ + 19u) = 0.0;
    fp->at(vector_end_idx - dynamic_vector_dim_ + 20u) = 0.0;
    fp->at(vector_end_idx - dynamic_vector_dim_ + 21u) = 0.0;
    fp->at(vector_end_idx - dynamic_vector_dim_ + 22u) = 0.0;
    fp->at(vector_end_idx - dynamic_vector_dim_ + 23u) = 0.0;
    iter = prev_iter;
  }

  // for (int vec_idx = 0; vec_idx < max_vector_size_; ++vec_idx) {
  //   std::string vector_str;
  //   for (int i = 0; i < dynamic_vector_dim_; ++i) {
  //     std::stringstream ss;
  //     ss << fp->at(start_idx + vec_idx * dynamic_vector_dim_ + i) << ", ";
  //     vector_str += ss.str();
  //   }
  //   LDEBUG("Dynamic obs %s vector %d :\n %s", id().c_str(), vec_idx,
  //   vector_str.c_str());
  // }
}

void Obstacle::ConvertStaticToFloats(const Vec2d& anchor_point,
                                     const double& rotate_heading,
                                     const int32_t& obs_idx,
                                     const std::size_t& start_idx,
                                     std::vector<float>* const fp) const {
  std::size_t vector_end_idx =
      start_idx + max_vector_size_ * static_vector_dim_;
  if (obs_history_frames_.empty()) {
    return;
  }
  const auto& curr_frame = obs_history_frames_.front();
  const auto& points = curr_frame.polygon();
  // const auto type = static_cast<float>(curr_frame.type());
  std::size_t max_size = std::min(max_vector_size_, points.size());
  if (max_size < 2) {
    return;
  }

  for (std::size_t point_idx = 0u; point_idx < max_size - 1;
       ++point_idx, vector_end_idx -= static_vector_dim_) {
    const auto& start_pt = math::RotateVector2d(
        points.at(point_idx) - anchor_point, rotate_heading);
    const auto& next_pt = math::RotateVector2d(
        points.at(point_idx + 1u) - anchor_point, rotate_heading);
    fp->at(vector_end_idx - static_vector_dim_) = start_pt.x();
    fp->at(vector_end_idx - static_vector_dim_ + 1u) = start_pt.y();
    fp->at(vector_end_idx - static_vector_dim_ + 2u) = next_pt.x();
    fp->at(vector_end_idx - static_vector_dim_ + 3u) = next_pt.y();
    fp->at(vector_end_idx - static_vector_dim_ + 4u) =
        0;  // adjust before not set static obs
    fp->at(vector_end_idx - static_vector_dim_ + 5u) =
        static_cast<float>(obs_idx);
    fp->at(vector_end_idx - static_vector_dim_ + 6u) =
        static_cast<float>(point_idx);
    fp->at(vector_end_idx - static_vector_dim_ + 7u) = 0.0;
  }
  // for (int vec_idx = 0; vec_idx < max_vector_size_; ++vec_idx) {
  //   std::string vector_str;
  //   for (int i = 0; i < static_vector_dim_; ++i) {
  //     std::stringstream ss;
  //     ss << fp->at(start_idx + vec_idx * static_vector_dim_ + i) << ", ";
  //     vector_str += ss.str();
  //   }
  //   LDEBUG("Static obs %s vector %d :\n %s", id().c_str(), vec_idx,
  //   vector_str.c_str());
  // }
}

void Obstacle::ConvertToFloats(const Vec2d& anchor_point,
                               const double& rotate_heading,
                               const int32_t& obs_idx,
                               const std::size_t& start_idx,
                               std::vector<float>* const fp) const {
  if (is_static_) {
    ConvertStaticToFloats(anchor_point, rotate_heading, obs_idx, start_idx, fp);
  } else {
    ConvertDynamicToFloats(anchor_point, rotate_heading, obs_idx, start_idx,
                           fp);
  }
}

std::vector<Vec2d> StationaryObstacle::GetCornerPoints(
    const TrajectoryPoint& point) const {
  // Using Polygen corner_points only IsStationary.
  if (!polygon_.points().empty()) {
    const Transformer current_tf(position_, theta_);
    const Transformer pt_tf(point, point.theta);
    std::vector<Vec2d> trans_pts;
    for (const auto& corner_pt : polygon_.points()) {
      const auto trans_pt = pt_tf.TransInverse(current_tf.Trans(corner_pt));
      trans_pts.emplace_back(trans_pt);
    }
    return trans_pts;
  } else {
    return GetBoundingBox().GetCornersWithBufferCounterClockwise(
        /*lat_buffer=*/0.0, /*lon_buffer=*/0.0);
  }
}

}  // namespace planning
}  // namespace ad_byd
