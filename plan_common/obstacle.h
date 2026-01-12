

#ifndef AD_BYD_PLANNING_COMMON_OBSTACLE_H
#define AD_BYD_PLANNING_COMMON_OBSTACLE_H

#include "plan_common/gflags.h"
#include "plan_common/log.h"
#include "object_situation.h"
#include "obstacle_frame.h"
#include "trajectory.h"
#include "transformer.h"
#include "plan_common/type_def.h"
#include "plan_common/math/polygon2d.h"
#include "modules/cnoa_pnc/planning/proto/prediction.pb.h"
#include "plan_common/util/circular_queue.h"

namespace ad_byd {
namespace planning {
bool IsStatic(uint8_t type);
class Obstacle {
 public:
  Obstacle() = default;
  Obstacle(const int32_t history_num);
  ~Obstacle() = default;
  const std::string& id() const { return id_; }

  void InsertFrame(const DynamicObstacleInfo& dyn_obs,
                   const ObstacleFrame* const adc_frame);
  void InsertFrame(const StaticObstacleInfo& stc_obs);
  void SetId(const std::string& id) { id_ = id; }

  ObstacleFrame* GetLatestFrame();

  ObstacleFrame* GetFrameAt(int32_t idx);

  const CircularQueue<ObstacleFrame>& GetObsHistoryFrames() const;

  CircularQueue<ObstacleFrame>* GetObsHistoryFramesPtr();

  void ConvertToFloats(const Vec2d& anchor_point, const double& rotate_heading,
                       const int32_t& obs_idx, const std::size_t& start_idx,
                       std::vector<float>* const fp) const;
  void ConvertDynamicToFloats(const Vec2d& anchor_point,
                              const double& rotate_heading,
                              const int32_t& obs_idx,
                              const std::size_t& start_idx,
                              std::vector<float>* const fp) const;
  void ConvertStaticToFloats(const Vec2d& anchor_point,
                             const double& rotate_heading,
                             const int32_t& obs_idx,
                             const std::size_t& start_idx,
                             std::vector<float>* const fp) const;

 private:
  CircularQueue<ObstacleFrame> obs_history_frames_;
  // CircularQueue<ObstacleFrame> static_obs_frames_;
  bool is_static_ = false;
  std::string id_;
  std::size_t max_vector_size_ = FLAGS_ad_byd_planning_max_obstacle_vector_size;
  std::size_t dynamic_vector_dim_ =
      FLAGS_ad_byd_planning_dynamic_obstacle_vector_dim;
  std::size_t static_vector_dim_ =
      FLAGS_ad_byd_planning_static_obstacle_vector_dim;
};

class StationaryObstacle {
 public:
  StationaryObstacle() = delete;
  explicit StationaryObstacle(const std::string& id, const st::ObjectType& type,
                              const math::Polygon2d& polygon,
                              const Vec2d& obs_pos, const double theta,
                              const double length, const double width)
      : id_(id),
        type_(type),
        polygon_(polygon),
        position_(obs_pos),
        theta_(theta),
        length_(length),
        width_(width) {}
  ~StationaryObstacle() = default;

  const std::string& id() const { return id_; }
  st::ObjectType type() const { return type_; }
  double x() const { return position_.x(); }
  double y() const { return position_.y(); }
  const Vec2d& pos() const { return position_; }
  double theta() const { return theta_; }
  double length() const { return length_; }
  double width() const { return width_; }
  const math::Polygon2d& polygon() const { return polygon_; }
  math::Box2d GetBoundingBox() const {
    return {position_, theta_, length_, width_};
  }
  double ds() const { return situation_.lon_distance(); }
  double dl() const { return situation_.lat_distance(); }
  double s() const { return sl_boundary_.center().s; }
  double l() const { return sl_boundary_.center().l; }
  double s_max() const { return situation_.lon_max(); }
  double s_min() const { return situation_.lon_min(); }
  // sl_boundary
  const SLBoundary& sl_boundary() const { return sl_boundary_; }
  void set_sl_boundary(const SLBoundary& sl_boundary) {
    sl_boundary_ = sl_boundary;
  }
  // situation
  const ObjectSituation& situation() const { return situation_; }
  ObjectSituation& mutable_situation() { return situation_; }
  /// @brief Get corner points at trajectory point
  /// @param point input the trajectory point
  /// @return Corner points
  std::vector<Vec2d> GetCornerPoints(const TrajectoryPoint& point) const;

 private:
  std::string id_;
  st::ObjectType type_;
  math::Polygon2d polygon_;
  Vec2d position_;
  double theta_;
  double length_;
  double width_;
  ObjectSituation situation_;
  SLBoundary sl_boundary_;
};
using StationaryObstaclePtr = std::shared_ptr<StationaryObstacle>;
using ObstaclePtr = std::shared_ptr<Obstacle>;
using ObstaclePtr = std::shared_ptr<Obstacle>;

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_COMMON_OBSTACLE_H
