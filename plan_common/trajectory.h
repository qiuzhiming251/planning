
#ifndef AD_BYD_PLANNING_COMMON_TRAJECTORY_H_
#define AD_BYD_PLANNING_COMMON_TRAJECTORY_H_
#include <memory>

#include "plan_common/type_def.h"

namespace ad_byd {
namespace planning {
class Trajectory {
 public:
  Trajectory() = default;
  explicit Trajectory(const std::vector<TrajectoryPoint> &points);
  explicit Trajectory(std::vector<TrajectoryPoint> &&points);
  ~Trajectory() = default;

  // reset
  void Reset();

  // timestamp
  double timestamp() const { return timestamp_; }
  void set_timestamp(const double &timestamp) { timestamp_ = timestamp; }

  // trajectory points
  const std::vector<TrajectoryPoint> &points() const { return points_; }
  void set_points(const std::vector<TrajectoryPoint> &points);
  void set_points(std::vector<TrajectoryPoint> &&points);

  // sl_bounds
  void set_sl_bounds(std::vector<SLBoundary> &&sl_bounds) {
    sl_bounds_ = std::move(sl_bounds);
  }

  // ref_points
  void set_ref_points(std::vector<PathPoint> &&ref_points) {
    ref_points_ = std::move(ref_points);
  }

  // probability
  double probability() const { return probability_; }
  void set_probability(const double &probability) {
    probability_ = probability;
  }

  // point count
  size_t point_size() const { return points_.size(); }

  /// @brief Get SLBoundary point by index
  /// @param idx input the trajectory point idx
  /// @return SLBoundary
  const SLBoundary &GetSLBoundAtIdx(const size_t &idx) const;

  /// @brief Get ref_point point by index
  /// @param idx input the trajectory point idx
  /// @return ref_point
  const PathPoint &GetRefPointAtIdx(const size_t &idx) const;

  /// @brief Get trajectory point by accum_s, interpolate by accu_s
  /// @param accu_s input accum_s, unit m
  /// @return trajectory point
  TrajectoryPoint GetPointAtS(const double &accu_s) const;

  /// @brief Get nearest point by accum_s, lower_bound
  /// @param accu_s input accum_s, unit m
  /// @return trajectory point index
  size_t GetNearestIndexAtS(const double &accu_s) const;

  /// @brief Get trajectory point by t, interpolate by t
  /// @param t input t, unit s
  /// @return trajectory point
  TrajectoryPoint GetPointAtTime(const double &t) const;

  /// @brief Get trajectory point by t, interpolate by t
  /// @param t input t, unit s
  /// @return SLPoint point for efficient
  SLPoint GetSLPointAtTime(const double &t) const;

  /// @brief Get nearest time point, lower_bound
  /// @param t input t, unit s
  /// @return trajectory point index
  size_t GetNearestIndexAtTime(const double &t) const;

  /// @brief get distance to trajectory
  /// @param point input x-y
  /// @param nearest_pt output the nearest pt on trajectory
  /// @return the min distance
  double GetDistance(const Point2d &point, TrajectoryPoint *nearest_pt) const;

  /// @brief Get nearest point
  /// @param point input point2d
  /// @return trajectory point index
  size_t GetNearestIndex(const Point2d &point) const;

  /// @brief check if trajectory is valid, length and curvature ok
  /// @return
  bool IsValid() const;

  void set_intention(const ObstacleIntention &intention) {
    intention_ = intention;
  }
  const ObstacleIntention &intention() const { return intention_; }

  double length() const {
    return points_.size() < 2 ? 0.0 : points_.back().accum_s;
  }

 private:
  void ComputeAccumulatedS();

 private:
  double timestamp_ = 0.0;
  double probability_ = 1.0;
  TrajectoryPoint empty_traj_pt_;
  SLBoundary empty_sl_bound_;
  PathPoint empty_path_pt_;
  std::vector<TrajectoryPoint> points_;
  std::vector<SLBoundary> sl_bounds_;
  std::vector<PathPoint> ref_points_;

  ObstacleIntention intention_ = INTENTION_UNKNOWN;
};
using TrajectoryPtr = std::shared_ptr<Trajectory>;
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_COMMON_TRAJECTORY_H_
