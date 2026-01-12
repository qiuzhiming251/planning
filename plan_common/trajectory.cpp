
#include <algorithm>
#include <cmath>

#include "plan_common/log.h"
#include "plan_common/math/line_segment2d.h"
#include "plan_common/math/linear_interpolation.h"
#include "plan_common/math/math_utils.h"
#include "trajectory.h"
namespace ad_byd {
namespace planning {

Trajectory::Trajectory(const std::vector<TrajectoryPoint> &points)
    : points_(points) {
  assert(!points_.empty());
  ComputeAccumulatedS();
}
Trajectory::Trajectory(std::vector<TrajectoryPoint> &&points)
    : points_(std::move(points)) {
  assert(!points_.empty());
  ComputeAccumulatedS();
}

void Trajectory::Reset() {
  points_.clear();
  timestamp_ = 0.0;
  probability_ = 1.0;
  points_.clear();
  sl_bounds_.clear();
  ref_points_.clear();
  intention_ = INTENTION_UNKNOWN;
}

void Trajectory::set_points(const std::vector<TrajectoryPoint> &points) {
  points_ = points;
  ComputeAccumulatedS();
}
void Trajectory::set_points(std::vector<TrajectoryPoint> &&points) {
  points_ = std::move(points);
  ComputeAccumulatedS();
}
void Trajectory::ComputeAccumulatedS() {
  if (points_.empty()) {
    LOG_ERROR << "Trajectory empty !";
    return;
  }
  auto it = points_.begin();
  it->accum_s = 0.0;
  it++;
  double ds = 0;
  for (; it != points_.end(); it++) {
    ds += it->DistanceTo(*(it - 1));
    it->accum_s = ds;
  }
}

const SLBoundary &Trajectory::GetSLBoundAtIdx(const size_t &idx) const {
  if (points_.empty() || points_.size() != sl_bounds_.size()) {
    LOG_ERROR << "Trajectory empty !";
    return empty_sl_bound_;
  }
  size_t size = sl_bounds_.size();
  if (idx + 1 > size) {
    LOG_WARN << "GetSLBoundAtIdx index > size !";
    return empty_sl_bound_;
  }
  return sl_bounds_.at(std::min(idx, size - 1));
}

const PathPoint &Trajectory::GetRefPointAtIdx(const size_t &idx) const {
  if (points_.empty() || points_.size() != ref_points_.size()) {
    LOG_ERROR << "Trajectory empty !";
    return empty_path_pt_;
  }
  size_t size = ref_points_.size();
  if (idx + 1 > size) {
    LOG_WARN << "GetRefPointAtIdx index > size !";
    return empty_path_pt_;
  }
  return ref_points_.at(std::min(idx, size - 1));
}

TrajectoryPoint Trajectory::GetPointAtS(const double &accu_s) const {
  if (points_.empty()) {
    LOG_ERROR << "Trajectory empty !";
    return {};
  }
  if (points_.size() == 1) {
    return points_.back();
  } else {
    auto comp = [](const TrajectoryPoint &p, const double &accu_s) {
      return p.accum_s < accu_s;
    };
    auto it_lower =
        std::lower_bound(points_.begin(), points_.end(), accu_s, comp);

    if (it_lower == points_.begin()) {
      return *points_.begin();
    } else if (it_lower == points_.end()) {
      return *points_.rbegin();
    }
    return math::InterpolateUsingLinearApproximation(
        *(it_lower - 1), (it_lower - 1)->accum_s, *it_lower, it_lower->accum_s,
        accu_s);
  }
}

size_t Trajectory::GetNearestIndexAtS(const double &accu_s) const {
  if (points_.empty()) {
    LOG_ERROR << "Trajectory empty !";
    return -1;
  }
  if (accu_s - points_.back().accum_s > Constants::ZERO) {
    return points_.size() - 1;
  } else {
    auto comp = [](const TrajectoryPoint &p, const double &accu_s) {
      return p.accum_s < accu_s;
    };
    auto it_lower =
        std::lower_bound(points_.begin(), points_.end(), accu_s, comp);

    if (std::distance(points_.begin(), it_lower) > 0) {
      if (std::fabs(accu_s - (it_lower - 1)->accum_s) <
          std::fabs(accu_s - it_lower->accum_s)) {
        it_lower--;
      }
    }
    return std::distance(points_.begin(), it_lower);
  }
}

TrajectoryPoint Trajectory::GetPointAtTime(const double &t) const {
  if (points_.empty()) {
    LOG_ERROR << "Trajectory empty !";
    return {};
  }
  if (points_.size() == 1) {
    return points_.back();
  } else {
    auto comp = [](const TrajectoryPoint &p, const double &time) {
      return p.t < time;
    };
    auto it_lower = std::lower_bound(points_.begin(), points_.end(), t, comp);

    if (it_lower == points_.begin()) {
      return *points_.begin();
    } else if (it_lower == points_.end()) {
      return *points_.rbegin();
    }
    return math::InterpolateUsingLinearApproximation(
        *(it_lower - 1), (it_lower - 1)->t, *it_lower, it_lower->t, t);
  }
}
SLPoint Trajectory::GetSLPointAtTime(const double &t) const {
  if (points_.empty()) {
    LOG_ERROR << "Trajectory empty !";
    return {};
  }
  if (points_.size() == 1) {
    return {points_.back().s, points_.back().l};
  } else {
    auto comp = [](const TrajectoryPoint &p, const double &time) {
      return p.t < time;
    };
    auto it_lower = std::lower_bound(points_.begin(), points_.end(), t, comp);

    if (it_lower == points_.begin()) {
      return {points_.front().s, points_.front().l};
    } else if (it_lower == points_.end()) {
      return {points_.back().s, points_.back().l};
    }
    SLPoint p1((it_lower - 1)->s, (it_lower - 1)->l);
    SLPoint p2(it_lower->s, it_lower->l);
    return math::InterpolateUsingLinearApproximation(p1, (it_lower - 1)->t, p2,
                                                     it_lower->t, t);
  }
}
size_t Trajectory::GetNearestIndexAtTime(const double &t) const {
  if (points_.empty()) {
    LOG_ERROR << "Trajectory empty !";
    return -1;
  }
  if (t - points_.back().t > Constants::ZERO) {
    return points_.size() - 1;
  } else {
    auto comp = [](const TrajectoryPoint &p, const double &time) {
      return p.t < time;
    };
    auto it_lower = std::lower_bound(points_.begin(), points_.end(), t, comp);
    if (std::distance(points_.begin(), it_lower) > 0) {
      if (std::fabs(t - (it_lower - 1)->t) < std::fabs(t - it_lower->t)) {
        it_lower--;
      }
    }
    return std::distance(points_.begin(), it_lower);
  }
}

double Trajectory::GetDistance(const Point2d &point,
                               TrajectoryPoint *nearest_pt) const {
  if (points_.empty()) {
    LOG_ERROR << "Trajectory empty !";
    return DBL_MAX;
  }
  double distance = std::numeric_limits<double>::max();
  size_t nearest_idx = 0;
  double w = 0.0;
  for (size_t i = 0; i < points_.size() - 1; ++i) {
    math::LineSegment2d segment(points_[i], points_[i + 1]);
    math::Vec2d trj_pt;
    double dist_t = segment.DistanceTo(point, &trj_pt);
    if (dist_t < distance) {
      distance = dist_t;
      nearest_idx = i;
      w = trj_pt.DistanceTo(points_[i]) /
          std::max(segment.length(), Constants::ZERO);
    }
  }
  if (nearest_pt) {
    const TrajectoryPoint &tp0 = points_[nearest_idx];
    const TrajectoryPoint &tp1 = points_[nearest_idx + 1];
    *nearest_pt = math::InterpolateUsingLinearApproximation(tp0, tp1, w);
  }
  return distance;
}

bool Trajectory::IsValid() const {
  if (points_.size() < 2) {
    LOG_ERROR << "Too few trajectory points!";
    return false;
  }
  return false;
}

size_t Trajectory::GetNearestIndex(const Point2d &point) const {
  auto s_sqr_min = DBL_MAX;
  size_t idx_min = 0;
  for (size_t i = 0; i < points_.size(); i++) {
    double dx = points_[i].x() - point.x();
    double dy = points_[i].y() - point.y();
    double s_sqr = dx * dx + dy * dy;
    if (s_sqr < s_sqr_min) {
      idx_min = i;
      s_sqr_min = s_sqr;
    }
  }
  return idx_min;
}
}  // namespace planning
}  // namespace ad_byd
