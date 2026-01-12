
#include "plan_common/path/path.h"
#include "plan_common/math/angle.h"
#include "plan_common/math/linear_interpolation.h"
#include "plan_common/math/fast_math.h"
namespace ad_byd {
namespace planning {
Path::Path(std::vector<PathPoint> &&path_points, bool compute_path_profile)
    : points_(std::forward<std::vector<PathPoint>>(path_points)) {
  if (points_.empty()) {
    LOG_ERROR << "path_points empty !";
  }
  if (compute_path_profile) {
    ComputePathProfile(points_);
  } else {
    ComputeAccumulatedS(points_);
  }
}
void Path::set_points(std::vector<PathPoint> &&path_points,
                      bool compute_path_profile) {
  points_ = std::forward<std::vector<PathPoint>>(path_points);
  if (points_.empty()) {
    LOG_ERROR << "path_points empty !";
  }
  if (compute_path_profile) {
    ComputePathProfile(points_);
  } else {
    ComputeAccumulatedS(points_);
  }
}
void Path::set_points(const std::vector<Point2d> &xy_points) {
  points_.clear();
  if (xy_points.empty()) {
    LOG_ERROR << "path_points empty !";
    return;
  }
  points_.reserve(xy_points.size());
  for (auto item : xy_points) {
    PathPoint temp_point;
    temp_point.set_x(item.x());
    temp_point.set_y(item.y());
    points_.emplace_back(temp_point);
  }
  // In some case, succeed junction lane's point number is no more than 4, so
  // ComputePathProfile funciton return false, we need ComputeAccumulatedS to
  // get accum_s.
  if (!ComputePathProfile(points_)) {
    ComputeAccumulatedS(points_);
  }
}

PathPoint Path::GetPointAtS(const double &accum_s) const {
  if (points_.empty()) {
    LOG_ERROR << "Path empty !";
    return {};
  }

  if (points_.size() == 1) {
    return points_.back();
  } else {
    size_t index = GetNearestIndexAtS(accum_s);

    if (points_[index] == points_.front()) {
      return points_.front();
    } else if (points_[index] == points_.back()) {
      return points_.back();
    }

    return math::InterpolateUsingLinearApproximation(
        points_[index - 1], points_[index - 1].accum_s, points_[index],
        points_[index].accum_s, accum_s);
  }
}

PathPoint Path::GetPrecisionPointAtS(const double &accum_s) const {
  if (points_.size() > 1) {
    size_t index = GetIndexOfLowerBound(accum_s);
    if (index == 0) {
      return points_.front();
    } else if (index >= points_.size()) {
      return points_.back();
    }

    return math::InterpolateUsingLinearApproximation(
        points_.at(index - 1), points_.at(index - 1).accum_s, points_.at(index),
        points_.at(index).accum_s, accum_s);
  } else if (points_.size() == 1) {
    return points_.front();
  }
  LOG_ERROR << "Path empty !";
  return {};
}

int Path::GetNearestIndexAtS(const double &accum_s) const {
  if (points_.empty()) {
    LOG_ERROR << "Path empty !";
    return -1;
  }
  if (accum_s - points_.back().accum_s > Constants::ZERO) {
    return static_cast<int>(points_.size() - 1);
  } else {
    size_t index = GetIndexOfLowerBound(accum_s);

    if (index != 0) {
      if (std::fabs(accum_s - points_[index - 1].accum_s) <
          std::fabs(accum_s - points_[index].accum_s)) {
        --index;
      }
    }
    return static_cast<int>(index);
  }
}

double Path::GetDistance(const Point2d &point, PathPoint *nearest_point) const {
  if (points_.size() < 2) {
    LOG_ERROR << "Path empty !";
    return {};
  }
  double distance = std::numeric_limits<double>::max();
  int nearest_idx = 0;
  double w = 0.0;
  for (size_t i = 0; i < points_.size() - 1; ++i) {
    const auto &start = points_[i];
    const auto &end = points_[i + 1];
    const double length = end.accum_s - start.accum_s;
    const math::Vec2d unit_direction = (end - start) / length;
    Point2d path_pt;
    double dist_t = DBL_MAX;
    const double proj = unit_direction.InnerProd(point - start);
    if (length <= math::kMathEpsilon || proj < 0.0) {
      path_pt = start;
      dist_t = point.DistanceTo(start);
    } else if (proj > length) {
      path_pt = end;
      dist_t = point.DistanceTo(end);
    } else {
      path_pt = start + unit_direction * proj;
      dist_t = std::abs(unit_direction.CrossProd(point - start));
    }
    // get min_dist
    if (dist_t < distance) {
      distance = dist_t;
      nearest_idx = i;
      w = path_pt.DistanceTo(points_[i]) / std::max(length, Constants::ZERO);
    }
  }
  const PathPoint &pt0 = points_.at(nearest_idx);
  const PathPoint &pt1 = points_.at(nearest_idx + 1);
  if (nearest_point) {
    *nearest_point = math::InterpolateUsingLinearApproximation(pt0, pt1, w);
  }
  return distance;
}

bool Path::SLToXY(const SLPoint &sl_point, Point2d *xy_point,
                  PathPoint *ref_point) const {
  if (!xy_point) {
    LOG_ERROR << "pointer xy_point invalid !";
    return false;
  }

  // 1. Check Path points
  if (points_.size() < 2) {
    LOG_ERROR << "The Path has too few points for SLToXY.";
    return false;
  }

  // 2.Get matched points
  PathPoint matched_point = GetPointAtS(sl_point.s);

  // 3.Recurrence if sl_point.s exceed
  const double diff_s = sl_point.s - matched_point.accum_s;
  const double diff_angle = matched_point.theta;
  xy_point->set_x(matched_point.x() + diff_s * std::cos(diff_angle) -
                  std::sin(diff_angle) * sl_point.l);
  xy_point->set_y(matched_point.y() + diff_s * std::sin(diff_angle) +
                  std::cos(diff_angle) * sl_point.l);
  if (ref_point) {
    *ref_point = matched_point;
  }
  return true;
}

bool Path::XYToSL(const Point2d &xy_point, SLPoint *sl_point,
                  PathPoint *ref_point) const {
  if (!sl_point) {
    LOG_ERROR << "pointer sl_point invalid !";
    return false;
  }

  // 1. Check Path points
  if (points_.size() < 2) {
    LOG_ERROR << "The Path has too few points for XYToSL.";
    return false;
  }
  // 2. Get matched point
  PathPoint matched_point = MatchToPath(points_, xy_point.x(), xy_point.y());
  std::array<double, 3> init_s{};
  std::array<double, 3> init_d{};
  TrajectoryPoint init_point;
  init_point.set_x(xy_point.x());
  init_point.set_y(xy_point.y());
  init_point.theta = 0.0;
  init_point.kappa = 0.0;
  init_point.v = 0.0;
  init_point.a = 0.0;

  // 3. Cartesian_to_frenet
  math::CartesianFrenetConverter::cartesian_to_frenet(
      matched_point.accum_s, matched_point.x(), matched_point.y(),
      matched_point.theta, matched_point.kappa, matched_point.dkappa,
      init_point.x(), init_point.y(), init_point.v, init_point.a,
      init_point.theta, init_point.kappa, &init_s, &init_d);
  sl_point->s = init_s[0];
  sl_point->l = init_d[0];
  if (ref_point) {
    *ref_point = matched_point;
  }
  return true;
}

bool Path::XYToSL(const TrajectoryPoint &traj_point,
                  FrenetPoint *frenet_point) const {
  if (!frenet_point) {
    LOG_ERROR << "pointer frenet_point invalid !";
    return false;
  }

  // 1.check if points valid
  if (points_.empty()) {
    LOG_ERROR << "The path has too few points for XYToSL.";
    return false;
  }

  // 2. Get sl point
  SLPoint sl_point;
  PathPoint ref_point;
  XYToSL(Point2d(traj_point.x(), traj_point.y()), &sl_point, &ref_point);

  // 3. Get match point
  std::array<double, 3> s_position{};
  std::array<double, 3> l_position{};

  // 4. Convert(x, y, v, a, theta, kappa) to Frenet (s, l, dl, ddl)
  math::CartesianFrenetConverter::cartesian_to_frenet(
      sl_point.s, ref_point.x(), ref_point.y(), ref_point.theta,
      ref_point.kappa, ref_point.dkappa, traj_point.x(), traj_point.y(),
      traj_point.v, traj_point.a, traj_point.theta, traj_point.kappa,
      &s_position, &l_position);

  frenet_point->s = s_position.at(0);
  frenet_point->ds = s_position.at(1);
  frenet_point->dds = s_position.at(2);
  frenet_point->l = l_position.at(0);
  frenet_point->dl = l_position.at(1);
  frenet_point->ddl = l_position.at(2);

  return true;
}

bool Path::XYToSL(const math::Box2d &box_2d, SLBoundary *sl_boundary) const {
  return XYToSL(box_2d.GetCornersWithBufferCounterClockwise(
                    /*lat_buffer=*/0.0, /*lon_buffer=*/0.0),
                sl_boundary);
}

bool Path::XYToSL(const std::vector<Point2d> &corners,
                  SLBoundary *sl_boundary) const {
  if (corners.empty()) {
    LOG_ERROR << "The reference line has too few points for XYToSL.";
    return false;
  }

  double s_min = 1e6;
  double s_max = -1e6;
  double l_min = 1e6;
  double l_max = -1e6;

  // Get min max sl points
  for (auto &corner : corners) {
    SLPoint sl_point;
    XYToSL(corner, &sl_point, nullptr);
    s_min = std::min(sl_point.s, s_min);
    s_max = std::max(sl_point.s, s_max);
    l_min = std::min(sl_point.l, l_min);
    l_max = std::max(sl_point.l, l_max);
  }
  sl_boundary->s_min = s_min;
  sl_boundary->s_max = s_max;
  sl_boundary->l_min = l_min;
  sl_boundary->l_max = l_max;
  return true;
}

bool Path::SamplePoints(const double &start_accu_s, const double &length,
                        const double &interval,
                        std::vector<PathPoint> *const path_points) const {
  if (points_.size() < 2 || start_accu_s > points_.back().accum_s) {
    LOG_ERROR << "path or start_s is invalid !";
    return false;
  }

  double valid_length = length + start_accu_s > this->length()
                            ? this->length() - start_accu_s
                            : length;

  size_t idx = 0;
  double dist = start_accu_s;
  path_points->reserve(static_cast<size_t>(valid_length / interval) + 1);
  while (dist < valid_length + start_accu_s + Constants::ZERO) {
    while (idx < points_.size() &&
           dist > points_[idx].accum_s + Constants::ZERO) {
      ++idx;
    }

    if (idx == 0) {
      path_points->emplace_back(points_.at(idx));
    } else {
      double weight = (dist - points_.at(idx - 1).accum_s) /
                      (points_.at(idx).accum_s - points_.at(idx - 1).accum_s);
      path_points->emplace_back(math::InterpolateUsingLinearApproximation(
          points_.at(idx - 1), points_.at(idx), weight));
    }
    dist += interval;
  }
  return true;
}

bool Path::ComputePathProfile(std::vector<PathPoint> &path_points) {
  if (path_points.size() < 4) {
    LOG_ERROR << "points size is invalid !";
    return false;
  }
  size_t points_size = path_points.size();
  if (path_points.size() != points_size) {
    path_points.resize(points_size);
  }

  auto calculate_heading = [&](const size_t &i) {
    double x_delta, y_delta;
    if (i == 0) {
      x_delta = (path_points[i + 1].x() - path_points[i].x());
      y_delta = (path_points[i + 1].y() - path_points[i].y());
    } else if (i == points_size - 1) {
      x_delta = (path_points[i].x() - path_points[i - 1].x());
      y_delta = (path_points[i].y() - path_points[i - 1].y());
    } else {
      x_delta = 0.5 * (path_points[i + 1].x() - path_points[i - 1].x());
      y_delta = 0.5 * (path_points[i + 1].y() - path_points[i - 1].y());
    }
    return st::fast_math::Atan2(y_delta, x_delta);
  };

  auto calculate_accumulated_s = [&](const double &previous_accumulated_s,
                                     const size_t &i) {
    if (i == 0) {
      return 0.0;
    } else {
      double end_segment_s = path_points[i].DistanceTo(path_points[i - 1]);
      return previous_accumulated_s + end_segment_s;
    }
  };

  for (size_t index = 0; index < points_size; ++index) {
    path_points.at(index).set_x(path_points.at(index).x());
    path_points.at(index).set_y(path_points.at(index).y());
    path_points.at(index).theta = calculate_heading(index);  // compute heading
    if (index == 0) {
      path_points.at(index).accum_s = 0.0;
    } else {
      path_points.at(index).accum_s =
          calculate_accumulated_s(path_points.at(index - 1).accum_s, index);
    }
  }

  std::vector<double> y_over_s_first_derivatives;
  std::vector<double> x_over_s_first_derivatives;

  // Get finite difference approximated first derivative of y and x respective
  // to s for kappa calculation
  x_over_s_first_derivatives.reserve(points_size);
  y_over_s_first_derivatives.reserve(points_size);
  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = 0.0;
    double yds = 0.0;
    if (i == 0) {
      xds = (path_points[i + 1].x() - path_points[i].x()) /
            (path_points.at(i + 1).accum_s - path_points.at(i).accum_s);
      yds = (path_points[i + 1].y() - path_points[i].y()) /
            (path_points.at(i + 1).accum_s - path_points.at(i).accum_s);
    } else if (i == points_size - 1) {
      xds = (path_points[i].x() - path_points[i - 1].x()) /
            (path_points.at(i).accum_s - path_points.at(i - 1).accum_s);
      yds = (path_points[i].y() - path_points[i - 1].y()) /
            (path_points.at(i).accum_s - path_points.at(i - 1).accum_s);
    } else {
      xds = (path_points[i + 1].x() - path_points[i - 1].x()) /
            (path_points.at(i + 1).accum_s - path_points.at(i - 1).accum_s);
      yds = (path_points[i + 1].y() - path_points[i - 1].y()) /
            (path_points.at(i + 1).accum_s - path_points.at(i - 1).accum_s);
    }
    x_over_s_first_derivatives.emplace_back(xds);
    y_over_s_first_derivatives.emplace_back(yds);
  }

  for (std::size_t i = 0; i < points_size; ++i) {
    double xdds, ydds;
    if (i == 0) {
      xdds =
          (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
          (path_points.at(i + 1).accum_s - path_points.at(i).accum_s);
      ydds =
          (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
          (path_points.at(i + 1).accum_s - path_points.at(i).accum_s);
    } else if (i == points_size - 1) {
      xdds =
          (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
          (path_points.at(i).accum_s - path_points.at(i - 1).accum_s);
      ydds =
          (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
          (path_points.at(i).accum_s - path_points.at(i - 1).accum_s);
    } else {
      xdds = (x_over_s_first_derivatives[i + 1] -
              x_over_s_first_derivatives[i - 1]) /
             (path_points.at(i + 1).accum_s - path_points.at(i - 1).accum_s);
      ydds = (y_over_s_first_derivatives[i + 1] -
              y_over_s_first_derivatives[i - 1]) /
             (path_points.at(i + 1).accum_s - path_points.at(i - 1).accum_s);
    }
    double xds = x_over_s_first_derivatives[i];
    double yds = y_over_s_first_derivatives[i];
    path_points.at(i).kappa =
        (xds * ydds - yds * xdds) /
        (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
  }

  for (std::size_t i = 0; i < points_size; ++i) {
    double dkappa;
    if (i == 0) {
      dkappa = (path_points.at(i + 1).kappa - path_points.at(i).kappa) /
               (path_points.at(i + 1).accum_s - path_points.at(i).accum_s);
    } else if (i == points_size - 1) {
      dkappa = (path_points.at(i).kappa - path_points.at(i - 1).kappa) /
               (path_points.at(i).accum_s - path_points.at(i - 1).accum_s);
    } else {
      dkappa = (path_points.at(i + 1).kappa - path_points.at(i - 1).kappa) /
               (path_points.at(i + 1).accum_s - path_points.at(i - 1).accum_s);
    }
    path_points.at(i).dkappa = dkappa;
  }
  return true;
}

bool Path::ComputeAccumulatedS(std::vector<PathPoint> &path_points) {
  if (path_points.size() < 2) {
    LOG_ERROR << "points size is invalid !";
    return false;
  }
  auto it = path_points.begin();
  it->accum_s = 0.0;
  it++;
  double ds = 0;
  for (; it != path_points.end(); it++) {
    ds += it->DistanceTo(*(it - 1));
    it->accum_s = ds;
  }
  return true;
}

size_t Path::GetIndexOfLowerBound(const double &accum_s) const {
  if (points_.empty()) {
    LOG_ERROR << "path is invalid !";
    return std::numeric_limits<size_t>::max();
  }
  auto comp = [](const PathPoint &p, const double &accu_s) {
    return p.accum_s < accu_s;
  };
  auto it_lower =
      std::lower_bound(points_.begin(), points_.end(), accum_s, comp);

  return std::distance(points_.begin(), it_lower);
}

PathPoint Path::MatchToPath(const std::vector<PathPoint> &reference_line,
                            const double &x, const double &y) {
  if (reference_line.size() < 2) {
    LOG_ERROR << "reference line is invalid !";
    return {};
  }

  size_t interval = 2;
  auto func_distance_square = [](const PathPoint &point, const double x,
                                 const double y) {
    double dx = point.x() - x;
    double dy = point.y() - y;
    return dx * dx + dy * dy;
  };

  double distance_min = func_distance_square(reference_line.front(), x, y);
  size_t rough_index_min = 0;

  // Use interval as step size to find the index of reference line point which
  // is the minimum distance to the query point.
  for (size_t i = interval; i < reference_line.size(); i += interval) {
    double distance_temp = func_distance_square(reference_line[i], x, y);
    if (distance_temp < distance_min) {
      distance_min = distance_temp;
      rough_index_min = i;
    }
  }

  // Find the index of minimum distance reference line point from index_min -
  // interval to index_min + interval.
  const size_t search_start_index =
      rough_index_min > interval ? rough_index_min - interval : 0;
  const size_t search_end_index =
      rough_index_min + interval >= reference_line.size()
          ? reference_line.size() - 1
          : rough_index_min + interval;

  size_t precise_index_min = rough_index_min;
  for (size_t index = search_start_index; index <= search_end_index; ++index) {
    double distance_temp = func_distance_square(reference_line[index], x, y);
    if (distance_temp < distance_min) {
      distance_min = distance_temp;
      precise_index_min = index;
    }
  }

  size_t path_point_1_index = precise_index_min;
  size_t path_point_2_index = precise_index_min;
  if (precise_index_min > 0 && precise_index_min < reference_line.size() - 1) {
    // The minimum distance point is not the first or the last point on
    // reference line.
    const double previous_point_distance =
        func_distance_square(reference_line[precise_index_min - 1], x, y);
    const double next_point_distance =
        func_distance_square(reference_line[precise_index_min + 1], x, y);
    if (previous_point_distance < next_point_distance) {
      path_point_1_index = precise_index_min - 1;
      path_point_2_index = precise_index_min;
    } else {
      path_point_1_index = precise_index_min;
      path_point_2_index = precise_index_min + 1;
    }
  } else if (precise_index_min == 0 &&
             precise_index_min + 1 < reference_line.size()) {
    // The minimum distance point is the first point.
    path_point_1_index = precise_index_min;
    path_point_2_index = precise_index_min + 1;
  } else if (precise_index_min == reference_line.size() - 1 &&
             precise_index_min >= 1) {
    // The minimum distance point is the last point.
    path_point_1_index = precise_index_min - 1;
    path_point_2_index = precise_index_min;
  } else {
    path_point_1_index = precise_index_min;
    path_point_2_index = precise_index_min;
  }
  const PathPoint &p0 = reference_line[path_point_1_index];
  const PathPoint &p1 = reference_line[path_point_2_index];

  double v0x = x - p0.x();
  double v0y = y - p0.y();

  double v1x = p1.x() - p0.x();
  double v1y = p1.y() - p0.y();

  double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
  double dot = v0x * v1x + v0y * v1y;

  double delta_s = dot / std::max(v1_norm, math::kMathEpsilon);

  return math::InterpolateUsingLinearApproximation(
      p0, p0.accum_s, p1, p1.accum_s, p0.accum_s + delta_s);
}

bool Path::XYToSL(std::vector<PathPoint> &path_points) const {
  if (points_.size() < 2 || path_points.empty()) {
    LOG_ERROR << "sequence XYToSL invalid size !";
    return false;
  }

  int index;
  PathPoint first_match =
      MatchToPath(points_, path_points[0].x(), path_points[0].y());
  if (first_match.accum_s < points_.front().accum_s) {
    index = 0;
  } else if (first_match.accum_s > points_.back().accum_s) {
    index = (int)points_.size() - 1;
  } else {
    index = (int)GetIndexOfLowerBound(first_match.accum_s);
  }

  path_points.front().s = first_match.accum_s;
  const Point2d init_vec_delta(path_points.front() - first_match);
  const Point2d init_vec_direct(cos(first_match.theta), sin(first_match.theta));
  double init_cross_prod = init_vec_direct.CrossProd(init_vec_delta);
  path_points.front().l = init_cross_prod >= Constants::ZERO
                              ? path_points.front().DistanceTo(first_match)
                              : -path_points.front().DistanceTo(first_match);

  size_t i = 1;
  const PathPoint *p0, *p1;
  while (i < path_points.size()) {
    // determine direction
    const Point2d init_vec = index >= (int)points_.size() - 1
                                 ? points_.back() - *(points_.rbegin() + 1)
                                 : points_[index + 1] - points_[index];

    Point2d temp_vec(path_points[i] - points_[index]);

    // init value
    double pre_proj = temp_vec.InnerProd(init_vec) /
                      std::max(init_vec.Length(), math::kMathEpsilon);
    index += std::floor(pre_proj / init_vec.Length());
    index = math::Clamp(index, 0, (int)points_.size() - 1);

    // find the index
    int last_direction = 0;
    while (index <= (int)points_.size() - 1 && index >= 0) {
      const Point2d unit_vec = index >= (int)points_.size() - 1
                                   ? points_.back() - *(points_.rbegin() + 1)
                                   : points_[index + 1] - points_[index];

      const Point2d point_vec(path_points[i] - points_[index]);

      const double temp_proj = point_vec.InnerProd(unit_vec) /
                               std::max(unit_vec.Length(), math::kMathEpsilon);

      int current_direction = temp_proj >= 0.0 ? 1 : -1;
      if ((temp_proj >= 0.0 && temp_proj <= unit_vec.Length()) ||
          last_direction * current_direction < 0) {
        break;
      }
      last_direction = current_direction;
      temp_proj >= 0.0 ? ++index : --index;
    }

    index = math::Clamp(index, 0, (int)points_.size() - 1);
    // boundary condition
    if (index == (int)points_.size() - 1) {
      p0 = &points_[index - 1];
      p1 = &points_.back();
    } else {
      p0 = &points_[index];
      p1 = &points_[index + 1];
    }

    // interpolation
    const Point2d vec_0(path_points[i] - *p0);
    const Point2d vec_1(*p1 - *p0);
    double v1_norm = vec_1.Length();
    double dot = vec_0.InnerProd(vec_1);

    double delta_s = p0->accum_s <= p1->accum_s
                         ? dot / std::max(v1_norm, math::kMathEpsilon)
                         : -dot / std::max(v1_norm, math::kMathEpsilon);
    PathPoint match_point = math::InterpolateUsingLinearApproximation(
        *p0, p0->accum_s, *p1, p1->accum_s, p0->accum_s + delta_s);
    path_points[i].s = match_point.accum_s;

    const Point2d vec_direct(std::cos(p0->theta), std::sin(p0->theta));
    double cross_prod = vec_direct.CrossProd(vec_0);
    path_points[i].l = cross_prod >= Constants::ZERO
                           ? path_points[i].DistanceTo(match_point)
                           : -path_points[i].DistanceTo(match_point);
    ++i;
  }

  return true;
}

bool Path::SLToXY(std::vector<PathPoint> &path_points) const {
  if (points_.size() < 2 || path_points.empty()) {
    LOG_ERROR << "sequence SLToXY invalid size !";
    return false;
  }

  PathPoint match_point = GetPointAtS(path_points.front().s);
  auto ite = std::find(points_.begin(), points_.end(), match_point);
  size_t index = std::distance(points_.begin(), ite);

  const double diff_s = path_points.front().s - match_point.accum_s;
  const double diff_angle = match_point.theta;
  path_points.front().set_x(match_point.x() + diff_s * std::cos(diff_angle) -
                            std::sin(diff_angle) * path_points.front().l);
  path_points.front().set_y(match_point.y() + diff_s * std::sin(diff_angle) +
                            std::cos(diff_angle) * path_points.front().l);

  size_t i = 1;
  while (i < path_points.size()) {
    const double target_s =
        points_[index].accum_s + (path_points[i].s - path_points[i - 1].s);

    if (target_s > points_[index].accum_s) {
      for (; index < points_.size(); ++index) {
        if (points_[index].accum_s >= target_s) {
          --index;
          break;
        }
      }
    } else if (target_s < points_[index].accum_s) {
      for (; index > 0; --index) {
        if (points_[index].accum_s <= target_s) break;
      }
    }

    if (points_[index] == points_.front()) {
      match_point = math::InterpolateUsingLinearApproximation(
          points_.front(), points_.front().accum_s, points_[1],
          points_[1].accum_s, target_s);
    } else if (points_[index] == points_.back()) {
      match_point = math::InterpolateUsingLinearApproximation(
          *(points_.rbegin() + 1), (points_.rbegin() + 1)->accum_s,
          points_.back(), points_.back().accum_s, target_s);
    } else {
      match_point = math::InterpolateUsingLinearApproximation(
          points_[index - 1], points_[index - 1].accum_s, points_[index],
          points_[index].accum_s, target_s);
    }

    const double ds = path_points[i].s - match_point.accum_s;
    const double dangle = match_point.theta;
    path_points[i].set_x(match_point.x() + ds * std::cos(dangle) -
                         std::sin(dangle) * path_points[i].l);
    path_points[i].set_y(match_point.y() + ds * std::sin(dangle) +
                         std::cos(dangle) * path_points[i].l);
    ++i;
  }

  return true;
}
bool Path::SamplePoints(const std::vector<double> &accum_sequence,
                        std::vector<PathPoint> *const path_points) const {
  if (points_.size() < 2 || accum_sequence.empty() ||
      accum_sequence.front() > points_.back().accum_s + Constants::ZERO) {
    LOG_ERROR << "path or start_s is invalid !";
    return false;
  }

  int idx = 0;
  path_points->reserve(accum_sequence.size());
  for (const double s : accum_sequence) {
    while (idx < (int)points_.size() &&
           s > points_[idx].accum_s + Constants::ZERO) {
      ++idx;
    }
    idx = math::Clamp(idx, 0, (int)points_.size() - 1);
    if (idx == 0) {
      path_points->emplace_back(points_.at(idx));
    } else {
      double weight = (s - points_.at(idx - 1).accum_s) /
                      (points_.at(idx).accum_s - points_.at(idx - 1).accum_s);
      path_points->emplace_back(math::InterpolateUsingLinearApproximation(
          points_.at(idx - 1), points_.at(idx), weight));
    }
  }

  return true;
}

bool Path::HasIntersect(const math::LineSegment2d &input_segment,
                        math::Vec2d *const point) const {
  if (points_.size() < 2) {
    return false;
  }
  for (size_t i = 0; i < points_.size() - 1; i++) {
    math::LineSegment2d segment(points_.at(i), points_.at(i + 1));
    if (segment.GetIntersect(input_segment, point)) {
      return true;
    }
  }
  return false;
}
}  // namespace planning
}  // namespace ad_byd
