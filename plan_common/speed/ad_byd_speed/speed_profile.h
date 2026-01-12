

#ifndef AD_BYD_PLANNING_COMMON_SPEED_SPEED_PROFILE_H
#define AD_BYD_PLANNING_COMMON_SPEED_SPEED_PROFILE_H
#include "plan_common/type_def.h"

namespace ad_byd {
namespace planning {
class SpeedProfile {
 public:
  SpeedProfile() = default;
  explicit SpeedProfile(std::vector<SpeedPoint> &&points,
                        bool re_evaluate = true);
  void SetPoints(const std::vector<SpeedPoint> &points,
                 bool re_evaluate = true);
  ~SpeedProfile() = default;

  bool IsValid() const { return speed_points_.size() >= 2; }
  void Reset() { speed_points_.clear(); }

  /// @brief get speed profile time length
  double GetTimeLength() const;

  /// @brief get speed point at time
  /// @param t time
  /// @return default value if input t < start point t. if input time exceed
  /// profile time length, then assume const speed movement according to last v
  SpeedPoint GetSpeedPointAtTime(const double &t) const;

  /// @brief get speed points by s interval
  /// @param start start s
  /// @param length s length
  /// @param interval speed point s interval
  /// @return speed points, point s would not be longer than stop point s, point
  /// length would not be greater than time_length + 1s
  std::vector<SpeedPoint> SampleSpeedPointsByS(const double &start,
                                               const double &length,
                                               const double &interval) const;

  /// @brief get stop point
  /// @param stop_point output, if can stop, this param would be set to computed
  /// stop point. if not, the param would not be modified
  /// @return true if this profile can stop in the future
  bool ComputeStopPoint(SpeedPoint &stop_point) const;

  /// @brief only for debug!!! nust not use this! get all speed points
  // const std::vector<SpeedPoint> &speed_points() const { return
  // speed_points_; }

 private:
  std::vector<SpeedPoint> speed_points_;

  /// @brief use first points and speed of all the points to recompute s and a
  /// (assuming const a between current point and next point)
  /// @return true if succeed
  bool ComputeSpeedProfile();

  /// @brief get speed point at time
  /// @param t time
  /// @param start_idx the index from which to start search
  /// @param sequential_search true: sequential search, false: binary search
  /// @param found_idx output, the index whose point.t >= input t
  /// @return default value if input t < start point t. if input time exceed
  /// profile time length, then assume const speed movement according to last v
  SpeedPoint GetSpeedPointAtTime(const double &t, const size_t &start_idx,
                                 const bool sequential_search,
                                 size_t &found_idx) const;

  /// @brief get speed point at s, use sequential search
  /// @param s s of speed point
  /// @param start_idx the index from which to start search
  /// @param sequential_search true: sequential search, false: binary search
  /// @param found_idx output, the index whose point.s >= input s
  /// @return default value if input s < start point s; assume const speed
  /// movement according to last v if input s > last point s
  SpeedPoint GetSpeedPointAtS(const double &s, const size_t &start_idx,
                              const bool sequential_search,
                              size_t &found_idx) const;

  /// @brief calculate v and s at t by given const a, if stopped at given t,
  /// output v = 0, a remain unchanged instead of output 0
  SpeedPoint CalConstAccSpeedPoint(const SpeedPoint &start, const double a,
                                   const double t) const;

  /// @brief calculate a, v and s at t by given const jerk, if stopped at given
  /// t, output v = 0, a remain unchanged
  SpeedPoint CalConstJerkSpeedPoint(const SpeedPoint &start, const double jerk,
                                    const double t) const;
};
using SpeedProfilePtr = std::shared_ptr<SpeedProfile>;
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_COMMON_SPEED_SPEED_PROFILE_H
