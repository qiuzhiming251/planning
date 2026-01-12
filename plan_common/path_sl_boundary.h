

#ifndef ST_PLANNING_COMMON_PATH_SL_BOUNDARY
#define ST_PLANNING_COMMON_PATH_SL_BOUNDARY

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/scheduler.pb.h"
#include "drive_passage.h"

namespace st::planning {

enum SlBoundaryType {
  TYPE_CURB = 0,
  TYPE_LANE_BOUNDARY = 1,
  TYPE_OBJECT = 2,
  TYPE_VEGETATION = 3
};

enum CenterLineOffsetType {
  CENTER_LINE_OFFSET_UNKNOWN = 0,
  CENTER_LINE_OFFSET_LARGE_VEHICLE_AVOID = 1,
  CENTER_LINE_OFFSET_PAUSE = 2,
  CENTER_LINE_OFFSET_PUSH = 3
};

class PathSlBoundary {
 public:
  PathSlBoundary() = default;

  explicit PathSlBoundary(
      std::vector<double> s, std::vector<double> ref_center_l,
      std::vector<double> right_l, std::vector<double> left_l,
      std::vector<double> opt_right_l, std::vector<double> opt_left_l,
      std::vector<double> target_right_l, std::vector<double> target_left_l,
      std::vector<Vec2d> ref_center_xy, std::vector<Vec2d> right_xy,
      std::vector<Vec2d> left_xy, std::vector<Vec2d> opt_right_xy,
      std::vector<Vec2d> opt_left_xy, std::vector<Vec2d> target_right_xy,
      std::vector<Vec2d> target_left_xy,
      CenterLineOffsetType offset_type =
          CenterLineOffsetType::CENTER_LINE_OFFSET_UNKNOWN,
      double offset_value = 0.0);

  // reference centers are calculated by boundary
  explicit PathSlBoundary(
      std::vector<double> s, std::vector<double> right_l,
      std::vector<double> left_l, std::vector<double> opt_right_l,
      std::vector<double> opt_left_l, std::vector<double> target_right_l,
      std::vector<double> target_left_l, std::vector<Vec2d> right_xy,
      std::vector<Vec2d> left_xy, std::vector<Vec2d> opt_right_xy,
      std::vector<Vec2d> opt_left_xy, std::vector<Vec2d> target_right_xy,
      std::vector<Vec2d> target_left_xy);

  bool IsEmpty() const { return s_vec_.empty(); }
  int size() const { return s_vec_.size(); }

  absl::Span<const double> s_vector() const { return s_vec_; }

  double start_s() const {
    CHECK(!s_vec_.empty()) << "Empty path sl boundary!";
    return s_vec_.front();
  }

  double end_s() const {
    CHECK(!s_vec_.empty()) << "Empty path sl boundary!";
    return s_vec_.back();
  }

  absl::Span<const double> reference_center_l_vector() const {
    return ref_center_l_vec_;
  }

  absl::Span<const double> right_l_vector() const { return right_l_vec_; }

  absl::Span<const double> left_l_vector() const { return left_l_vec_; }

  absl::Span<const double> opt_right_l_vector() const {
    return opt_right_l_vec_;
  }

  absl::Span<const double> opt_left_l_vector() const { return opt_left_l_vec_; }

  absl::Span<const double> target_right_l_vector() const {
    return target_right_l_vec_;
  }

  absl::Span<const double> target_left_l_vector() const {
    return target_left_l_vec_;
  }

  absl::Span<const Vec2d> reference_center_xy_vector() const {
    return ref_center_xy_vec_;
  }

  absl::Span<const Vec2d> right_xy_vector() const { return right_xy_vec_; }

  absl::Span<const Vec2d> left_xy_vector() const { return left_xy_vec_; }

  absl::Span<const Vec2d> opt_right_xy_vector() const {
    return opt_right_xy_vec_;
  }

  absl::Span<const Vec2d> opt_left_xy_vector() const {
    return opt_left_xy_vec_;
  }

  absl::Span<const Vec2d> target_right_xy_vector() const {
    return target_right_xy_vec_;
  }

  absl::Span<const Vec2d> target_left_xy_vector() const {
    return target_left_xy_vec_;
  }

  const std::vector<SlBoundaryType>& right_type_vector() const {
    return right_type_vec_;
  }

  const std::vector<SlBoundaryType>& left_type_vector() const {
    return left_type_vec_;
  }

  // {right_l, left_l}
  std::pair<double, double> QueryBoundaryL(double s) const;
  // {opt_right_l, opt_left_l}
  std::pair<double, double> QueryOptBoundaryL(double s) const;
  // {target_right_l, target_left_l}
  std::pair<double, double> QueryTargetBoundaryL(double s) const;
  // {right_xy, left_xy}
  std::pair<Vec2d, Vec2d> QueryBoundaryXY(double s) const;
  // {opt_right_xy, opt_left_xy}
  std::pair<Vec2d, Vec2d> QueryOptBoundaryXY(double s) const;
  // {target_right_xy, target_left_xy}
  std::pair<Vec2d, Vec2d> QueryTargetBoundaryXY(double s) const;

  double QueryReferenceCenterL(double s) const;
  Vec2d QueryReferenceCenterXY(double s) const;

  void ModifySoftBoundByDpLabel(const DrivePassage& drive_passage,
                                const double s_catchup, const double s_overtake,
                                const double l_press, const bool is_left);

  void ModifyHardBoundByDpLabel(const DrivePassage& drive_passage,
                                const double s_catchup, const double s_overtake,
                                const double l_press, const bool is_left);
  bool ModifyReferenceCenter(const DrivePassage& drive_passage,
                             const double l_offset);
  void SwapOptBoundary() {
    right_l_vec_.swap(opt_right_l_vec_);
    left_l_vec_.swap(opt_left_l_vec_);
    right_xy_vec_.swap(opt_right_xy_vec_);
    left_xy_vec_.swap(opt_left_xy_vec_);
    resampled_right_l_vec_.swap(resampled_opt_right_l_vec_);
    resampled_left_l_vec_.swap(resampled_opt_left_l_vec_);
    resampled_right_xy_vec_.swap(resampled_opt_right_xy_vec_);
    resampled_left_xy_vec_.swap(resampled_opt_left_xy_vec_);
  }

  void ToProto(ScheduledPathBoundaryProto* proto) const;

  void DumpToDebugFrame(int task_i) const;

  const std::pair<CenterLineOffsetType, double>& offset_type_value() const {
    return offset_type_value_;
  }

 private:
  void BuildResampledData();
  // {start_index, lerp_factor}
  std::pair<int, double> FindLerpInfo(double s) const;

  std::vector<double> s_vec_;             // Station.
  std::vector<double> ref_center_l_vec_;  // Reference center
  // Right or left l vec is always the outer boundary
  std::vector<double> right_l_vec_;
  std::vector<double> left_l_vec_;
  // outer boundary for optimizer
  std::vector<double> opt_right_l_vec_;
  std::vector<double> opt_left_l_vec_;
  // Target right or left l vec is always the inner boundary
  std::vector<double> target_right_l_vec_;
  std::vector<double> target_left_l_vec_;
  std::vector<Vec2d> ref_center_xy_vec_;
  std::vector<Vec2d> right_xy_vec_;
  std::vector<Vec2d> left_xy_vec_;
  std::vector<Vec2d> opt_right_xy_vec_;
  std::vector<Vec2d> opt_left_xy_vec_;
  std::vector<Vec2d> target_right_xy_vec_;
  std::vector<Vec2d> target_left_xy_vec_;
  std::vector<SlBoundaryType> right_type_vec_;
  std::vector<SlBoundaryType> left_type_vec_;

  std::vector<double> resampled_s_vec_;  // Resampled Station.
  std::vector<double>
      resampled_ref_center_l_vec_;  // Resampled Reference center
  // Right or left l vec is always the outer boundary
  std::vector<double> resampled_right_l_vec_;
  std::vector<double> resampled_left_l_vec_;
  std::vector<double> resampled_opt_right_l_vec_;
  std::vector<double> resampled_opt_left_l_vec_;
  // Target right or left l vec is always the inner boundary
  std::vector<double> resampled_target_right_l_vec_;
  std::vector<double> resampled_target_left_l_vec_;
  std::vector<Vec2d> resampled_ref_center_xy_vec_;
  std::vector<Vec2d> resampled_right_xy_vec_;
  std::vector<Vec2d> resampled_left_xy_vec_;
  std::vector<Vec2d> resampled_opt_right_xy_vec_;
  std::vector<Vec2d> resampled_opt_left_xy_vec_;
  std::vector<Vec2d> resampled_target_right_xy_vec_;
  std::vector<Vec2d> resampled_target_left_xy_vec_;
  double sample_interval_ = 0.0;
  std::pair<CenterLineOffsetType, double> offset_type_value_{
      CenterLineOffsetType::CENTER_LINE_OFFSET_UNKNOWN, 0.0};

  template <typename Archive>
  friend void serialize(Archive& ar, PathSlBoundary& path_sl_boundary);
};

inline std::pair<double, double> PathSlBoundary::QueryBoundaryL(
    double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return std::make_pair(Lerp(resampled_right_l_vec_[index],
                             resampled_right_l_vec_[index + 1], factor),
                        Lerp(resampled_left_l_vec_[index],
                             resampled_left_l_vec_[index + 1], factor));
}

inline std::pair<Vec2d, Vec2d> PathSlBoundary::QueryBoundaryXY(double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return std::make_pair(Lerp(resampled_right_xy_vec_[index],
                             resampled_right_xy_vec_[index + 1], factor),
                        Lerp(resampled_left_xy_vec_[index],
                             resampled_left_xy_vec_[index + 1], factor));
}

inline std::pair<double, double> PathSlBoundary::QueryOptBoundaryL(
    double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return std::make_pair(Lerp(resampled_opt_right_l_vec_[index],
                             resampled_opt_right_l_vec_[index + 1], factor),
                        Lerp(resampled_opt_left_l_vec_[index],
                             resampled_opt_left_l_vec_[index + 1], factor));
}

inline std::pair<Vec2d, Vec2d> PathSlBoundary::QueryOptBoundaryXY(
    double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return std::make_pair(Lerp(resampled_opt_right_xy_vec_[index],
                             resampled_opt_right_xy_vec_[index + 1], factor),
                        Lerp(resampled_opt_left_xy_vec_[index],
                             resampled_opt_left_xy_vec_[index + 1], factor));
}

inline std::pair<double, double> PathSlBoundary::QueryTargetBoundaryL(
    double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return std::make_pair(Lerp(resampled_target_right_l_vec_[index],
                             resampled_target_right_l_vec_[index + 1], factor),
                        Lerp(resampled_target_left_l_vec_[index],
                             resampled_target_left_l_vec_[index + 1], factor));
}

inline std::pair<Vec2d, Vec2d> PathSlBoundary::QueryTargetBoundaryXY(
    double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return std::make_pair(Lerp(resampled_target_right_xy_vec_[index],
                             resampled_target_right_xy_vec_[index + 1], factor),
                        Lerp(resampled_target_left_xy_vec_[index],
                             resampled_target_left_xy_vec_[index + 1], factor));
}

inline double PathSlBoundary::QueryReferenceCenterL(double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return Lerp(resampled_ref_center_l_vec_[index],
              resampled_ref_center_l_vec_[index + 1], factor);
}

inline Vec2d PathSlBoundary::QueryReferenceCenterXY(double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return Lerp(resampled_ref_center_xy_vec_[index],
              resampled_ref_center_xy_vec_[index + 1], factor);
}

inline std::pair<int, double> PathSlBoundary::FindLerpInfo(double s) const {
  if (sample_interval_ == 0.0) {
    return {0, 0};
  }
  int idx = static_cast<int>(
      std::floor((s - resampled_s_vec_.front()) / sample_interval_));
  idx = std::clamp<int>(idx, 0, static_cast<int>(resampled_s_vec_.size()) - 2);
  return {idx, (s - resampled_s_vec_[idx]) / sample_interval_};
}
}  // namespace st::planning

#endif  // ST_PLANNING_COMMON_PATH_SL_BOUNDARY
