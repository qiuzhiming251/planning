

#ifndef ST_PLANNING_MATH_FRENET_FRAME
#define ST_PLANNING_MATH_FRENET_FRAME

#include <algorithm>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/segment_matcher/segment_matcher_kdtree.h"
#include "plan_common/math/vec.h"
#include "plan_common/util/qtfm_segment_matcher_v2.h"
//#include "plan_common/util/source_location.h"

namespace st {

// Build a frenet frame based on discrete 2D points.
// Point in cartesian coordinate can be projected to frenet coordinate by XYToSL
// Use the `Build*FrenetFrame` functions at the bottom to create frenet frames
// based on different implementation types.

enum class FrenetFrameType {
  kBruteFroce = 0,
  kKdTree = 1,
  kQtfmKdTree = 2,
};

class FrenetFrame {
 public:
  const std::vector<Vec2d>& points() const { return points_; }

  const std::vector<Vec2d>& tangents() const { return tangents_; }

  const std::vector<int>& raw_indices() const { return raw_indices_; }

  const std::vector<double>& s_knots() const { return s_knots_; }

  double start_s() const { return s_knots_.front(); }

  double end_s() const { return s_knots_.back(); }

  double length() const { return s_knots_.back() - s_knots_.front(); }

  Vec2d InterpolateTangentByS(double s) const;

  Vec2d InterpolateTangentByXY(const Vec2d& xy) const;

  Vec2d SLToXY(const FrenetCoordinate& sl) const;

  // Transform a cartesian point to a frenet point with different output
  // verbosity
  FrenetCoordinate XYToSL(const Vec2d& xy) const;

  absl::StatusOr<FrenetCoordinate> XYToSLWithHeadingDiffLimit(
      const Vec2d& xy, double heading, double max_heading_diff) const;

  void XYToSL(const Vec2d& xy, FrenetCoordinate* sl, Vec2d* normal) const;

  void XYToSL(const Vec2d& xy, FrenetCoordinate* sl, Vec2d* normal, int* index,
              double* alpha) const;

  void XYToSL(const Vec2d& xy, FrenetCoordinate* sl, Vec2d* normal,
              std::pair<int, int>* raw_index_pair, double* alpha) const;

  Vec2d FindAABBNearestPoint(const Polygon2d& polygon,
                             bool get_max_s = false) const;

  absl::StatusOr<FrenetBox> QueryFrenetBoxWithHeading(
      const Box2d& box, double max_heading_diff = M_PI_2) const;

  absl::StatusOr<FrenetBox> QueryFrenetBoxAt(const Box2d& box) const;

  absl::StatusOr<FrenetBox> QueryFrenetBoxAtContour(
      const Polygon2d& contour) const;

  absl::StatusOr<FrenetBox> QueryFrenetBoxAtPoints(
      absl::Span<const Vec2d> points) const;

  virtual ~FrenetFrame() = default;

  FrenetFrame(std::vector<Vec2d> points, std::vector<double> s_knots,
              std::vector<double> segment_len_inv, std::vector<Vec2d> tangents,
              std::vector<int> raw_indices)
      : points_(std::move(points)),
        s_knots_(std::move(s_knots)),
        segment_len_inv_(std::move(segment_len_inv)),
        tangents_(std::move(tangents)),
        raw_indices_(std::move(raw_indices)) {}

 protected:
  virtual absl::StatusOr<FrenetCoordinate> XYToSLWithHeadingDiffLimitImplement(
      const Vec2d& xy, double heading, double max_heading_diff) const;

  virtual void XYToSLImplement(const Vec2d& xy, FrenetCoordinate* sl,
                               Vec2d* normal, int* index,
                               double* alpha) const = 0;

  absl::StatusOr<FrenetBox> QueryFrenetBoxAtPointsWithHeading(
      absl::Span<const Vec2d> points, double heading,
      double max_heading_diff) const;

  std::tuple<Vec2d, Vec2d, double> GetInterpolationRange(double s) const;

 protected:
  std::vector<Vec2d> points_;
  std::vector<double> s_knots_;
  std::vector<double> segment_len_inv_;
  std::vector<Vec2d> tangents_;
  std::vector<int> raw_indices_;
};

class BruteForceFrenetFrame : public FrenetFrame {
 public:
  BruteForceFrenetFrame(std::vector<Vec2d> points, std::vector<double> s_knots,
                        std::vector<double> segment_len_inv,
                        std::vector<Vec2d> tangents,
                        std::vector<int> raw_indices)
      : FrenetFrame(std::move(points), std::move(s_knots),
                    std::move(segment_len_inv), std::move(tangents),
                    std::move(raw_indices)) {}

  ~BruteForceFrenetFrame() = default;

 private:
  void XYToSLImplement(const Vec2d& xy, FrenetCoordinate* sl, Vec2d* normal,
                       int* index, double* alpha) const override;
};

class KdTreeFrenetFrame : public FrenetFrame {
 public:
  KdTreeFrenetFrame(std::vector<Vec2d> points, std::vector<double> s_knots,
                    std::vector<double> segment_len_inv,
                    std::vector<Vec2d> tangents, std::vector<int> raw_indices,
                    std::vector<Segment2d> segments,
                    std::shared_ptr<SegmentMatcherKdtree> segment_matcher)
      : FrenetFrame(std::move(points), std::move(s_knots),
                    std::move(segment_len_inv), std::move(tangents),
                    std::move(raw_indices)),
        segments_(std::move(segments)),
        segment_matcher_(std::move(segment_matcher)) {}

  ~KdTreeFrenetFrame() = default;

 protected:
  absl::StatusOr<FrenetCoordinate> XYToSLWithHeadingDiffLimitImplement(
      const Vec2d& xy, double heading, double max_heading_diff) const override;

  void XYToSLImplement(const Vec2d& xy, FrenetCoordinate* sl, Vec2d* normal,
                       int* index, double* alpha) const override;

 protected:
  std::vector<Segment2d> segments_;
  std::shared_ptr<SegmentMatcherKdtree> segment_matcher_;
};

class QtfmEnhancedKdTreeFrenetFrame : public KdTreeFrenetFrame {
 public:
  QtfmEnhancedKdTreeFrenetFrame(
      std::vector<Vec2d> points, std::vector<double> s_knots,
      std::vector<double> segment_len_inv, std::vector<Vec2d> tangents,
      std::vector<int> raw_indices, std::vector<Segment2d> segments,
      std::shared_ptr<SegmentMatcherKdtree> segment_matcher,
      std::shared_ptr<planning::QtfmSegmentMatcherV2> qtfm_segment_matcher)
      : KdTreeFrenetFrame(std::move(points), std::move(s_knots),
                          std::move(segment_len_inv), std::move(tangents),
                          std::move(raw_indices), std::move(segments),
                          std::move(segment_matcher)),
        qtfm_segment_matcher_(std::move(qtfm_segment_matcher)) {}

  ~QtfmEnhancedKdTreeFrenetFrame() = default;

 private:
  void XYToSLImplement(const Vec2d& xy, FrenetCoordinate* sl, Vec2d* normal,
                       int* index, double* alpha) const override;

 private:
  std::shared_ptr<planning::QtfmSegmentMatcherV2> qtfm_segment_matcher_;
};

// Build a frenet frame from a list of points by several alternative methods.
// If down_sample_raw_points is true, the points those are very close to the
// previous point (distance < 0.1m) will be removed to build the frenet frame.
// If down_sample_raw_points is false and raw_points are dense (say average
// point-to-point distance << 0.1m), the projection may be less efficient and
// precise. Use the result with caution.
// NOTE: For building KD-tree frenet frame and QTFM kd-tree frenet frame,
// extremely close points (distance < 1e-6m) would be removed from raw_points
// even if down_sample_raw_points is false to make sure the KD-tree construction
// is ok.
absl::StatusOr<BruteForceFrenetFrame> BuildBruteForceFrenetFrame(
    absl::Span<const Vec2d> raw_points, bool down_sample_raw_points);

absl::StatusOr<KdTreeFrenetFrame> BuildKdTreeFrenetFrame(
    absl::Span<const Vec2d> raw_points, bool down_sample_raw_points);

absl::StatusOr<QtfmEnhancedKdTreeFrenetFrame>
BuildQtfmEnhancedKdTreeFrenetFrame(absl::Span<const Vec2d> raw_points,
                                   bool down_sample_raw_points);

}  // namespace st

#endif  // ST_PLANNING_MATH_FRENET_FRAME
