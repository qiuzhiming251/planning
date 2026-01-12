

#ifndef ST_PLANNING_MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_KDTREE
#define ST_PLANNING_MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_KDTREE

#include <array>
#include <string>
#include <utility>
#include <vector>

#include <stdint.h>

#include "plan_common/math/geometry/aabox2d.h"
#include "plan_common/math/geometry/aabox_kdtree2d.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/segment_matcher/aabox_info.h"
#include "plan_common/math/segment_matcher/segment_matcher.h"
#include "plan_common/math/vec.h"

namespace st {

class SegmentMatcherKdtree : public SegmentMatcher {
 public:
  explicit SegmentMatcherKdtree(const std::vector<Vec2d>& points);

  explicit SegmentMatcherKdtree(const std::vector<Segment2d>& segments);

  explicit SegmentMatcherKdtree(
      const std::vector<std::pair<std::string, Segment2d>>& named_segments);

  bool GetNearestSegmentId(double x, double y, std::string* id) const override;

  std::vector<std::string> GetSegmentIdInRadius(double x, double y,
                                                double r) const override;

  bool GetNearestSegmentIndex(double x, double y, int* index) const override;

  std::vector<int> GetSegmentIndexInRadius(double x, double y,
                                           double r) const override;

  const Segment2d* GetNearestSegment(double x, double y) const override;

  std::vector<const Segment2d*> GetSegmentInRadius(double x, double y,
                                                   double r) const override;

  bool GetNearestNamedSegment(double x, double y, Segment2d* seg,
                              std::string* id) const override;

  std::vector<std::pair<const Segment2d*, std::string>>
  GetNamedSegmentsInRadius(double x, double y, double r) const override;

  SegmentMatcherKdtree(const SegmentMatcherKdtree&) = delete;

  /************************ Get Segment Within AABox ************************/
  std::vector<std::string> GetSegmentIdInAABox(const AABox2d& aabox) const;

  std::vector<int> GetSegmentIndexInAABox(const AABox2d& aabox) const;

  std::vector<const Segment2d*> GetSegmentInAABox(const AABox2d& aabox) const;

  std::vector<std::pair<const Segment2d*, std::string>> GetNamedSegmentsInAABox(
      const AABox2d& aabox) const;

 private:
  AABoxKDTree2d<AABoxInfo> segments_tree_;
};

}  // namespace st

#endif  // ST_PLANNING_MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_KDTREE
