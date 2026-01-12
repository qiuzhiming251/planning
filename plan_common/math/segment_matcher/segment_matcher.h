

#ifndef ONBOARD_MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_H_
#define ONBOARD_MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/types/span.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/segment_matcher/aabox_info.h"
#include "plan_common/math/vec.h"

namespace st {

/**
 * @class SegmentMatcher
 * @brief The base class of segment matcher, you can use different search
 *        engine to inherit it, such as SegmentMatcherKdtree.
 */
class SegmentMatcher {
 public:
  /**
   * @brief Constructor which takes a vector of Vec2d.
   * @param segments The points to construct the SegmentMatcher.
   */
  explicit SegmentMatcher(absl::Span<const Vec2d> points);

  /**
   * @brief Constructor which takes a vector of segments.
   * @param segments The segments to construct the SegmentMatcher.
   */
  explicit SegmentMatcher(const std::vector<Segment2d>& segments);

  /**
   * @brief Constructor which takes a vector of Segments with id.
   * @param named_segments The named_segments to construct the SegmentMatcher.
   */
  explicit SegmentMatcher(
      absl::Span<const std::pair<std::string, Segment2d>> named_segments);

  virtual ~SegmentMatcher() = default;

  SegmentMatcher(const SegmentMatcher&) = delete;

  const Segment2d* GetSegmentByIndex(int index) const;

  const Segment2d* GetSegmentById(const std::string& id) const;

  const std::vector<const Segment2d*>& segments() const { return segments_; }

  /**
   * @brief Convert xy coordinates to sl coordinates. Do not use this function
   *        when constructing with discretized segments
   * @param is_clamp If is_clamp is true, do not support linear extension
   *        interpolation.
   * @param accumulated_s The longitudinal accumulated distance.
   * @param lateral The lateral distance.
   * @return Whether the conversion is successful.
   */
  bool GetProjection(double x, double y, bool is_clamp, double* accumulated_s,
                     double* lateral) const;

  /**
   * @brief Convert xy coordinates to sl coordinates. Do not use this function
   *        when constructing with discretized segments and do not
   *        support linear extension interpolation.
   * @param nearest_point The nearest point on the segment to the input point.
   * @param accumulated_s The longitudinal accumulated distance.
   * @param min_dist The shortest distance from points on the segment
   *         to the input point.
   * @param segment The segment on the lane.
   * @return Whether the conversion is successful.
   */
  bool GetProjection(double x, double y, Vec2d* nearest_point = nullptr,
                     double* accumulated_s = nullptr,
                     double* min_dist = nullptr,
                     Segment2d* segment = nullptr) const;

  /**
   * @brief Get the nearest segment index with heading.
   * @param max_radius Radius of maximum search range.
   * @param max_heading_diff If the angle between point heading and segment
   *        heading is greater than max_heading_diff, the segment is ignored.
   * @param nearest_index The index of nearest segment.
   * @return Whether the search is successful.
   */
  bool GetNearestSegmentIndexWithHeading(double x, double y, double theta,
                                         double max_radius,
                                         double max_heading_diff,
                                         int* nearest_index) const;

  /**
   * @brief Get the nearest segment id with heading.
   * @param max_radius Radius of maximum search range.
   * @param max_heading_diff If the angle between point heading and segment
   *        heading is greater than max_heading_diff, the segment is ignored.
   * @param nearest_id The id of nearest segment.
   * @return Whether the search is successful.
   */

  /*GetSegmentById(id) return nullptr*/
  //   bool GetNearestSegmentIdWithHeading(double x, double y, double theta,
  //                                       double max_radius,
  //                                       double max_heading_diff,
  //                                       std::string* nearest_id) const;

  /**
   * @brief Get segments in search radius with heading.
   * @param max_radius Radius of maximum search range.
   * @param max_heading_diff If the angle between point heading and segment
   *        heading is greater than max_heading_diff, the segment is ignored.
   * @return The segments in search radius.
   */
  /*GetSegmentById(id) return nullptr*/
  //   std::vector<std::string> GetSegmentIdInRadiusWithHeading(
  //       double x, double y, double theta, double max_radius,
  //       double max_heading_diff) const;

  std::vector<int> GetSegmentIndexInRadiusWithHeading(
      double x, double y, double theta, double max_radius,
      double max_heading_diff) const;

  /********************** Get segment id ***********************/
  virtual bool GetNearestSegmentId(double x, double y,
                                   std::string* id) const = 0;

  virtual std::vector<std::string> GetSegmentIdInRadius(double x, double y,
                                                        double r) const = 0;

  /********************** Get segment index ***********************/
  virtual bool GetNearestSegmentIndex(double x, double y, int* index) const = 0;

  virtual std::vector<int> GetSegmentIndexInRadius(double x, double y,
                                                   double r) const = 0;

  /********************** Get segment ***********************/
  virtual const Segment2d* GetNearestSegment(double x, double y) const = 0;

  virtual std::vector<const Segment2d*> GetSegmentInRadius(double x, double y,
                                                           double r) const = 0;

  /*********************** Get named segment *****************/
  virtual bool GetNearestNamedSegment(double x, double y, Segment2d* seg,
                                      std::string* id) const = 0;

  virtual std::vector<std::pair<const Segment2d*, std::string>>
  GetNamedSegmentsInRadius(double x, double y, double radius) const = 0;

 protected:
  std::vector<AABoxInfo> aa_boxes_info_;
  absl::flat_hash_map<std::string, const Segment2d*> segment_map_;
  std::vector<const std::string*> segment_names_;
  std::vector<const Segment2d*> segments_;
  bool index_flag_ = false;
  bool named_flag_ = false;
};

}  // namespace st

#endif  // ONBOARD_MATH_SEGMENT_MATCHER_SEGMENT_MATCHER_H_
