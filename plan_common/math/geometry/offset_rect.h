

#ifndef ONBOARD_MATH_GEOMETRY_OFFSET_RECT_H_
#define ONBOARD_MATH_GEOMETRY_OFFSET_RECT_H_

#include <algorithm>
#include <string>

#include "absl/strings/str_cat.h"
#include "plan_common/math/util.h"

namespace st {

// This class represents a rectangle whose center point has an offset to its
// geometric center. It is a useful simplification of vehicle geometry, as we
// often use the rear axle center as the 'center' of vehicle box.
class OffsetRect {
 public:
  OffsetRect(double half_length, double half_width, double offset)
      : half_length_(half_length), half_width_(half_width), offset_(offset) {}

  double offset_to_front() const { return half_length_ - offset_; }
  double offset_to_rear() const { return -(half_length_ + offset_); }
  double half_length() const { return half_length_; }
  double half_width() const { return half_width_; }
  double offset() const { return offset_; }
  double radius() const {
    return Hypot(
        std::max(std::abs(offset_to_front()), std::abs(offset_to_rear())),
        half_width_);
  }

  std::string DebugString() const {
    return absl::StrCat("half_length: ", half_length_,
                        ", half_width: ", half_width_, ", offset: ", offset_);
  }

 private:
  double half_length_;
  double half_width_;
  // Offset is positive if the center is closer to front side, negative if the
  // center is closer to the rear side.
  double offset_;
};

}  // namespace st

#endif  // ONBOARD_MATH_GEOMETRY_OFFSET_RECT_H_
