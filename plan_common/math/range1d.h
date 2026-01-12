

#ifndef ONBOARD_MATH_RANGE1D_H_
#define ONBOARD_MATH_RANGE1D_H_

#include <algorithm>
#include <string>
#include <type_traits>

#include "absl/strings/str_cat.h"
//#include "global/buffered_logger.h"
//#include "lite/check.h"

namespace st {

// This class represents a one dimensional range. It represents numeric range
// between [low, high), and note that the high is not included in the range.
template <typename T,
          typename =
              typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
class Range1d {
 public:
  Range1d() : low_(T(0)), high_(T(0)) {}
  Range1d(T low, T high) : low_(low), high_(high) { CHECK_LE(low, high); }

  T low() const { return low_; }
  T high() const { return high_; }

  T Length() const { return high_ - low_; }
  T Clamp(T v) const { return std::clamp(v, low_, high_); }
  bool Contains(T v) const { return v >= low_ && v < high_; }
  bool Empty() const { return low_ == high_; }

  std::string DebugString() const {
    return absl::StrCat("lo: ", low(), ", hi: ", high());
  }

 private:
  T low_;
  T high_;
};

}  // namespace st

#endif  // ONBOARD_MATH_RANGE1D_H_
