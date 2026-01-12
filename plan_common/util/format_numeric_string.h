#pragma once

#include <limits>
#include <type_traits>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"

namespace st::planning {

// Support enum, bool, int, double, NOT support `unsigned` modifier.
template <
    typename T,
    std::enable_if_t<std::is_enum_v<T> || std::is_arithmetic_v<T>, bool> = true>
std::string FormatNumericString(T value) {
  if constexpr (std::is_enum_v<T> ||
                std::is_same_v<bool, typename std::remove_cv_t<T>>) {
    return absl::StrCat(value);
  } else if constexpr (std::is_integral_v<T>) {
    if (value == std::numeric_limits<T>::max()) {
      return "INTEGRAL_MAX";
    }
    if constexpr (std::is_signed_v<T>) {
      if (value == std::numeric_limits<T>::min()) {
        return "INTEGRAL_MIN";
      }
    }
    return absl::StrCat(value);
  } else {
    if (value == std::numeric_limits<T>::max()) {
      return "DBL_MAX";
    } else if (value == std::numeric_limits<T>::min()) {
      return "DBL_MIN";
    } else if (value == std::numeric_limits<T>::lowest()) {
      return "lowest";
    } else if (value == std::numeric_limits<T>::epsilon()) {
      return "epsilon";
    }
  }
  return absl::StrCat(value);
}

inline std::string FormatNumericString(absl::string_view literal,
                                       double value) {
  if (value == std::numeric_limits<double>::max()) {
    return absl::StrCat(literal, "DBL_MAX");
  } else if (value == std::numeric_limits<double>::min()) {
    return absl::StrCat(literal, "DBL_MIN");
  }
  return absl::StrCat(literal, absl::StrFormat("%.2f", value));
}

}  // namespace st::planning
