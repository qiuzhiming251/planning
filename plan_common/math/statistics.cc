#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <numeric>
#include <sstream>

#include "plan_common/math/statistics.h"

namespace st {

static constexpr int64_t kNsInUs = 1000;
static constexpr int64_t kNsInMs = 1000 * kNsInUs;
static constexpr int64_t kNsInSec = 1000 * kNsInMs;
static constexpr int64_t kNsInMin = 60 * kNsInSec;
static constexpr int64_t kNsInHour = 60 * kNsInMin;

std::string WrapSecond(double sec) {
  static const std::map<int64_t, std::string> units{
      {kNsInHour, "h"}, {kNsInMin, "min"}, {kNsInSec, "s"},
      {kNsInMs, "ms"},  {kNsInUs, "us"},   {1, "ns"}};
  std::ostringstream stream;
  int64_t abs_nanosec = std::abs(sec * 1e9);

  for (auto it = units.rbegin(); it != units.rend(); it++) {
    if (0 != abs_nanosec / it->first) {
      stream << std::fixed << std::setprecision(2)
             << abs_nanosec / static_cast<double>(it->first) << it->second;
      return stream.str();
    }
  }
  return "0";
}

}  // namespace st