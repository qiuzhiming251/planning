#ifndef ST_PLANNING_MATH_STATISTICS
#define ST_PLANNING_MATH_STATISTICS
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

namespace st {

std::string WrapSecond(double sec);

template <typename Container>
struct Statistics {
  Statistics() = default;
  Statistics(const Container& v)
      : count(v.size()),
        min(v.size() ? *std::min_element(v.begin(), v.end()) : 0),
        max(v.size() ? *std::max_element(v.begin(), v.end()) : 0),
        mean(Mean(v)),
        stddev(StdDev(v, mean)) {}
  int64_t count{0};
  double min{0}, max{0};
  double mean{0};
  double stddev{0};

  std::string Report(bool is_time) const {
    std::ostringstream stream;
    if (is_time) {
      stream << "count: " << count << ", min: " << WrapSecond(min)
             << ", max: " << WrapSecond(max)
             << ", average: " << WrapSecond(mean)
             << ", stddev: " << WrapSecond(stddev);
    } else {
      stream << "count: " << count << ", min: " << min << ", max: " << max
             << ", average: " << mean << ", stddev: " << stddev;
    }
    return stream.str();
  }

  static double Mean(const Container& v) {
    if (v.empty()) {
      return 0.0;
    }
    return std::accumulate(v.begin(), v.end(), 0.0) * (1.0 / v.size());
  }

  static double StdDev(const Container& v) {
    return Statistics::StdDev(v, Mean(v));
  }

  static double StdDev(const Container& v, double mean) {
    if (v.size() <= 1) {
      return 0.0;
    }
    const double sum_squares =
        std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    const double avg_squares = sum_squares * (1.0 / v.size());
    const double dev_square =
        v.size() / (v.size() - 1.0) * (avg_squares - mean * mean);
    return dev_square < 0.0 ? 0.0 : std::sqrt(dev_square);
  }
};

}  // namespace st

#endif  // ST_PLANNING_MATH_STATISTICS
