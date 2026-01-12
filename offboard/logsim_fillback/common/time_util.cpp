#include <iomanip>
#include <map>
#include <sstream>

#include "common/time_util.h"
namespace worldview {

namespace util {

static constexpr int64_t kNsInUs = 1000;
static constexpr int64_t kNsInMs = 1000 * kNsInUs;
static constexpr int64_t kNsInSec = 1000 * kNsInMs;
static constexpr int64_t kNsInMin = 60 * kNsInSec;
static constexpr int64_t kNsInHour = 60 * kNsInMin;

std::string formatTime(const std::string& format,
                       const std::chrono::system_clock::time_point& tp) {
  std::time_t time = std::chrono::system_clock::to_time_t(tp);
  std::tm* t = std::localtime(&time);

  std::stringstream ssm;
  ssm << std::put_time(t, format.c_str());
  return ssm.str();
}

std::tm parseTime(const std::string& str, const std::string& format) {
  std::tm t = {};
  std::istringstream issm(str);
  issm >> std::get_time(&t, format.c_str());

  if (issm.fail()) {
    throw std::invalid_argument("failed to parse " + str + " as time");
  }
  return t;
}

std::string stamp2readable(int64_t stamp, const std::string& format,
                           bool with_ms) {
  // 1e9 = 2001-09-09 01:46:40
  // 3e9 = 2065-01-24 05:20:00
  using tp_t = std::chrono::system_clock::time_point;
  tp_t tp;
  if (stamp > 1e9 && stamp < 3e9) {
    tp = tp_t(std::chrono::seconds(stamp));
  } else if (stamp > 1e12 && stamp < 3e12) {
    tp = tp_t(std::chrono::milliseconds(stamp));
  } else if (stamp > 1e15 && stamp < 3e15) {
    tp = tp_t(std::chrono::microseconds(stamp));
  } else if (stamp > 1e18 && stamp < 3e18) {
    tp = tp_t(std::chrono::nanoseconds(stamp));
  } else {
    // we don't know what it is
    return std::to_string(stamp);
  }

  std::time_t time = std::chrono::system_clock::to_time_t(tp);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                tp.time_since_epoch())
                .count() %
            1000;
  std::tm* t = std::localtime(&time);

  std::stringstream ssm;
  ssm << std::put_time(t, format.c_str());
  if (with_ms) {
    ssm << "." << std::setiosflags(std::ios::right) << std::setw(3)
        << std::setfill('0') << ms;
  }
  return ssm.str();
}

std::string formatDuration(int64_t duration_ns, int precision) {
  static const std::map<int64_t, std::string> units{
      {kNsInHour, "h"}, {kNsInMin, "min"}, {kNsInSec, "s"},
      {kNsInMs, "ms"},  {kNsInUs, "us"},   {1, "ns"}};
  std::ostringstream stream;

  for (auto it = units.rbegin(); it != units.rend(); it++) {
    if (0 != duration_ns / it->first) {
      stream << std::fixed << std::setprecision(precision)
             << duration_ns / static_cast<double>(it->first) << it->second;
      return stream.str();
    }
  }
  return "0";
}

}  // namespace util
}  // namespace worldview