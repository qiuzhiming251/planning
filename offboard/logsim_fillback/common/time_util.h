#pragma once
#include <chrono>
#include <string>

namespace worldview {
namespace util {

std::string formatTime(const std::string& format = "%Y-%m-%d %H:%M:%S",
                       const std::chrono::system_clock::time_point& tp =
                           std::chrono::system_clock::now());
std::tm parseTime(const std::string& str,
                  const std::string& format = "%Y-%m-%d %H:%M:%S");
std::string stamp2readable(int64_t stamp,
                           const std::string& format = "%Y-%m-%d %H:%M:%S",
                           bool with_ms = true);
std::string formatDuration(int64_t duration_ns, int precision = 0);

}  // namespace util
}  // namespace worldview
