#pragma once
#include <chrono>

namespace worldview {
namespace util {
class Timer {
 public:
  Timer() : start_time_(std::chrono::system_clock::now()) {}

  template <typename T>
  int64_t time_since_start() const {
    return std::chrono::duration_cast<T>(std::chrono::system_clock::now() -
                                         start_time_)
        .count();
  }

  int64_t time_ms() const {
    return time_since_start<std::chrono::milliseconds>();
  }
  int64_t time_us() const {
    return time_since_start<std::chrono::microseconds>();
  }
  int64_t time_ns() const {
    return time_since_start<std::chrono::nanoseconds>();
  }
  void reset() { start_time_ = std::chrono::system_clock::now(); }

  void advance(int64_t ns) { start_time_ += std::chrono::nanoseconds(ns); }

  const std::chrono::system_clock::time_point& start_time() const {
    return start_time_;
  }

 private:
  std::chrono::system_clock::time_point start_time_;
};
}  // namespace util
}  // namespace worldview