#ifndef ST_PLANNING_COMMON_TIMER
#define ST_PLANNING_COMMON_TIMER
#include <chrono>
#include <string>
namespace st {
class Timeline {
 public:
  Timeline(const std::string& name);
  ~Timeline();

  uint64_t GetThreadCpuTimeUs();

 private:
  uint64_t start_time_;
  int pid_;
  int tid_;
  uint64_t start_cpu_time_;
  std::string name_;
};

class Timer {
 public:
  Timer(const std::string& name, bool log_auto = true);
  Timer() : Timer("", false){};
  ~Timer();

  double TimeS() const { return static_cast<double>(TimeMs()) / 1e3; }

  int64_t TimeMs() const { return TimeSinceStart<std::chrono::milliseconds>(); }
  int64_t TimeUs() const { return TimeSinceStart<std::chrono::microseconds>(); }
  int64_t TimeNs() const { return TimeSinceStart<std::chrono::nanoseconds>(); }

  std::string Time() const { return WrapDuration(TimeNs()); }

  const std::chrono::system_clock::time_point& start_time() const;

  void Reset(const std::string& name);
  void Reset();

 private:
  void Log();

  template <typename T>
  int64_t TimeSinceStart() const {
    return std::chrono::duration_cast<T>(std::chrono::system_clock::now() -
                                         start_time_)
        .count();
  }

  static std::string WrapDuration(int64_t ns);

  std::chrono::system_clock::time_point start_time_;
  std::string name_;
  bool log_auto_{true};
};

}  // namespace st

#define CAT(a, b) a##b
// #define NAME_CAT(a, b) CAT(a, b)
#define TIMELINE(str)

#endif  // ST_PLANNING_COMMON_TIMER
