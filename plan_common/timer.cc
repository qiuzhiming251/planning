#include <iomanip>
#include <map>
#include <sstream>
#include <chrono>
#include <thread>
#include <sys/resource.h>
#include <sys/syscall.h>
#include "plan_common/log.h"
#include <time.h>
#include "plan_common/timer.h"
namespace st {

static constexpr int64_t kNsInUs = 1000;
static constexpr int64_t kNsInMs = 1000 * kNsInUs;
static constexpr int64_t kNsInSec = 1000 * kNsInMs;
static constexpr int64_t kNsInMin = 60 * kNsInSec;
static constexpr int64_t kNsInHour = 60 * kNsInMin;

Timeline::Timeline(const std::string& name) : name_(name) {
  // start_time_ = std::chrono::duration_cast<std::chrono::microseconds>(
  //     std::chrono::system_clock::now().time_since_epoch()).count();
  // pid_ = getpid();
  // tid_ = static_cast<int>(syscall(SYS_gettid));
  // start_cpu_time_ = GetThreadCpuTimeUs();
}

Timeline::~Timeline() {
  // uint64_t end_time = std::chrono::duration_cast<std::chrono::microseconds>(
  //     std::chrono::system_clock::now().time_since_epoch()).count();
  // uint64_t end_cpu_time_ = GetThreadCpuTimeUs();
  // uint64_t delta_time = end_time - start_time_;
  // int cpu_perc = static_cast<int>((end_cpu_time_ - start_cpu_time_) * 100 /
  // delta_time); LOG_INFO << "Timeline: {\"name\": \"" << name_ << "_cpu" <<
  // cpu_perc << "\", \"ph\": \"X\", \"pid\": "
  //       << pid_ << ", \"tid\": " << tid_ << ", \"ts\": " << start_time_
  //       << ", \"dur\": " << delta_time << "}";
}

uint64_t Timeline::GetThreadCpuTimeUs() {
  struct timespec cpu_ts;
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_ts);
  return cpu_ts.tv_sec * 1e6 + cpu_ts.tv_nsec / 1e3;
}

Timer::Timer(const std::string& name, bool log_auto)
    : start_time_(std::chrono::system_clock::now()),
      name_(name),
      log_auto_(log_auto) {}

Timer::~Timer() {
  if (log_auto_) {
    Log();
  }
}

void Timer::Reset(const std::string& name) {
  Reset();
  name_ = name;
}

void Timer::Reset() {
  if (log_auto_) {
    Log();
  }
  start_time_ = std::chrono::system_clock::now();
}

void Timer::Log() {
  if (!name_.empty()) {
    // std::ostringstream ossm;
    // ossm << "[timestat] " << name_ << " " << WrapDuration(TimeNs());
    // byd::log::Warn("fsd-planning", "%s", ossm.str().c_str());
    LOG_INFO << "[timestat] " << name_ << " " << WrapDuration(TimeNs());
  }
}

std::string Timer::WrapDuration(int64_t ns) {
  static const std::map<int64_t, std::string> units{
      {kNsInHour, "h"}, {kNsInMin, "min"}, {kNsInSec, "s"},
      {kNsInMs, "ms"},  {kNsInUs, "us"},   {1, "ns"}};
  std::ostringstream stream;

  for (auto it = units.rbegin(); it != units.rend(); it++) {
    if (0 != ns / it->first) {
      stream << std::fixed << std::setprecision(2)
             << ns / static_cast<double>(it->first) << it->second;
      return stream.str();
    }
  }
  return "0";
}

const std::chrono::system_clock::time_point& Timer::start_time() const {
  return start_time_;
}

}  // namespace st
