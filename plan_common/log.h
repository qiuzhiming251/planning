

#ifndef AD_BYD_PLANNING_COMMON_LOG
#define AD_BYD_PLANNING_COMMON_LOG

#include <stdarg.h>

#include <cstdio>
#include <sstream>
#include <string>
//#include <log/log.hpp>
#include "plan_common/gflags.h"
#include "glog/logging.h"
#include "absl/strings/str_cat.h"
#include "cyber/common/log.h"

namespace ad_byd {
namespace planning {
enum LogLevel { DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3, FATAL = 4 };
}  // namespace planning
}  // namespace ad_byd

[[maybe_unused]] static std::string log_mapping(char const *name,
                                                char const *format, ...) {
  const size_t bufferSize = 1024;
  char buffer[bufferSize];
  va_list args;
  va_start(args, format);
  int result = vsnprintf(buffer, bufferSize, format, args);
  va_end(args);
  if (result < 0) {
    throw std::runtime_error("Formatting error");
  } else if (static_cast<size_t>(result) >= bufferSize) {
    throw std::runtime_error("Buffer size is too small");
  }
  return "{[(cnoa_plan)]}" + std::string(name) + ":" +
         std::string(buffer, result);
}

[[maybe_unused]] static std::string const_prefix(const char *file_name,
                                                 int line_no) {
  return absl::StrCat(file_name, ":", line_no);
}

[[maybe_unused]] static std::string const_prefix(char const *module_name,
                                                 char const *file_name,
                                                 int line_no) {
  return absl::StrCat(module_name, ":", file_name, ":", line_no);
}

#define __FILENAME__ \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define MODULE const_prefix("AD-BYD_PLANNING", __FILENAME__, __LINE__).c_str()

#define LLOG_LEVEL(log_level)                                          \
  do {                                                                 \
    FLAGS_ad_byd_planning_log_level = static_cast<int32_t>(log_level); \
  } while (0)

#define LFATAL(args...)                                               \
  do {                                                                \
    if (FLAGS_ad_byd_planning_log_level <= ad_byd::planning::FATAL) { \
      LOG(FATAL) << log_mapping(MODULE, args);                        \
    }                                                                 \
  } while (0)
#define LERROR(args...)                                               \
  do {                                                                \
    if (FLAGS_ad_byd_planning_log_level <= ad_byd::planning::ERROR) { \
      LOG(ERROR) << log_mapping(MODULE, args);                        \
    }                                                                 \
  } while (0)
#define LWARN(args...)                                               \
  do {                                                               \
    if (FLAGS_ad_byd_planning_log_level <= ad_byd::planning::WARN) { \
      LOG(WARNING) << log_mapping(MODULE, args);                     \
    }                                                                \
  } while (0)
#define LINFO(args...)                                               \
  do {                                                               \
    if (FLAGS_ad_byd_planning_log_level <= ad_byd::planning::INFO) { \
      LOG(INFO) << log_mapping(MODULE, args);                        \
    }                                                                \
  } while (0)
#define LDEBUG(args...)                                               \
  do {                                                                \
    if (FLAGS_ad_byd_planning_log_level <= ad_byd::planning::DEBUG) { \
      LOG(INFO) << log_mapping(MODULE, args);                         \
    }                                                                 \
  } while (0)

#define LERROR_NUM_MSG(msg) byd::log::ErrorNumMsg(MODULE, msg)

/*
#define LOG_FATAL(...)         \
  do {                         \
    std::ostringstream oss;    \
    oss << __VA_ARGS__;        \
    LFATAL(oss.str().c_str()); \
  } while (0)

#define LOG_ERROR(...)         \
  do {                         \
    std::ostringstream oss;    \
    oss << __VA_ARGS__;        \
    LERROR(oss.str().c_str()); \
  } while (0)

#define LOG_WARN(...)         \
  do {                        \
    std::ostringstream oss;   \
    oss << __VA_ARGS__;       \
    LWARN(oss.str().c_str()); \
  } while (0)

#define LOG_INFO(...)         \
  do {                        \
    std::ostringstream oss;   \
    oss << __VA_ARGS__;       \
    LINFO(oss.str().c_str()); \
  } while (0)

#define LOG_DEBUG(...)         \
  do {                         \
    std::ostringstream oss;    \
    oss << __VA_ARGS__;        \
    LDEBUG(oss.str().c_str()); \
  } while (0)
*/

#ifdef PLATFORM_X86
#define LOG_FATAL AFATAL
#define LOG_ERROR AERROR
#define LOG_WARN AWARN
#define LOG_INFO AINFO
#define LOG_DEBUG ADEBUG
#define LOGINFO_EVERY(freq) AINFO_EVERY(freq)
#elif defined(PLATFORM_ARM)
#include <ostream>
namespace {
class NullBuffer : public std::streambuf {
 public:
  int overflow(int c) override { return c; }
};

NullBuffer null_buffer;
std::ostream null_stream(&null_buffer);
}  // namespace

#define LOG_FATAL AFATAL
#define LOG_ERROR null_stream
#define LOG_WARN null_stream
#define LOG_INFO null_stream
#define LOG_DEBUG null_stream
#define LOGINFO_EVERY(freq) null_stream
#else
#define LOG_FATAL AFATAL
#define LOG_ERROR AERROR
#define LOG_WARN AWARN
#define LOG_INFO AINFO
#define LOG_DEBUG ADEBUG
#define LOGINFO_EVERY(freq) AINFO_EVERY(freq)
#endif

#endif  // AD_BYD_PLANNING_COMMON_LOG
