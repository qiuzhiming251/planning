#pragma once
#ifdef USE_DROGON_LOG

#include <trantor/utils/Logger.h>

namespace worldview {
using LogStream = trantor::LogStream;
using Logger = trantor::Logger;
using LogLevel = trantor::Logger::LogLevel;

static inline void showInfoLog() {
  trantor::Logger::setLogLevel(trantor::Logger::LogLevel::kInfo);
}
static inline void showErrorLog() {
  trantor::Logger::setLogLevel(trantor::Logger::LogLevel::kError);
}
}  // namespace worldview

#else

#include <glog/logging.h>
#define LOG_TRACE VLOG(1)
#define LOG_DEBUG DLOG(INFO)
#define LOG_INFO LOG(INFO)
#define LOG_ERROR LOG(ERROR)
#define LOG_WARN LOG(WARNING)

namespace worldview {
static inline void showInfoLog() { FLAGS_minloglevel = 0; }
static inline void showErrorLog() { FLAGS_minloglevel = 2; }
}  // namespace worldview
#endif