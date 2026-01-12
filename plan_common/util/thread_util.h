

#ifndef ONBOARD_LITE_UTILS_THREAD_UTIL_H_
#define ONBOARD_LITE_UTILS_THREAD_UTIL_H_

#include <mutex>
#include <string>
#include <unordered_map>

#include "plan_common/base/singleton.h"

namespace st {

#define QCONTEXT_THREAD_NAME_SET(thread_name) \
  st::ThreadUtil::Instance()->QSetThreadName(thread_name);

#define QCONTEXT_THREAD_NAME_GET(pid) \
  st::ThreadUtil::Instance()->QGetThreadName(pid);

class ThreadUtil {
 public:
  void QSetThreadName(const std::string& name);
  const std::string QGetThreadName(int pid);

 private:
  std::mutex mutex_;
  std::unordered_map<int, std::string> pid_to_name;

  DECLARE_SINGLETON(ThreadUtil);
};

}  // namespace st

#endif  // ONBOARD_LITE_UTILS_THREAD_UTIL_H_
