

#include <unordered_map>

#include <syscall.h>  // IWYU pragma: keep
#include <unistd.h>

#include "absl/strings/str_cat.h"
#include "plan_common/util/file_util.h"
#include "plan_common/util/thread_util.h"

namespace st {

pid_t gettid(void) { return static_cast<pid_t>(syscall(__NR_gettid)); }

ThreadUtil::ThreadUtil() {}

void ThreadUtil::QSetThreadName(const std::string& name) {
  std::lock_guard<std::mutex> lock(mutex_);
  pid_t pid = gettid();
  pid_to_name[pid] = name;
}

const std::string ThreadUtil::QGetThreadName(int pid) {
  std::string str;
  std::lock_guard<std::mutex> lock(mutex_);
  if (pid_to_name.find(pid) != pid_to_name.end()) {
    str = pid_to_name[pid];
  } else {
    std::string thread_name;
    std::string thread_file(absl::StrCat("/proc/", pid, "/comm"));
    file_util::GetFileContentByGetline(thread_file, &thread_name);
    str = thread_name.substr(0, thread_name.length() - 1);
    pid_to_name[pid] = str;
  }
  return str;
}

}  // namespace st
