#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <unistd.h>

namespace worldview {
namespace util {
using Callback = std::function<void(std::string&)>;

// execute a command, return its retcode and output
std::pair<int, std::string> execCommand(const std::string& command,
                                        const Callback& cb = nullptr);
std::pair<int, std::string> execCommandWithTimeout(
    const std::string& command, int timeoutSeconds,
    const Callback& cb = nullptr);

// start a command and return its pid
pid_t startCommand(const std::string& command);
std::vector<pid_t> getDescendants(pid_t ancestor);
void killDescendants(pid_t ancestor, int sig);

bool is_number(const std::string& s);
std::vector<pid_t> get_pids_by_command(const std::string& search_string);
std::string getenv(const std::string& var);

enum ProcessState {
  UNKNOWN,
  RUNNING,
  SLEEPING,
  DISK_SLEEP,
  STOPPED,
  TRACING_STOP,
  ZOMBIE,
  DEAD
};

std::string to_string(ProcessState);

struct ProcessInfo {
  using Ptr = std::shared_ptr<ProcessInfo>;
  std::string work_dir;
  std::string command;
  ProcessState state;
  int64_t virtual_mem_kb{0};
  int64_t resident_mem_kb{0};
  int64_t shared_mem_kb{0};
  double cpu_percent{0};
  double mem_percent{0};
};

ProcessInfo::Ptr getProcessInfo(pid_t pid);

}  // namespace util
}  // namespace worldview