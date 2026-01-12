#include <unistd.h>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <queue>
#include <regex>
#include <sstream>
#include <future>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <cstring>
#include <thread>
#include <sys/wait.h>

#include <fstream>
#include <string>
#include <vector>
#include <dirent.h>
#include <cctype>
#include <algorithm>

#include "common/command_util.h"
#include "common/log.h"
#include "common/string_util.h"

namespace worldview {
namespace util {

std::pair<int, std::string> execCommand(const std::string &command,
                                        const Callback &cb) {
  std::vector<char> buffer(128);
  std::string result;
  int retcode = 0;
  auto close = [&retcode](FILE *f) { retcode = WEXITSTATUS(pclose(f)); };
  {
    std::unique_ptr<FILE, decltype(close)> pipe(popen(command.c_str(), "r"),
                                                close);
    if (!pipe) {
      throw std::runtime_error("popen failed for " + command);
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
      // LOG_ERROR << " execCommandWithTimeout buffer:" << buffer.data();
      result += buffer.data();
      std::string strResult(buffer.data());
      if (cb) {
        // LOG_ERROR << "call back function progress:" << progress;
        cb(strResult);
      }
    }
  }
  return std::make_pair(retcode, result);
}

std::pair<int, std::string> execCommandWithTimeout(const std::string &command,
                                                   int timeoutSeconds,
                                                   const Callback &cb) {
  // LOG_ERROR << " execCommandWithTimeout start...";
  std::string result;
  int retcode = -1;

  auto future = std::async(std::launch::async, execCommand, command, cb);
  auto status = future.wait_for(std::chrono::seconds(timeoutSeconds));
  if (status == std::future_status::ready) {
    // LOG_ERROR << " execCommandWithTimeout ok";
    result = "file down ok";
    retcode = 0;
  } else {
    LOG_ERROR << " execCommandWithTimeout timeout";
    result = "file down timeout";
    retcode = -2;
  }

  return {retcode, result};
}

pid_t startCommand(const std::string &command) {
  pid_t pid = fork();
  if (pid == 0) {
    // child
    char *const args[] = {const_cast<char *>("bash"), const_cast<char *>("-c"),
                          const_cast<char *>(command.c_str()), NULL};

    auto ret = execvp("bash", args);
    if (ret != 0) {
      // fatal error
      LOG_ERROR << "failed to run execvp";
      exit(ret);
    }
  }
  return pid;
}

std::vector<pid_t> getDescendants(pid_t ancestor) {
  std::vector<pid_t> res;
  // bfs
  std::queue<pid_t> q;
  q.push(ancestor);
  while (!q.empty()) {
    auto pid = q.front();
    res.push_back(pid);
    q.pop();
    auto ret = execCommand("pgrep -P " + std::to_string(pid));
    LOG_DEBUG << "children of " << pid << ": " << ret.second;
    std::stringstream ssm(ret.second);
    pid_t child_pid{0};
    while (ssm >> child_pid) {
      q.push(child_pid);
    }
  }
  return res;
}

bool is_number(const std::string &s) {
  return !s.empty() && std::all_of(s.begin(), s.end(), [](char c) {
    return std::isdigit(static_cast<unsigned char>(c));
  });
}

std::vector<pid_t> get_pids_by_command(const std::string &search_string) {
  std::vector<pid_t> pids;
  DIR *dir = opendir("/proc");

  if (!dir) {
    perror("opendir failed");
    return pids;
  }

  struct dirent *entry;
  while ((entry = readdir(dir)) != nullptr) {
    if (entry->d_type != DT_DIR) continue;

    std::string dir_name(entry->d_name);
    if (!is_number(dir_name)) continue;

    pid_t pid = std::stoi(dir_name);
    std::string cmdline_path = "/proc/" + dir_name + "/cmdline";

    std::ifstream cmdline_file(cmdline_path);
    if (!cmdline_file) continue;

    std::string cmdline;
    std::getline(cmdline_file, cmdline);

    std::replace(cmdline.begin(), cmdline.end(), '\0', ' ');

    if (cmdline.find(search_string) != std::string::npos) {
      pids.push_back(pid);
    }
  }

  closedir(dir);
  return pids;
}

void killDescendants(pid_t ancestor, int sig) {
  if (ancestor != 0) {
    auto descendants = getDescendants(ancestor);
    LOG_INFO << "killing processes " << util::join(descendants, ",");
    std::string command =
        "ps -p " + util::join(descendants, ",") + " -o pid,command";
    auto ret = execCommand(command);
    LOG_INFO << ret.second;
    // kill from the most remote descendant
    for (auto it = descendants.rbegin(); it != descendants.rend(); ++it) {
      kill(*it, sig);
    }
  }
}

std::string getenv(const std::string &var) {
  const char *p = std::getenv(var.c_str());
  if (p) {
    return std::string(p);
  }
  return "";
}

ProcessState parseState(const std::string &s) {
  if (s.length() == 0) {
    return UNKNOWN;
  }
  switch (s[0]) {
    case 'R':
      return RUNNING;
    case 'S':
      return SLEEPING;
    case 'D':
      return DISK_SLEEP;
    case 'T':
      return STOPPED;
    case 't':
      return TRACING_STOP;
    case 'Z':
      return ZOMBIE;
    case 'X':
      return DEAD;
    default:
      return UNKNOWN;
  }
}

std::string to_string(ProcessState s) {
#define TO_STRING(NAME) \
  case NAME:            \
    return #NAME
  switch (s) {
    TO_STRING(UNKNOWN);
    TO_STRING(RUNNING);
    TO_STRING(SLEEPING);
    TO_STRING(DISK_SLEEP);
    TO_STRING(STOPPED);
    TO_STRING(TRACING_STOP);
    TO_STRING(ZOMBIE);
    TO_STRING(DEAD);
    default:
      return "UNKNOWN";
  }
}

/* Top output:
    PID USER      PR  NI    VIRT    RES    SHR S  %CPU  %MEM     TIME+ COMMAND
 327490 lijing17  20   0 3378964   1.3g  11116 S 100.0   9.7  39:04.20
 /home/lijing17/.vscode/exten+
*/

ProcessInfo::Ptr getProcessInfo(pid_t pid) {
  std::stringstream ssm;
  ssm << "top -bcn 1 -p " << pid << " -w 512 | grep " << pid << " | tr -s ' '";
  std::string command = ssm.str();

  auto ret = execCommand(command);
  if (ret.first != 0) {
    LOG_ERROR << "exec " << command << " err, " << ret.first << " "
              << ret.second;
    return nullptr;
  }
  auto lines = split(ret.second, '\n');
  for (auto &line : lines) {
    auto segments = split(trim(line), ' ');
    if (segments.size() < 12) {
      LOG_ERROR << "unexpected result: " << line;
      continue;
    }
    if (segments[0] == std::to_string(pid)) {
      try {
        auto info = std::make_shared<ProcessInfo>();

        info->virtual_mem_kb = std::stoll(segments[4]);
        info->resident_mem_kb = std::stoll(segments[5]);
        info->shared_mem_kb = std::stoll(segments[6]);
        info->state = parseState(segments[7]);
        info->cpu_percent = std::stod(segments[8]);
        info->mem_percent = std::stod(segments[9]);
        std::vector<std::string> command_segs(segments.begin() + 11,
                                              segments.end());
        info->command = join(command_segs, ' ');

        auto cwd_ret =
            execCommand("readlink -f /proc/" + std::to_string(pid) + "/cwd");
        if (cwd_ret.first != 0) {
          LOG_ERROR << "read link err " << ret.first << " " << ret.second;
        } else {
          info->work_dir = trim(cwd_ret.second);
        }

        return info;
      } catch (std::exception &e) {
      }
    }
  }
  LOG_ERROR << "process " << pid << " not found";
  return nullptr;
}

}  // namespace util
}  // namespace worldview