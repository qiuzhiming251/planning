

#include <cerrno>
#include <cstring>
#include <mutex>
#include <string>
#include <cctype>
#include <thread>

#include <glog/logging.h>
#include <pthread.h>
#include <unistd.h>
#include <sched.h>
#include <sys/resource.h>

#include "absl/strings/str_format.h"
#include "plan_common/async/thread_pool.h"
#include "plan_common/gflags.h"
#include "gflags/gflags.h"
#include "plan_common/log.h"
#include "plan_common/util/thread_util.h"
#include "plan_common/log.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"

namespace st {
namespace {
int ToPOSIXPolicy(ThreadPool::SchedulePolicy policy) {
  return static_cast<int>(policy);
}

std::pair<int, int> GetPriorityMinMax(ThreadPool::SchedulePolicy policy) {
  int posix_policy = ToPOSIXPolicy(policy);
  return std::make_pair(sched_get_priority_min(posix_policy),
                        sched_get_priority_max(posix_policy));
}

int ToPOSIXPriority(ThreadPool::SchedulePolicy policy,
                    ThreadPool::SchedulePriority priority) {
  const auto& [min_value, max_value] = GetPriorityMinMax(policy);
  const int span = max_value - min_value;
  switch (priority) {
    case ThreadPool::SchedulePriority::NO_VALUE:
      return 0;
    case ThreadPool::SchedulePriority::HIGH:
      return min_value;
    case ThreadPool::SchedulePriority::VERY_HIGH:
      return min_value + 0.5 * span;
    case ThreadPool::SchedulePriority::ULTRA_HIGH:
      return max_value;
    default:
      return 0;
  }
}
}  // namespace

ThreadPool::ThreadPool(int num_workers, const char* thd_name,
                       const std::function<void(int index)>& init_thread) {
  CHECK_GE(num_workers, 0);
  const char* actual_name =
      (thd_name && *thd_name != '\0') ? thd_name : "ThreadPool";
  for (int index = 0; index < num_workers; ++index) {
    std::string thread_name = absl::StrFormat("%s-%d", actual_name, index);
    workers_.emplace_back([this, index, init_thread, actual_name, thread_name] {
      // QCONTEXT_THREAD_NAME_SET("ThreadPool");
      pthread_setname_np(pthread_self(), thread_name.c_str());
      if (init_thread) {
        init_thread(index);
      }

      while (true) {
        std::function<void()> task;
        {
          absl::MutexLock lock(&mutex_);

          while (!stop_requested_ && tasks_.empty()) {
            cond_var_.Wait(&mutex_);
          }
          if (stop_requested_ && tasks_.empty()) {
            return;
          }
          task = std::move(tasks_.front());
          tasks_.pop();
        }
        task();
      }
    });
    apollo::cyber::scheduler::Instance()->SetInnerThreadAttr(thread_name,
                                                             &workers_[index]);
  }

  // BindCore();
}

ThreadPool::~ThreadPool() { Drain(); }

void ThreadPool::Drain(void) {
  std::call_once(drain_flag_, [&] {
    {
      absl::MutexLock lock(&mutex_);
      stop_requested_ = true;
      cond_var_.SignalAll();
    }
    for (std::thread& worker : workers_) {
      worker.join();
    }
  });
}

ThreadPool* ThreadPool::DefaultPool() {
  static ThreadPool* default_pool =
      new ThreadPool(FLAGS_ad_byd_prediction_pool_size, "DefaultPool");
  return default_pool;
}

ThreadPool* ThreadPool::MapDefaultPool() {
  static ThreadPool* default_pool = new ThreadPool(2, "MapDefaultPool");
  return default_pool;
}

ThreadPool* ThreadPool::DisposalPool() {
  static ThreadPool* disposal_pool = new ThreadPool(1, "DisposalPool");
  return disposal_pool;
}

absl::Status ThreadPool::SetScheduleParam(SchedulePolicy policy,
                                          SchedulePriority priority) {
  sched_param sch;
  sch.sched_priority = ToPOSIXPriority(policy, priority);
  int posix_policy = ToPOSIXPolicy(policy);
  for (auto& worker : workers_) {
    if (int ret =
            pthread_setschedparam(worker.native_handle(), posix_policy, &sch);
        ret != 0) {
      // The ret code are tested on Linux
      switch (ret) {
        case EPERM:
          return absl::PermissionDeniedError(
              "Setting thread pool schedule policy");
        case EINVAL:
          return absl::InvalidArgumentError(
              absl::StrFormat("policy is not a recognized policy, or priority "
                              "does not make sense "
                              "for the policy. policy: %d, priority: %d",
                              posix_policy, sch.sched_priority));
        default:
          return absl::UnknownError(absl::StrFormat(
              "Failed to setschedparam: %s. policy: %d, priority: %d",
              std::strerror(errno), posix_policy, sch.sched_priority));
      }
    }
  }
  return absl::OkStatus();
}

void ThreadPool::BindCore(void) {
  int cores_num = FLAGS_planner_thread_pool_bind_cores.size();
  if (cores_num == 0) {
    LWARN("bind cores is empty.");
    return;
  }

  std::vector<int> cpu_list;
  cpu_list.reserve(cores_num);
  for (int i = 0; i < cores_num; i++) {
    if (std::isdigit(FLAGS_planner_thread_pool_bind_cores[i])) {
      cpu_list.emplace_back(FLAGS_planner_thread_pool_bind_cores[i] - '0');
      LWARN("core num: %d", cpu_list.back());
    }
  }

  int cpu_idx = 0;
  for (auto& worker : workers_) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu_list[cpu_idx], &set);
    pthread_setaffinity_np(worker.native_handle(), sizeof(set), &set);
    cpu_idx = (cpu_idx + 1) % cpu_list.size();
    LWARN("fsd-planning",
          "fsd-planning thread_pool add thread in spec cpu, id: %" PRIu64,
          worker.get_id());
  }
}
}  // namespace st
