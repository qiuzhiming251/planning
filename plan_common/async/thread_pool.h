

#ifndef ST_PLANNING_ASYNC_THREAD_POOL
#define ST_PLANNING_ASYNC_THREAD_POOL

#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <gflags/gflags.h>
#include <sched.h>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "plan_common/async/future.h"

namespace st {

// A thread pool implementation.
// This class is thread safe.
class ThreadPool {
 public:
  // POSIX systems support more schedule policy, add as required. For details,
  // see `man 7 sched`.
  // DEFAULT corresponds to Linux's SCHED_OTHER (the default)
  // REALTIME corresponds to Linux's SCHED_RR
  // LOW corresponds to Linux's SCHED_IDLE, it basically means run when the
  // system is idle.
  // NOTE: REALTIME is a CPU hungry monster, use with caution!!! The
  // threads with REALTIME policy will always preempt lower priority threads
#if defined(__linux__) || defined(__unix__)
  enum class SchedulePolicy {
    DEFAULT = SCHED_OTHER,
    REALTIME = SCHED_RR,
    LOW = SCHED_IDLE
  };
  // Only SCHED_REALTIME suports adjusting priority, other SchedPolicy should
  // use NO_VALUE. In principle, SCHED_REALTIME always means higher priority
  // compared to other non-realtime policies.
  enum class SchedulePriority { NO_VALUE, HIGH, VERY_HIGH, ULTRA_HIGH };
#endif
  // TODO: The init_thread func here is used to install lite context so
  // that we can locate the module that each thread resides in. It won't be
  // needed when we run each module in a standalone process.
  // When num_workers is zero, this thread pool becomes an inline thread pool,
  // and any task schedule to this pool will be implemented in place.
  explicit ThreadPool(int num_workers, const char* thd_name,
                      const std::function<void(int index)>& init_thread = {});
  ~ThreadPool();

  // Return the default thread pool.
  static ThreadPool* DefaultPool();

  // used by Map Msg UserCallback
  static ThreadPool* MapDefaultPool();

  // Return the disposal thread pool to destroy stuff asynchronously.
  static ThreadPool* DisposalPool();

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  int NumWorkers() const { return workers_.size(); }

  // Sets scheduling policy and priority. SCHED_REALTIME needs root permission.
  // Not all SchedulePolicy supports adjusting priority, pls refer to their
  // definition or Linux `man 7 sched` for more details.
#if defined(__linux__) || defined(__unix__)
  absl::Status SetScheduleParam(SchedulePolicy policy,
                                SchedulePriority priority);
#endif

  template <class Func, class... Args>
  using FutureType = Future<typename std::result_of<Func(Args...)>::type>;

  // Schedule a new task.
  template <class Func, class... Args>
  FutureType<Func, Args...> Schedule(Func&& f, Args&&... args)
      ABSL_LOCKS_EXCLUDED(mutex_);

  // Blocked Drain the remaining tasks
  void Drain(void);

 private:
  void BindCore(void);

 private:
  absl::Mutex mutex_;
  absl::CondVar cond_var_ ABSL_GUARDED_BY(mutex_);

  std::vector<std::thread> workers_;
  // A thread safe queue protected by condition_ and mutex_.
  std::queue<std::function<void()>> tasks_ ABSL_GUARDED_BY(mutex_);
  bool stop_requested_ ABSL_GUARDED_BY(mutex_) = false;
  std::once_flag drain_flag_;
};

template <class Func, class... Args>
ThreadPool::FutureType<Func, Args...> ThreadPool::Schedule(Func&& f,
                                                           Args&&... args) {
  using ReturnType = typename std::result_of<Func(Args...)>::type;
  const auto task = std::make_shared<std::packaged_task<ReturnType()>>(
      std::bind(std::forward<Func>(f), std::forward<Args>(args)...));
  Future<ReturnType> res(task->get_future());

  // If there is no worker, this is an inline thread pool, and the task will be
  // immediately run on the current thread.
  if (workers_.empty()) {
    (*task)();
  } else {
    absl::MutexLock lock(&mutex_);
    // CHECK(!stop_requested_) << "The thread pool has been stopped";
    tasks_.emplace([task]() { (*task)(); });
    cond_var_.Signal();
  }
  return res;
}

}  // namespace st

#endif  // ST_PLANNING_ASYNC_THREAD_POOL
