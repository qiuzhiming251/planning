

#ifndef AD_BYD_PLANNING_UTILS_THREAD_POOL_H
#define AD_BYD_PLANNING_UTILS_THREAD_POOL_H

#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <iterator>
#include <string>

#include "boost/asio.hpp"
#include "boost/thread.hpp"
#include "plan_common/log.h"
//#include "delay_clean.h"
#include "plan_common/gflags.h"
#include "plan_common/planning_macros.h"
#include "plan_common/util/logger_stack.h"

namespace ad_byd {
namespace planning {

class ThreadPool {
 public:
  ThreadPool(int number);

  ThreadPool(int number, int next_thread_pool_level);

  ThreadPool(int number, int next_thread_pool_level, int thread_pool_index);

  template <typename InputIter, typename OutputIter, typename F>
  OutputIter transform(InputIter begin, InputIter end, OutputIter out, F f) {
    typedef std::future<decltype(f(*begin))> Future;
    std::vector<Future> futures;
    for (auto i = begin; i != end; ++i) {
      auto& num = *i;
      futures.emplace_back(this->post([&] { return f(num); }));
    }
    bool succ = true;
    for (auto& future : futures) {
      try {
        *(out++) = future.get();
      } catch (std::exception& e) {
        LINFO("exception occured: %s", e.what());
        LoggerStack::PrintLoggerStackTrace();
        succ = false;
      } catch (...) {
        LINFO("Unknown exception occured");
        LoggerStack::PrintLoggerStackTrace();
        succ = false;
      }
    }
    CHECK(succ) << "excpetion occured in thread pool user function";
    return out;
  }

  template <typename InputIter, typename F>
  void for_each(InputIter begin, InputIter end, F f) {
    if (std::distance(begin, end) == 1) {
      f(*begin);
      return;
    }
    typedef std::future<void> Future;
    std::vector<Future> futures;
    for (auto i = begin; i != end; ++i) {
      auto& num = *i;
      futures.emplace_back(this->post([&] { f(num); }));
    }
    bool succ = true;
    for (auto& future : futures) {
      if (future.wait_for(std::chrono::seconds(
              FLAGS_ad_byd_planning_future_wait_timeout)) ==
          std::future_status::timeout) {
        CHECK(false) << "future wait timeout";
      }
      try {
        future.get();
      } catch (std::exception& e) {
        LINFO("exception occured: %s", e.what());
        LoggerStack::PrintLoggerStackTrace();
        succ = false;
      } catch (...) {
        LINFO("Unknown exception occured");
        LoggerStack::PrintLoggerStackTrace();
        succ = false;
      }
    }
    CHECK(succ) << "excpetion occured in thread pool user function";
  }

  template <typename FuncType>
  std::future<typename std::result_of<FuncType()>::type> post(FuncType&& func) {
    // keep in mind that std::result_of is std::invoke_result in C++17
    typedef typename std::result_of<FuncType()>::type return_type;
    typedef typename std::packaged_task<return_type()> task_type;
    // since post requires that the functions in it are copy-constructible,
    // we use a shared pointer for the packaged_task since it's only movable and
    // non-copyable
    std::shared_ptr<task_type> task =
        std::make_shared<task_type>(std::move(func));
    std::future<return_type> returned_future = task->get_future();
    uint64_t father_logger_id = LoggerStack::GetStack().current_id();

    // note: variables eg. `task` must be copied here because of the lifetime
    _io.post([=] {
      LoggerStack::SetThreadFatherId(father_logger_id);
      (*task)();
    });
    return returned_future;
  }

  template <typename F>
  auto delay_clean(F&& f) {
    return post(std::move(f));
  }

  void stop();

  ~ThreadPool();

 private:
  boost::thread_group _thread_group;
  boost::asio::io_service _io;
  boost::asio::io_service::work _work;
  bool _stopped = false;
};

// leveled thread pool capacity
#define MAX_THREAD_POOL_LEVEL 5
#define MAX_THREAD_POOL_INDEX 3
const static int
    THREAD_POOL_CAPACITY[MAX_THREAD_POOL_INDEX][MAX_THREAD_POOL_LEVEL] = {
        {
            10, 10, 10, 10, 10  // index 0 threads
        },
        {
            10, 10, 10, 10, 10  // index 1 threads
        },
        {
            10, 10, 10, 10, 10  // index 2 threads
        }};

// Do not use this class directly, use GlobalThreadPool::instance() instead
template <int LEVEL, int INDEX>
class LevelThreadPool : public ThreadPool {
 public:
  static LevelThreadPool* instance() {
    static LevelThreadPool<LEVEL, INDEX> pool;
    return &pool;
  }

 private:
  LevelThreadPool()
      : ThreadPool(THREAD_POOL_CAPACITY[INDEX][LEVEL], LEVEL + 1, INDEX) {
    LINFO("index %d level %d thread pool capacity = %d", INDEX, LEVEL,
          THREAD_POOL_CAPACITY[INDEX][LEVEL]);
  }
};

class GlobalThreadPool {
 public:
  static ThreadPool* instance();
  static void set_index(int index);

  static thread_local int s_thread_pool_level;
  static thread_local int s_thread_pool_index;
};

namespace parallel {

template <typename InputIter, typename OutputIter, typename F>
OutputIter transform(InputIter begin, InputIter end, OutputIter out, F f) {
  return GlobalThreadPool::instance()->transform(begin, end, out, f);
}

template <typename InputIter, typename F>
void for_each(InputIter begin, InputIter end, F f) {
  GlobalThreadPool::instance()->for_each(begin, end, f);
}

template <typename F>
void for_each(size_t n, F f) {
  std::vector<size_t> ranges(n);
  for (size_t i = 0; i != n; ++i) {
    ranges[i] = i;
  }
  GlobalThreadPool::instance()->for_each(ranges.begin(), ranges.end(), f);
}

template <typename InputIter, typename F>
void for_each_with_index(InputIter begin, InputIter end, F f) {
  std::vector<std::pair<size_t, InputIter>> ranges;
  size_t count = 0;
  for (auto i = begin; i != end; ++i, ++count) {
    ranges.push_back(std::make_pair(count, i));
  }
  GlobalThreadPool::instance()->for_each(
      ranges.begin(), ranges.end(),
      [&](const std::pair<size_t, InputIter>& pair) {
        f(pair.first, *(pair.second));
      });
}

}  // namespace parallel

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_UTILS_THREAD_POOL_H

#define DELAY_CLEAN(p) GlobalThreadPool::instance()->delay_clean(p)
