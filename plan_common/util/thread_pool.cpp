

#include "plan_common/util/thread_pool.h"

namespace ad_byd {
namespace planning {

void ThreadPool::stop() {
  _io.stop();
  _thread_group.join_all();
  _stopped = true;
}

ThreadPool::~ThreadPool() {
  if (!_stopped) {
    try {
      stop();
    } catch (std::exception& e) {
      LERROR("stop thread pool failed. %s", e.what());
    }
  }
}

ThreadPool::ThreadPool(int num, int next_thread_pool_level)
    : ThreadPool(num, next_thread_pool_level, 0) {}

ThreadPool::ThreadPool(int num, int next_thread_pool_level, int pool_index)
    : _work(_io) {
  LINFO("Starting thread pool, thread_num = %d, index = %d", num, pool_index);
  for (int i = 0; i != num; ++i) {
    _thread_group.create_thread([this, next_thread_pool_level, pool_index, i] {
      GlobalThreadPool::s_thread_pool_level = next_thread_pool_level;
      GlobalThreadPool::s_thread_pool_index = pool_index;
      this->_io.run();
    });
  }
}

ThreadPool::ThreadPool(int num)
    : ThreadPool(num, GlobalThreadPool::s_thread_pool_level) {}

thread_local int GlobalThreadPool::s_thread_pool_level = 0;
thread_local int GlobalThreadPool::s_thread_pool_index = 0;

#define CASE_LEVEL_THREAD_POOL(N, INDEX) \
  case N:                                \
    return LevelThreadPool<N, INDEX>::instance();

#define CASE_LEVEL_THREAD_POOL_0(N) CASE_LEVEL_THREAD_POOL(N, 0)
#define CASE_LEVEL_THREAD_POOL_1(N) CASE_LEVEL_THREAD_POOL(N, 1)
#define CASE_LEVEL_THREAD_POOL_2(N) CASE_LEVEL_THREAD_POOL(N, 2)

ThreadPool* GlobalThreadPool::instance() {
  int pool_level = s_thread_pool_level;
  CHECK_LT(pool_level, MAX_THREAD_POOL_LEVEL)
      << "max supported pool level is 4";
  int index = s_thread_pool_index;
  switch (index) {
    case 0:
      switch (pool_level) {
        PLAN_REPEAT(MAX_THREAD_POOL_LEVEL, CASE_LEVEL_THREAD_POOL_0)
      }
      [[fallthrough]];
    case 1:
      switch (pool_level) {
        PLAN_REPEAT(MAX_THREAD_POOL_LEVEL, CASE_LEVEL_THREAD_POOL_1)
      }
      [[fallthrough]];
    case 2:
      switch (pool_level) {
        PLAN_REPEAT(MAX_THREAD_POOL_LEVEL, CASE_LEVEL_THREAD_POOL_2)
      }
  }
  CHECK(false) << "pool level " << pool_level << " is invalid";

  return nullptr;
}

void GlobalThreadPool::set_index(int index) { s_thread_pool_index = index; }

}  // namespace planning
}  // namespace ad_byd
