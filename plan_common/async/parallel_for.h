

#ifndef ONBOARD_ASYNC_PARALLEL_FOR_H_
#define ONBOARD_ASYNC_PARALLEL_FOR_H_

#include <functional>
#include <iterator>

#include "plan_common/timer.h"
#include "plan_common/async/thread_pool.h"
//#include "plan_common/log.h"

namespace st {

namespace {
// This class creates a thread safe barrier which will block until
// Finished() is called.
class BlockUntilFinished {
 public:
  explicit BlockUntilFinished(int num_iters)
      : num_iters_(num_iters), iters_done_(0) {}

  // Signal the blocking thread that all jobs have finished.
  void Finished(int iters_done) {
    absl::MutexLock lock(&mutex_);
    iters_done_ += iters_done;
    if (iters_done_ == num_iters_) {
      cond_var_.Signal();
    }
  }

  // Block until the finished notification arrives.
  void Block() {
    absl::MutexLock lock(&mutex_);
    while (iters_done_ != num_iters_) {
      cond_var_.Wait(&mutex_);
    }
  }

 private:
  absl::Mutex mutex_;
  absl::CondVar cond_var_ ABSL_GUARDED_BY(mutex_);
  int num_iters_ ABSL_GUARDED_BY(mutex_);
  int iters_done_ ABSL_GUARDED_BY(mutex_);
};

// Shared state between the parallel tasks. Each thread will use this
// information to get the next block of work to be performed.
struct SharedState {
  explicit SharedState(int num_iters) : block_until_finished(num_iters) {}

  std::atomic<int> next_index{0};

  // Used to signal when all the work has been completed. Thread safe.
  BlockUntilFinished block_until_finished;
};

}  // namespace

namespace parallel_for {

struct Options {
  // Each work will get iterations of this block size. If it is zero, the block
  // size is computed as num_iterations / (num_workers * 4).
  int block_size = 0;
};

// Represents the worker index to not be confused with iteration index.
class WorkerIndex {
 public:
  explicit WorkerIndex(int index) : index_(index) {}
  operator int() const { return index_; }

 private:
  const int index_;
};

}  // namespace parallel_for

// Run func() in parallel in the given thread pool. The function func() may have
// one or two arguments: for two-argument version, the first one is the worker
// index, and the second one is the loop index, which is in the range [begin,
// end); for one-argument version, the worker index is omitted. If the thread
// pool is null, the default thread poll will be used.

void ParallelFor(int begin, int end, ThreadPool* thread_pool,
                 const parallel_for::Options& options,
                 std::function<void(parallel_for::WorkerIndex, int)>&& func);
void ParallelFor(int begin, int end, ThreadPool* thread_pool,
                 std::function<void(parallel_for::WorkerIndex, int)>&& func);
void ParallelFor(int begin, int end,
                 std::function<void(parallel_for::WorkerIndex, int)>&& func);

void ParallelFor(int begin, int end, ThreadPool* thread_pool,
                 const parallel_for::Options& options,
                 std::function<void(int)>&& func);
void ParallelFor(int begin, int end, ThreadPool* thread_pool,
                 std::function<void(int)>&& func);
void ParallelFor(int begin, int end, std::function<void(int)>&& func);

template <typename InputIter, typename F>
void ParallelFor(InputIter begin_it, InputIter end_it, F f,
                 ThreadPool* thread_pool = ThreadPool::DefaultPool()) {
  if (begin_it == end_it) return;
  TIMELINE("ParallelForIter");
  int end = std::distance(begin_it, end_it);
  // LOG_WARN << "++++++container_size: " << end;
  if (end == 1) {
    f(*begin_it);
    return;
  }

  auto func = [f = std::move(f)](int worker_index, InputIter it) { f(*it); };
  const int begin = 0;
  const int num_iters = end - begin;
  int block_size =
      std::max<int>(1, num_iters / ((thread_pool->NumWorkers() + 1) << 2));

  auto shared_state = std::make_shared<SharedState>(num_iters);

  // Dynamically assign tasks to all workers for better load balance.
  const auto grab_tasks = [=, &func](parallel_for::WorkerIndex worker_index) {
    // ("ParallelFor::WorkerThread");
    int iters_done = 0;
    while (true) {
      const int index = shared_state->next_index.fetch_add(
                            block_size, std::memory_order_acq_rel) +
                        begin;
      if (index >= end) break;
      for (int i = index; i < std::min(end, index + block_size);
           ++i, ++iters_done) {
        // LOG_WARN << "++++++worker_index: " << worker_index << ", i: " <<
        // i;
        func(worker_index, std::next(begin_it, i));
      }
    }
    shared_state->block_until_finished.Finished(iters_done);
  };

  const int min_num_workers = (num_iters + block_size - 1) / block_size;
  const int num_workers =
      std::min(min_num_workers - 1, thread_pool->NumWorkers());
  // Schedule futures.
  for (int i = 0; i < num_workers; ++i) {
    thread_pool->Schedule(grab_tasks, parallel_for::WorkerIndex(i + 1));
  }
  // Also run tasks in the master thread.
  grab_tasks(parallel_for::WorkerIndex(0));

  // Wait until all tasks have finished.
  shared_state->block_until_finished.Block();
}

template <typename InputIter, typename F>
void MapParallelFor(InputIter begin_it, InputIter end_it, F f,
                    ThreadPool* thread_pool = ThreadPool::MapDefaultPool()) {
  if (begin_it == end_it) return;
  TIMELINE("MapParallelForIter");
  int end = std::distance(begin_it, end_it);
  if (end == 1) {
    f(*begin_it);
    return;
  }

  auto func = [f = std::move(f)](int worker_index, InputIter it) { f(*it); };
  const int begin = 0;
  const int num_iters = end - begin;
  int block_size =
      std::max<int>(1, num_iters / ((thread_pool->NumWorkers() + 1) << 2));

  auto shared_state = std::make_shared<SharedState>(num_iters);

  // Dynamically assign tasks to all workers for better load balance.
  const auto grab_tasks = [=, &func](parallel_for::WorkerIndex worker_index) {
    // ("ParallelFor::WorkerThread");
    int iters_done = 0;
    while (true) {
      const int index = shared_state->next_index.fetch_add(
                            block_size, std::memory_order_acq_rel) +
                        begin;
      if (index >= end) break;
      for (int i = index; i < std::min(end, index + block_size);
           ++i, ++iters_done) {
        func(worker_index, std::next(begin_it, i));
      }
    }
    shared_state->block_until_finished.Finished(iters_done);
  };

  const int min_num_workers = (num_iters + block_size - 1) / block_size;
  const int num_workers =
      std::min(min_num_workers - 1, thread_pool->NumWorkers());
  // Schedule futures.
  for (int i = 0; i < num_workers; ++i) {
    thread_pool->Schedule(grab_tasks, parallel_for::WorkerIndex(i + 1));
  }
  // Also run tasks in the master thread.
  grab_tasks(parallel_for::WorkerIndex(0));

  // Wait until all tasks have finished.
  shared_state->block_until_finished.Block();
}

}  // namespace st

#endif  // ONBOARD_ASYNC_PARALLEL_FOR_H_
