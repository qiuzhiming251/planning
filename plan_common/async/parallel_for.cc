

#include "plan_common/async/parallel_for.h"

// IWYU pragma: no_include <cxxabi.h>  // for __forced_unwind
#include <algorithm>
#include <atomic>
#include <memory>
#include <type_traits>
#include <utility>
#include <random>
#include <chrono>

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
#include "absl/strings/str_cat.h"
//#include "global/trace.h"

namespace st {

void ParallelFor(int begin, int end, ThreadPool* thread_pool,
                 const parallel_for::Options& options,
                 std::function<void(parallel_for::WorkerIndex, int)>&& func) {
  if (end == begin) return;

  // Run this for loop sequentially if thread_pool is null.
  if (thread_pool == nullptr) {
    // _ARG1("ParallelFor::WorkerThread_serial", "num", end -
    // begin);
    TIMELINE(absl::StrFormat("SingleParallelFor_%d", (end - begin)));
    for (int i = begin; i < end; ++i) {
      func(parallel_for::WorkerIndex(0), i);
    }
    return;
  }

  // Run this for loop sequentially if thread_pool is null.
  if (thread_pool == nullptr) {
    for (int i = begin; i < end; ++i) {
      func(parallel_for::WorkerIndex(0), i);
    }
    return;
  }

  const int num_iters = end - begin;
  constexpr int kBlockSizeDivider = 4;
  int block_size = options.block_size;
  if (block_size == 0) {
    block_size = std::max<int>(
        1, num_iters / ((thread_pool->NumWorkers() + 1) * kBlockSizeDivider));
  }

  auto shared_state = std::make_shared<SharedState>(num_iters);
  const auto grab_tasks =
      [=, func = std::move(func)](parallel_for::WorkerIndex worker_index) {
        int iters_done = 0;
        while (true) {
          const int index = shared_state->next_index.fetch_add(
                                block_size, std::memory_order_acq_rel) +
                            begin;
          if (index >= end) break;
          for (int i = index; i < std::min(end, index + block_size);
               ++i, ++iters_done) {
            func(worker_index, i);
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

void ParallelFor(int begin, int end, ThreadPool* thread_pool,
                 std::function<void(parallel_for::WorkerIndex, int)>&& func) {
  ParallelFor(
      begin, end, thread_pool, {},
      std::forward<std::function<void(parallel_for::WorkerIndex, int)>>(func));
}

void ParallelFor(int begin, int end,
                 std::function<void(parallel_for::WorkerIndex, int)>&& func) {
  ParallelFor(
      begin, end, ThreadPool::DefaultPool(),
      std::forward<std::function<void(parallel_for::WorkerIndex, int)>>(func));
}

void ParallelFor(int begin, int end, ThreadPool* thread_pool,
                 const parallel_for::Options& options,
                 std::function<void(int)>&& func) {
  ParallelFor(begin, end, thread_pool, options,
              [func = std::move(func)](int worker_index, int i) { func(i); });
}

void ParallelFor(int begin, int end, ThreadPool* thread_pool,
                 std::function<void(int)>&& func) {
  ParallelFor(begin, end, thread_pool, {},
              [func = std::move(func)](int worker_index, int i) { func(i); });
}

void ParallelFor(int begin, int end, std::function<void(int)>&& func) {
  ParallelFor(begin, end, ThreadPool::DefaultPool(),
              [func = std::move(func)](int worker_index, int i) { func(i); });
}

}  // namespace st
