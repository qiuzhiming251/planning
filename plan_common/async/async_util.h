

#ifndef ONBOARD_ASYNC_ASYNC_UTIL_H_
#define ONBOARD_ASYNC_ASYNC_UTIL_H_

#include <future>
#include <string>
#include <type_traits>
#include <utility>

#include "plan_common/async/future.h"
#include "plan_common/async/thread_pool.h"
//#include "global/trace.h"

namespace st {

// Schedule a task to the given thread pool. If thread_pool is null, run the
// task synchronously on the current thread.
template <typename Func, typename... Args>
auto ScheduleFuture(ThreadPool* thread_pool, Func&& f, Args&&... args) {
  if (thread_pool == nullptr) {
    Future<typename std::result_of<Func(Args...)>::type> future(
        std::async(std::launch::deferred, std::forward<Func>(f),
                   std::forward<Args>(args)...));
    future.Wait();
    return future;
  }
  return thread_pool->Schedule(std::forward<Func>(f),
                               std::forward<Args>(args)...);
}

// Schedule a task to the default thread pool.
template <typename Func, typename... Args>
auto ScheduleFuture(Func&& f, Args&&... args) {
  return ScheduleFuture(ThreadPool::DefaultPool(), std::forward<Func>(f),
                        std::forward<Args>(args)...);
}

// Asynchronously destroy a container.
template <typename ContainerT>
void DestroyContainerAsync(ThreadPool* thread_pool, ContainerT container) {
  ScheduleFuture(thread_pool, [_ = std::move(container)]() mutable {
    [[maybe_unused]] const auto unused = std::move(_);
  });
}

// Same, but use the default thread pool.
template <typename ContainerT>
void DestroyContainerAsync(ContainerT container) {
  DestroyContainerAsync(ThreadPool::DisposalPool(), std::move(container));
}

template <typename ContainerT>
void DestroyContainerAsyncMarkSource(ThreadPool* thread_pool,
                                     ContainerT container, std::string source) {
  ScheduleFuture(thread_pool, [_ = std::move(container),
                               str_source = std::move(source)]() mutable {
    [[maybe_unused]] const auto unused = std::move(_);
  });
}

// Same, but use the default thread pool.
template <typename ContainerT>
void DestroyContainerAsyncMarkSource(ContainerT container, std::string source) {
  DestroyContainerAsyncMarkSource(ThreadPool::DisposalPool(),
                                  std::move(container), std::move(source));
}

// Asynchronously destroy a pointer.
template <typename PointerT,
          std::enable_if_t<std::is_pointer_v<PointerT>, bool> = true>
void DestroyPointerAsync(ThreadPool* thread_pool, PointerT ptr) {
  ScheduleFuture(thread_pool, [ptr = std::move(ptr)]() mutable {
    if (ptr != nullptr) delete ptr;
  });
}

// Same, but use the default thread pool.
template <typename PointerT,
          std::enable_if_t<std::is_pointer_v<PointerT>, bool> = true>
void DestroyPointerAsync(PointerT ptr) {
  DestroyPointerAsync(ThreadPool::DisposalPool(), std::move(ptr));
}

template <typename T>
struct AsyncDeleter {
  void operator()(T* p) const { DestroyPointerAsync(p); }
};

template <typename T>
void WaitForFuture(const Future<T>& future) {
  // ("WaitForFuture");
  future.Wait();
}

template <typename T>
class AsyncDestroyedResourceHolder {
 public:
  AsyncDestroyedResourceHolder() = default;
  ~AsyncDestroyedResourceHolder() { DestroyContainerAsync(std::move(val_)); }
  explicit AsyncDestroyedResourceHolder(T&& val) : val_(val) {}
  void TakeOwnership(T val) {
    DestroyContainerAsync(std::move(val_));
    val_ = std::move(val);
  }
  T& operator*() { return val_; }
  const T& operator*() const { return val_; }

 private:
  T val_;
};
}  // namespace st

#endif  // ONBOARD_ASYNC_ASYNC_UTIL_H_
