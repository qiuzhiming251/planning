

#ifndef ONBOARD_ASYNC_FUTURE_H_
#define ONBOARD_ASYNC_FUTURE_H_

#include <chrono>
#include <cstdint>
#include <future>
#include <utility>

namespace st {

enum class FutureStatus : uint8_t {
  kInvalid = 0,
  kDeferred,
  kReady,
  kTimeout,
};

template <typename T>
class Future {
 public:
  Future() = default;
  explicit Future(std::future<T> future) : future_(std::move(future)) {}

  bool IsValid() const { return future_.valid(); }

  // Wait for future to be ready.
  void Wait() const {
    if (future_.valid()) {
      future_.wait();
    }
  }

  FutureStatus WaitFor(double seconds) const {
    if (!future_.valid()) return FutureStatus::kInvalid;
    const auto status =
        future_.wait_for(std::chrono::duration<double>(seconds));
    switch (status) {
      case std::future_status::deferred:
        return FutureStatus::kDeferred;
      case std::future_status::ready:
        return FutureStatus::kReady;
      case std::future_status::timeout:
        return FutureStatus::kTimeout;
    }
  }

  bool IsReady() const {
    return future_.valid() &&
           future_.wait_for(std::chrono::duration<int>::zero()) ==
               std::future_status::ready;
  }

  // Wait for future to be ready and get the returned value.
  T Get() const { return future_.get(); }

 private:
  std::shared_future<T> future_;
};

}  // namespace st

#endif  // ONBOARD_ASYNC_FUTURE_H_
