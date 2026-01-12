

#ifndef ONBOARD_ASYNC_ASYNC_MACRO_H_
#define ONBOARD_ASYNC_ASYNC_MACRO_H_

#include <memory>
#include <string>
#include <system_error>
#include <utility>

#include "absl/strings/str_format.h"
#include "plan_common/async/async_util.h"
//#include "global/trace.h"

#define MOVE_DESTROY_CONTAINER_ASYNC(thread_pool, container)                 \
  ScheduleFuture(thread_pool, [_ = std::move(container)]() mutable {         \
    const std::string container_name =                                       \
        absl::StrFormat("%s_%d", #container, __LINE__);                      \
    const std::string file(__FILE__);                                        \
    _ARG2("DestroyContainerAsync", "container name", container_name, "file", \
          file);                                                             \
    [[maybe_unused]] const auto unused = std::move(_);                       \
  })

#define MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(container) \
  MOVE_DESTROY_CONTAINER_ASYNC(ThreadPool::DisposalPool(), container)

#endif  // ONBOARD_ASYNC_ASYNC_MACRO_H_
