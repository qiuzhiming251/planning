//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// NOTE: This file is copied from MediaPipe project and modified by BYD.ai
// for its own use.

#ifndef ONBOARD_UTILS_STATUS_BUILDER_H_
#define ONBOARD_UTILS_STATUS_BUILDER_H_

#include <memory>
#include <sstream>
#include <utility>

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"

namespace st {

class ABSL_MUST_USE_RESULT StatusBuilder {
 public:
  StatusBuilder(const StatusBuilder& sb);
  StatusBuilder& operator=(const StatusBuilder& sb);
  // Creates a `StatusBuilder` based on an original status.
  StatusBuilder(const absl::Status& original_status)
      : status_(original_status), stream_(new std::ostringstream) {}

  StatusBuilder(absl::Status&& original_status)
      : status_(std::move(original_status)), stream_(new std::ostringstream) {}

  // Creates a `StatusBuilder` from a absl status code.
  StatusBuilder(absl::StatusCode code)
      : status_(code, ""), stream_(new std::ostringstream) {}

  bool ok() const { return status_.ok(); }

  template <typename T>
  T Returns(T&& val) const {
    return std::forward<T>(val);
  }

  void Void() const {}

  StatusBuilder& SetAppend();

  StatusBuilder& SetPrepend();

  StatusBuilder& SetNoLogging();

  StatusBuilder& operator<<(const StatusBuilder& other) { return *this; }

  template <typename T>
  StatusBuilder& operator<<(const T& msg) {
    if (status_.ok()) return *this;
    *stream_ << msg;
    return *this;
  }

  operator absl::Status() const&;
  operator absl::Status() &&;

  absl::Status JoinMessageToStatus() const;

 private:
  // Specifies how to join the error message in the original status and any
  // additional message that has been streamed into the builder.
  enum class MessageJoinStyle {
    kAnnotate,
    kAppend,
    kPrepend,
  };

  // The status that the result will be based on.
  absl::Status status_;
  bool no_logging_ = false;
  // The additional messages added with `<<`.
  std::unique_ptr<std::ostringstream> stream_;
  // Specifies how to join the message in `status_` and `stream_`.
  MessageJoinStyle join_style_ = MessageJoinStyle::kAnnotate;
};

inline StatusBuilder AlreadyExistsErrorBuilder() {
  return StatusBuilder(absl::StatusCode::kAlreadyExists);
}

inline StatusBuilder FailedPreconditionErrorBuilder() {
  return StatusBuilder(absl::StatusCode::kFailedPrecondition);
}

inline StatusBuilder InternalErrorBuilder() {
  return StatusBuilder(absl::StatusCode::kInternal);
}

inline StatusBuilder InvalidArgumentErrorBuilder() {
  return StatusBuilder(absl::StatusCode::kInvalidArgument);
}

inline StatusBuilder NotFoundErrorBuilder() {
  return StatusBuilder(absl::StatusCode::kNotFound);
}

inline StatusBuilder UnavailableErrorBuilder() {
  return StatusBuilder(absl::StatusCode::kUnavailable);
}

inline StatusBuilder UnimplementedErrorBuilder() {
  return StatusBuilder(absl::StatusCode::kUnimplemented);
}

inline StatusBuilder UnknownErrorBuilder() {
  return StatusBuilder(absl::StatusCode::kUnknown);
}

}  // namespace st

#endif  // ONBOARD_UTILS_STATUS_BUILDER_H_
