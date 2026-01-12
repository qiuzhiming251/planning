#pragma once

namespace worldview {

class noncopyable {
 public:
  noncopyable() = default;
  ~noncopyable() = default;

  noncopyable(noncopyable &&) = default;
  noncopyable &operator=(noncopyable &&) = default;

  noncopyable(const noncopyable &) = delete;
  noncopyable &operator=(const noncopyable &) = delete;
};

}  // namespace worldview