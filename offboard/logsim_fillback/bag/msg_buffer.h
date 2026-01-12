#pragma once
#include <algorithm>
#include <map>
#include <mutex>
#include <vector>

#include "common/noncopyable.h"
#include "msg/dynamic_data.h"

namespace worldview {
class MsgBuffer {
 public:
  using Ptr = std::shared_ptr<MsgBuffer>;
  using iterator = std::multimap<int64_t, MsgInstance::Ptr>::iterator;

  MsgBuffer() = default;

  void push(const MsgInstance::Ptr& msg) {
    {
      std::lock_guard<std::mutex> mid_lock(mid_buffer_mutex_);
      mid_buffer_.emplace(msg->time(), msg);
    }
    topic_buffers_[msg->topic()].emplace(msg->time(), msg);
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return buffer_.empty();
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return buffer_.size();
  }

  void merge() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    std::lock_guard<std::mutex> mid_lock(mid_buffer_mutex_);
    buffer_.insert(mid_buffer_.begin(), mid_buffer_.end());
    mid_buffer_.clear();
  }

  iterator begin() { return buffer_.begin(); }
  iterator end() { return buffer_.end(); }

  iterator floor(int64_t time) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    iterator dest = buffer_.lower_bound(time);
    if (dest == buffer_.end()) {
      return dest;
    }
    if (dest != buffer_.begin() && dest->second->time() != time) {
      --dest;
    }
    return dest;
  }

  bool hasTopic(const std::string& topic) {
    auto it = topic_buffers_.find(topic);
    return it != topic_buffers_.end();
  }

  size_t size(const std::string& topic) {
    auto it = topic_buffers_.find(topic);
    return it == topic_buffers_.end() ? 0 : it->second.size();
  }

  iterator begin(const std::string& topic) {
    auto it = topic_buffers_.find(topic);
    if (it == topic_buffers_.end()) {
      throw std::runtime_error("the buffer does not have topic " + topic);
    }
    return it->second.begin();
  }

  iterator end(const std::string& topic) {
    auto it = topic_buffers_.find(topic);
    if (it == topic_buffers_.end()) {
      throw std::runtime_error("the buffer does not have topic " + topic);
    }
    return it->second.end();
  }

  iterator floor(const std::string& topic, int64_t time) {
    auto it = topic_buffers_.find(topic);
    if (it == topic_buffers_.end()) {
      throw std::runtime_error("the buffer does not have topic " + topic);
    }
    iterator dest = it->second.lower_bound(time);
    if (dest == it->second.end()) {
      return dest;
    }
    if (dest != it->second.begin() && dest->second->time() != time) {
      --dest;
    }
    return dest;
  }

 private:
  mutable std::mutex buffer_mutex_;
  mutable std::mutex mid_buffer_mutex_;
  std::multimap<int64_t, MsgInstance::Ptr> buffer_;
  std::multimap<int64_t, MsgInstance::Ptr> mid_buffer_;

  std::map<std::string, std::multimap<int64_t, MsgInstance::Ptr>>
      topic_buffers_;
};
}  // namespace worldview
