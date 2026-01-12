#pragma once

#pragma once
#include <functional>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "common/json_util.h"
#include "common/string_util.h"
#include "msg/typedefs.h"
#include "msg/cyber/type_manager_cyber.h"

namespace worldview {
class MsgInstance {
 public:
  using Ptr = std::shared_ptr<MsgInstance>;
  using PtrComp = std::function<bool(const Ptr &, const Ptr &)>;

  MsgInstance(int64_t time, const std::string &topic_name,
              const DynamicDataCyberPtr &data)
      : time_(time), topic_name_(topic_name), data_cyber_(data) {}

  bool operator<(const MsgInstance &rhs) const {
    return this->time_ < rhs.time_;
  }
  bool operator>(const MsgInstance &rhs) const {
    return this->time_ > rhs.time_;
  }
  int64_t time() const { return time_; }
  const std::string &topic() const { return topic_name_; }
  const DynamicDataCyberPtr &data_cyber() const { return data_cyber_; }
  DynamicDataCyberPtr &data_cyber() { return data_cyber_; }

  static PtrComp PtrGreater;
  static PtrComp PtrLess;

 private:
  int64_t time_;
  std::string topic_name_;
  DynamicDataCyberPtr data_cyber_;
};
}  // namespace worldview