#pragma once
#include <string>

#include "msg/typedefs.h"

namespace worldview {

struct PubSubSetting {
  PubSubSetting() = default;
  PubSubSetting(const NodeCyberPtr node_cyber_, int32_t domain_id_,
                const std::string& name_, const std::string& type_)
      : node_cyber(node_cyber_),
        domain_id(domain_id_),
        name(name_),
        type(type_) {}
  NodeCyberPtr node_cyber = nullptr;
  int32_t domain_id = 0;
  std::string name = "";
  std::string type = "";
};

using PubSetting = PubSubSetting;
using SubSetting = PubSubSetting;
}  // namespace worldview