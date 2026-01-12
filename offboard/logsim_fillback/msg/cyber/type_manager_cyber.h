#pragma once
#include <map>
#include <set>

#include "common/noncopyable.h"
#include "msg/cyber/supported_types_cyber.h"
#include "msg/filter.h"

namespace worldview {

using DynamicTypeCyberPtr = const google::protobuf::Descriptor*;

class TypeManagerCyber : public noncopyable {
 public:
  // singleton
  static TypeManagerCyber& getInstance() {
    static TypeManagerCyber manager;
    return manager;
  }

  std::map<std::string, DynamicTypeCyberPtr> supportedTypes() const {
    return supported_types_;
  };
  std::map<std::string, DynamicTypeCyberPtr> enabledTypes() const;

  const DynamicTypeCyberPtr& get(const std::string& name) const;
  bool add(const DynamicTypeCyberPtr& type);

  bool supported(const std::string&) const;
  bool enabled(const std::string&) const;

  void disableTypes(const std::set<std::string>&);
  void update();

  Filter cleanTypeFilter(const Filter& filter = Filter()) const;

 private:
  TypeManagerCyber();

  std::map<std::string, DynamicTypeCyberPtr> supported_types_;
  std::set<std::string> disabled_types_;
};
}  // namespace worldview