#include "msg/cyber/type_manager_cyber.h"
#include "common/lib_util.h"
#include "common/log.h"

namespace worldview {
TypeManagerCyber::TypeManagerCyber() { update(); }

bool TypeManagerCyber::add(const DynamicTypeCyberPtr& type) {
  return supported_types_.try_emplace(type->full_name(), type).second;
}

std::map<std::string, DynamicTypeCyberPtr> TypeManagerCyber::enabledTypes()
    const {
  std::map<std::string, DynamicTypeCyberPtr> res;
  for (auto& type : supportedTypes()) {
    if (disabled_types_.count(type.first)) {
      continue;
    }
    res.emplace(type.first, get(type.first));
  }
  return res;
}

const DynamicTypeCyberPtr& TypeManagerCyber::get(
    const std::string& name) const {
  auto s_it = supported_types_.find(name);
  if (s_it != supported_types_.end()) {
    return s_it->second;
  }
  throw std::runtime_error(name + " type is unrecognized");
}

bool TypeManagerCyber::supported(const std::string& name) const {
  return supported_types_.find(name) != supported_types_.end();
}

bool TypeManagerCyber::enabled(const std::string& type) const {
  return supported(type) && disabled_types_.count(type) == 0;
}

void TypeManagerCyber::disableTypes(const std::set<std::string>& types) {
  disabled_types_ = types;
}

Filter TypeManagerCyber::cleanTypeFilter(const Filter& filter) const {
  Filter res;
  res.black = filter.black;
  if (filter.white.size()) {
    for (auto& type : filter.white) {
      if (enabled(type)) {
        res.white.insert(type);
      }
    }
  } else {
    for (auto& type : enabledTypes()) {
      res.white.insert(type.first);
    }
  }
  return res;
}

void TypeManagerCyber::update() {
  supported_types_.clear();
  // util::LibWrapper supported_types_lib("libsupported_cyber_types.so");
  // auto getSupportedTypes = supported_types_lib.GetFunc<std::map<
  //     std::string, std::shared_ptr<const google::protobuf::Descriptor>>
  //     (*)()>( "getSupportedCyberTypes");
  // if (!getSupportedTypes) {
  //   throw std::runtime_error("get supportted types failed");
  // }
  for (auto& type : getSupportedCyberTypes()) {
    add(type.second);
  }
}
}  // namespace worldview