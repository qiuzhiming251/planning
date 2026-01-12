#pragma once
#include <glog/logging.h>
#include <google/protobuf/descriptor.h>
#include <map>
#include <memory>
#include <string>

extern std::map<std::string,
                std::shared_ptr<const google::protobuf::Descriptor>>
    kSupportedCyberTypes;

// extern "C" {
const std::map<std::string, const google::protobuf::Descriptor*>&
getSupportedCyberTypes();
// }