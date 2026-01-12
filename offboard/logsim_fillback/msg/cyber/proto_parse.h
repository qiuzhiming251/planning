#pragma once
#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>
#include <json/json.h>

#include <map>
#include <string>

#include "common/log.h"
#include "msg/typedefs.h"
namespace worldview {

class CyberStruct {
 public:
  using Ptr = std::shared_ptr<CyberStruct>;
  CyberStruct(const DynamicTypeCyberPtr& cyber_type)
      : cyber_type_(cyber_type) {}

  virtual ~CyberStruct() = default;
  Json::Value getLikeIDLStruct() const;
  static DynamicGoogleMessagePtr JsonToMessage(
      const std::string& jsonData, const DynamicTypeCyberPtr& descriptor);
  static std::string ProtoJsonWithDefaults(
      const DynamicGoogleMessagePtr& message_ptr);
  static Json::Value GetRawStringJsonCyber(const DynamicDataCyberPtr& raw_msg,
                                           const std::string& type_cyber);
  static DynamicDataCyberPtr GetRawMsgFromJson(const Json::Value& json_value,
                                               const std::string& type_cyber);

 private:
  const DynamicTypeCyberPtr& cyber_type_;
};

}  // namespace worldview