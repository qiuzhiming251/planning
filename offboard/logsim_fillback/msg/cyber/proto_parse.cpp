#include "msg/cyber/proto_parse.h"

#include <google/protobuf/util/json_util.h>

#include <limits>
#include <string>

namespace worldview {
namespace {

std::string GetTypeName(const google::protobuf::FieldDescriptor* field) {
  switch (field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
      return "struct";
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
      return "enum";
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
      return "primitive";
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
      return "others";
    default:
      return "unknown";
  }
}

std::string GetFieldTypeName(const google::protobuf::FieldDescriptor* field) {
  if (field->is_repeated()) {
    return "sequence";
  } else {
    return GetTypeName(field);
  }
}

void MessageToJson(const google::protobuf::Descriptor* descriptor,
                   Json::Value& json_value) {
  for (int i = 0; i < descriptor->field_count(); ++i) {
    const google::protobuf::FieldDescriptor* field = descriptor->field(i);
    if (field->is_repeated()) {
      Json::Value sub_json = Json::arrayValue;
      Json::Value child_sub_sequence = Json::objectValue;
      Json::Value temp_json = Json::objectValue;
      if (field->message_type()) {
        MessageToJson(field->message_type(), sub_json);
        child_sub_sequence["children"] = sub_json;
        child_sub_sequence["type"] = GetTypeName(field);
        child_sub_sequence["name"] = field->name();
        temp_json["element"] = child_sub_sequence;
      } else {
        Json::Value child_json = Json::objectValue;
        child_json["type"] = GetTypeName(field);
        child_json["name"] = field->name();
        temp_json["element"] = child_json;
      }
      temp_json["type"] = GetFieldTypeName(field);
      temp_json["name"] = field->name();
      if (!json_value.isArray()) {
        json_value = Json::arrayValue;
      }
      json_value.append(temp_json);
    } else {
      if (field->cpp_type() ==
          google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE) {
        Json::Value sub_json = Json::arrayValue;
        if (field->message_type()) {
          MessageToJson(field->message_type(), sub_json);
        } else {
          Json::Value child_json = Json::objectValue;
          child_json["type"] = GetFieldTypeName(field);
          child_json["name"] = field->name();
          sub_json.append(child_json);
        }
        Json::Value temp_json = Json::objectValue;
        temp_json["children"] = sub_json;
        temp_json["type"] = GetFieldTypeName(field);
        temp_json["name"] = field->name();
        if (!json_value.isArray()) {
          json_value = Json::arrayValue;
        }
        json_value.append(temp_json);
      } else {
        Json::Value sub_json = Json::objectValue;
        sub_json["type"] = GetFieldTypeName(field);
        sub_json["name"] = field->name();
        if (!json_value.isArray()) {
          json_value = Json::arrayValue;
        }
        json_value.append(sub_json);
      }
    }
  }
}

void SetFieldDefaultValue(DynamicGoogleMessage* message,
                          const google::protobuf::FieldDescriptor* field,
                          int index = -1) {
  const auto* reflection = message->GetReflection();
  if (index < 0) {
    if (!reflection->HasField(*message, field)) {
      switch (field->cpp_type()) {
        case google::protobuf::FieldDescriptor::CPPTYPE_INT32: {
          int32_t default_val = field->default_value_int32();
          reflection->SetInt32(message, field, default_val);
          break;
        }
        case google::protobuf::FieldDescriptor::CPPTYPE_INT64: {
          int64_t default_val = field->default_value_int64();
          reflection->SetInt64(message, field, default_val);
          break;
        }
        case google::protobuf::FieldDescriptor::CPPTYPE_UINT32: {
          uint32_t default_val = field->default_value_uint32();
          reflection->SetUInt32(message, field, default_val);
          break;
        }
        case google::protobuf::FieldDescriptor::CPPTYPE_UINT64: {
          uint64_t default_val = field->default_value_uint64();
          reflection->SetUInt64(message, field, default_val);
          break;
        }
        case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE: {
          double default_val = field->default_value_double();
          if (!std::isfinite(default_val)) {
            default_val = 0.0;
          }
          reflection->SetDouble(message, field, default_val);
          break;
        }
        case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT: {
          float default_val = field->default_value_float();
          if (!std::isfinite(default_val)) {
            default_val = 0.0f;
          }
          reflection->SetFloat(message, field, default_val);
          break;
        }
        case google::protobuf::FieldDescriptor::CPPTYPE_BOOL: {
          bool default_val = field->default_value_bool();
          reflection->SetBool(message, field, default_val);
          break;
        }
        case google::protobuf::FieldDescriptor::CPPTYPE_STRING: {
          std::string default_val = field->default_value_string();
          reflection->SetString(message, field, default_val);
          break;
        }
        case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
          const auto* default_val = field->default_value_enum();
          reflection->SetEnum(message, field, default_val);
          break;
        }
        default:
          break;
      }
    }
  } else {
    bool shouldSetDefault = false;
    switch (field->cpp_type()) {
      case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
        shouldSetDefault =
            reflection->GetRepeatedInt32(*message, field, index) ==
            field->default_value_int32();
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
        shouldSetDefault =
            reflection->GetRepeatedInt64(*message, field, index) ==
            field->default_value_int64();
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
        shouldSetDefault =
            reflection->GetRepeatedUInt32(*message, field, index) ==
            field->default_value_uint32();
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
        shouldSetDefault =
            reflection->GetRepeatedUInt64(*message, field, index) ==
            field->default_value_uint64();
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE: {
        double val = reflection->GetRepeatedDouble(*message, field, index);
        double default_val = field->default_value_double();
        if (!std::isfinite(default_val)) {
          default_val = 0.0;
        }
        shouldSetDefault = std::abs(val - default_val) <
                           std::numeric_limits<double>::epsilon();
        break;
      }
      case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT: {
        float val = reflection->GetRepeatedFloat(*message, field, index);
        float default_val = field->default_value_float();
        if (!std::isfinite(default_val)) {
          default_val = 0.0f;
        }
        shouldSetDefault =
            std::abs(val - default_val) < std::numeric_limits<float>::epsilon();
        break;
      }
      case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
        shouldSetDefault =
            reflection->GetRepeatedBool(*message, field, index) ==
            field->default_value_bool();
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
        shouldSetDefault =
            reflection->GetRepeatedString(*message, field, index) ==
            field->default_value_string();
        break;
      case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
        shouldSetDefault =
            reflection->GetRepeatedEnum(*message, field, index) ==
            field->default_value_enum();
        break;
      default:
        break;
    }
    if (shouldSetDefault) {
      switch (field->cpp_type()) {
        case google::protobuf::FieldDescriptor::CPPTYPE_INT32: {
          int32_t default_val = field->default_value_int32();
          reflection->SetRepeatedInt32(message, field, index, default_val);
          break;
        }
        case google::protobuf::FieldDescriptor::CPPTYPE_INT64: {
          int64_t default_val = field->default_value_int64();
          reflection->SetRepeatedInt64(message, field, index, default_val);
          break;
        }
        case google::protobuf::FieldDescriptor::CPPTYPE_UINT32: {
          uint32_t default_val = field->default_value_uint32();
          reflection->SetRepeatedUInt32(message, field, index, default_val);
          break;
        }
        case google::protobuf::FieldDescriptor::CPPTYPE_UINT64: {
          uint64_t default_val = field->default_value_uint64();
          reflection->SetRepeatedUInt64(message, field, index, default_val);
          break;
        }
        case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE: {
          double default_val = field->default_value_double();
          if (!std::isfinite(default_val)) {
            default_val = 0.0;
          }
          reflection->SetRepeatedDouble(message, field, index, default_val);
          break;
        }
        case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT: {
          float default_val = field->default_value_float();
          if (!std::isfinite(default_val)) {
            default_val = 0.0f;
          }
          reflection->SetRepeatedFloat(message, field, index, default_val);
          break;
        }
        case google::protobuf::FieldDescriptor::CPPTYPE_BOOL: {
          bool default_val = field->default_value_bool();
          reflection->SetRepeatedBool(message, field, index, default_val);
          break;
        }
        case google::protobuf::FieldDescriptor::CPPTYPE_STRING: {
          std::string default_val = field->default_value_string();
          reflection->SetRepeatedString(message, field, index, default_val);
          break;
        }
        case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
          const auto* default_val = field->default_value_enum();
          reflection->SetRepeatedEnum(message, field, index, default_val);
          break;
        }
        default:
          break;
      }
    }
  }
}

void FillMissingFields(DynamicGoogleMessage* message) {
  const auto* descriptor = message->GetDescriptor();
  const auto* reflection = message->GetReflection();
  for (int i = 0; i < descriptor->field_count(); ++i) {
    const auto* field = descriptor->field(i);
    if (field->is_repeated()) {
      int size = reflection->FieldSize(*message, field);
      if (size > 0) {
        if (field->cpp_type() ==
                google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE &&
            field->message_type()) {
          for (int j = 0; j < size; ++j) {
            DynamicGoogleMessage* subMessage =
                reflection->MutableRepeatedMessage(message, field, j);
            FillMissingFields(subMessage);
          }
        } else {
          for (int j = 0; j < size; ++j) {
            SetFieldDefaultValue(message, field, j);
          }
        }
      }
      continue;
    } else if (field->cpp_type() ==
                   google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE &&
               field->message_type()) {
      FillMissingFields(reflection->MutableMessage(message, field));
    } else {
      SetFieldDefaultValue(message, field);
    }
  }
}
}  // namespace

std::string CyberStruct::ProtoJsonWithDefaults(
    const DynamicGoogleMessagePtr& message_ptr) {
  std::string output_json;
  if (!message_ptr) {
    return output_json;
  }
  FillMissingFields(message_ptr.get());
  google::protobuf::util::JsonOptions json_options;
  json_options.add_whitespace = true;
  json_options.always_print_primitive_fields = true;
  json_options.preserve_proto_field_names = true;
  json_options.always_print_enums_as_ints = true;
  google::protobuf::util::MessageToJsonString(*message_ptr, &output_json,
                                              json_options);
  return output_json;
}

DynamicGoogleMessagePtr CyberStruct::JsonToMessage(
    const std::string& jsonData, const DynamicTypeCyberPtr& descriptor) {
  google::protobuf::MessageFactory* factory =
      google::protobuf::MessageFactory::generated_factory();
  DynamicGoogleMessagePtr message_ptr(
      factory->GetPrototype(descriptor.get())->New());
  google::protobuf::util::JsonParseOptions options;
  options.ignore_unknown_fields = true;
  auto status = google::protobuf::util::JsonStringToMessage(
      jsonData, message_ptr.get(), options);
  if (!status.ok()) {
    return nullptr;
  }
  return message_ptr;
}

Json::Value CyberStruct::getLikeIDLStruct() const {
  Json::Value children = Json::objectValue;
  MessageToJson(cyber_type_.get(), children);
  Json::Value res = Json::objectValue;
  res["children"] = children;
  res["type"] = GetFieldTypeName(cyber_type_->field(0));
  res["name"] = cyber_type_->full_name();
  return res;
}

DynamicDataCyberPtr CyberStruct::GetRawMsgFromJson(
    const Json::Value& json_value, const std::string& type_cyber) {
  Json::StreamWriterBuilder writer;
  std::string json_str = Json::writeString(writer, json_value);

  auto rawFactory = apollo::cyber::message::ProtobufFactory::Instance();
  auto raw_msg_class = rawFactory->GenerateMessageByType(type_cyber);
  if (!raw_msg_class) {
    return nullptr;
  }

  std::unique_ptr<google::protobuf::Message> up_msg(raw_msg_class);
  google::protobuf::util::JsonParseOptions json_options;
  json_options.ignore_unknown_fields = true;
  auto status = google::protobuf::util::JsonStringToMessage(
      json_str, raw_msg_class, json_options);
  if (!status.ok()) {
    LOG_ERROR << "Failed to parse JSON to Protobuf: " << status.message();
    return nullptr;
  }

  std::string serialize_str;
  up_msg->SerializeToString(&serialize_str);
  return std::make_shared<DynamicDataCyber>(serialize_str);
}

Json::Value CyberStruct::GetRawStringJsonCyber(
    const DynamicDataCyberPtr& raw_msg, const std::string& type_cyber) {
  Json::Value json_value;
  if (type_cyber.empty()) {
    return json_value;
  }
  auto rawFactory = apollo::cyber::message::ProtobufFactory::Instance();
  auto raw_msg_class = rawFactory->GenerateMessageByType(type_cyber);
  if (!raw_msg_class) {
    return json_value;
  }

  std::unique_ptr<google::protobuf::Message> up_msg(raw_msg_class);
  std::string serialize_str;
  raw_msg->SerializeToString(&serialize_str);
  up_msg->ParseFromString(serialize_str);
  std::string json_str;

  google::protobuf::util::JsonOptions json_options;
  json_options.add_whitespace = true;
  json_options.always_print_primitive_fields = true;
  json_options.preserve_proto_field_names = true;
  json_options.always_print_enums_as_ints = true;
  google::protobuf::util::MessageToJsonString(*raw_msg_class, &json_str,
                                              json_options);
  try {
    Json::CharReaderBuilder reader;
    std::string errs;
    std::istringstream s(json_str);
    if (!Json::parseFromStream(reader, s, &json_value, &errs)) {
      throw std::runtime_error("Failed to parse JSON: " + errs);
    }
  } catch (std::exception& e) {
    LOG_ERROR << "Failed to parse JSON: " << e.what();
  }
  return json_value;
}
}  // namespace worldview