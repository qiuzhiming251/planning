#include "bag/record.h"

#include <google/protobuf/util/json_util.h>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "cyber/message/protobuf_factory.h"
#include "common/command_util.h"
#include "common/log.h"
#include "common/sqlite.h"
#include "msg/cyber/type_manager_cyber.h"

namespace worldview {

Json::Value RecordInfo::toJson() const {
  Json::Value root;
  for (auto &f : filenames) {
    root["filenames"].append(f);
  }
  for (auto &e : extra) {
    root["extra"][e.first] = e.second;
  }
  root["duration"] = static_cast<Json::Int64>(duration);
  root["min"] = static_cast<Json::Int64>(min);
  root["max"] = static_cast<Json::Int64>(max);
  root["total_num"] = total_num;
  for (auto &[topic, type] : topics) {
    Json::Value topic_info;
    topic_info["type"] = type;
    topic_info["num"] = topics_num.count(topic) ? topics_num.at(topic) : 0;
    topic_info["freq"] = topics_freq.count(topic) ? topics_freq.at(topic) : 0;
    topic_info["empty_front"] = static_cast<Json::Int64>(
        topics_empty_front.count(topic) ? topics_empty_front.at(topic) : 0);
    topic_info["empty_back"] = static_cast<Json::Int64>(
        topics_empty_back.count(topic) ? topics_empty_back.at(topic) : 0);
    root["topics"][topic] = topic_info;
  }
  root["tags"] = Json::arrayValue;
  for (auto &tag : tags) {
    root["tags"].append(tag.toJson());
  }
  root["recordType"] = static_cast<Json::Int64>(type);
  return root;
}

void Record::addTag(int64_t stamp_ms, const std::string &name,
                    const std::string &desc) {
  RecordTag tag{stamp_ms, name, desc};
  tags_.push_back(tag);
}

void Record::insertTopic(const std::string &topic_name,
                         const std::string &type) {
  topics_.emplace(topic_name, type);
  types_cyber_.insert(std::make_pair(topic_name, type));
}

void Record::insertMsg(int64_t time, const std::string &topic_name,
                       const DynamicDataCyberPtr &sample) {
  auto type_it = types_cyber_.find(topic_name);
  if (TypeManagerCyber::getInstance().supported(type_it->first)) {
    buffer_->push(std::make_shared<MsgInstance>(time, topic_name, sample));
    return;
  }
  buffer_->push(std::make_shared<MsgInstance>(time, topic_name, sample));
}

RecordInfo Record::info() {
  RecordInfo info;
  info.tags = getTags();
  info.type = type_;
  info.filenames = getFiles();
  info.min = minTime();
  info.max = maxTime();
  info.duration = duration() * 1e9;
  info.total_num = size();

  // auto res = meta();
  // for (auto &kv : res) {
  //   info.extra[kv.key] = kv.val;
  // }

  info.topics = topics_;
  for (auto it = topics_.begin(); it != topics_.end(); ++it) {
    std::string name = it->first;
    if (hasTopic(name)) {
      int num = size(name);
      int64_t topic_start = begin(name)->second->time();
      int64_t topic_end = (--end(name))->second->time();
      info.topics_num[name] = num;
      info.topics_empty_front[name] = topic_start - info.min;
      info.topics_empty_back[name] = info.max - topic_end;
      info.topics_freq[name] = num / ((topic_end - topic_start) / 1e9);
    }
  }
  return info;
}

sim::FunctionId Record::noaType() {
  sim::FunctionId func_type{sim::FUNCTION_NONE};
  if (hasTopic("/st/pnc/sm_behavior")) {
    for (auto it = begin("/st/pnc/sm_behavior");
         it != end("/st/pnc/sm_behavior"); ++it) {
      auto raw_json = getRawStringJsonCyber(it->second);
      int func_id = 0;
      if (raw_json.isObject() && raw_json.isMember("func_id")) {
        func_id = raw_json["func_id"].asInt();
      }
      if (func_id == 1) {
        func_type = sim::FUNCTION_NOA;
        break;
      } else if (func_id == 7) {
        func_type = sim::FUNCTION_CITY_NOA;
        break;
      } else if (func_id == 8) {
        func_type = sim::FUNCTION_MAPLESS_NOA;
        break;
      }
    }
  }
  return func_type;
}

std::string Record::getRawDebugStringCyber(const MsgInstance::Ptr &msg) const {
  if (type_ != RecordType::RECORD_CYBER || !msg ||
      msg->data_cyber()->ByteSize() == 0 ||
      types_cyber_.find(msg->topic()) == types_cyber_.end()) {
    return "";
  }
  auto rawFactory = apollo::cyber::message::ProtobufFactory::Instance();
  auto raw_msg_class =
      rawFactory->GenerateMessageByType(types_cyber_.at(msg->topic()));
  if (!raw_msg_class) {
    return "";
  }

  std::unique_ptr<google::protobuf::Message> up_msg(raw_msg_class);
  std::string serialize_str;
  msg->data_cyber()->SerializeToString(&serialize_str);
  up_msg->ParseFromString(serialize_str);
  return up_msg->DebugString();
}

Json::Value Record::getRawStringJsonCyber(const MsgInstance::Ptr &msg) const {
  Json::Value json_value;
  if (type_ != RecordType::RECORD_CYBER || !msg ||
      msg->data_cyber()->ByteSize() == 0 ||
      types_cyber_.find(msg->topic()) == types_cyber_.end()) {
    return json_value;
  }
  auto rawFactory = apollo::cyber::message::ProtobufFactory::Instance();
  auto raw_msg_class =
      rawFactory->GenerateMessageByType(types_cyber_.at(msg->topic()));
  if (!raw_msg_class) {
    return json_value;
  }

  std::unique_ptr<google::protobuf::Message> up_msg(raw_msg_class);
  std::string serialize_str;
  msg->data_cyber()->SerializeToString(&serialize_str);
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
  } catch (std::exception &e) {
    LOG_ERROR << "Failed to parse JSON: " << e.what();
  }
  return json_value;
}
}  // namespace worldview