#include "logsim/logsim_reprocessor.h"
#include "modules/cnoa_pnc/planning/proto/prediction.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory.pb.h"
#include "modules/msg/st_msgs/planning_debug_frame.pb.h"
#include "modules/msg/prediction_msgs/pred_obj_info_v2.pb.h"
#include "modules/msg/localization_msgs/localization_info.pb.h"
#include "modules/msg/drivers_msgs/canbus_uplink.pb.h"
#include "modules/msg/orin_msgs/vmc_msgs.pb.h"
#include "common/command_util.h"
#include "common/file_util.h"
#include "common/filesystem.h"
#include "common/log.h"
#include "common/linear_interpolation.h"

#include <chrono>
#include <optional>
#include <regex>
#include <thread>
#include <type_traits>

namespace worldview {

template <typename T>
std::string GetRawMsgStr(const T& msg) {
  if constexpr (std::is_same_v<T, std::string>) {
    return msg;
  } else {
    return msg.str();
  }
}

std::shared_ptr<google::protobuf::Message> LogsimReprocessor::getSpPtrMsg(
    const std::string topic, worldview::MsgBuffer::iterator iter) {
  const auto& types_cyber = record_->getTopicsCyber();
  if (types_cyber.find(topic) == types_cyber.end()) {
    LOG_ERROR << "[logsim] Fail due to undefined type, topic: " << topic;
    return nullptr;
  }
  auto rawFactory = apollo::cyber::message::ProtobufFactory::Instance();
  auto raw_msg_class = rawFactory->GenerateMessageByType(types_cyber.at(topic));
  if (!raw_msg_class) {
    LOG_ERROR << "[logsim] Fail due to msg type convert, topic: " << topic;
    return nullptr;
  }
  std::shared_ptr<google::protobuf::Message> up_msg(raw_msg_class);
  std::string serialize_str;
  iter->second->data_cyber()->SerializeToString(&serialize_str);
  up_msg->ParseFromString(serialize_str);
  return up_msg;
}

double LogsimReprocessor::getMeasurementTimestamp(
    std::shared_ptr<google::protobuf::Message> up_msg) {
  if (!up_msg) {
    return false;
  }
  const google::protobuf::Descriptor* up_msg_desc = up_msg->GetDescriptor();
  const google::protobuf::Reflection* up_msg_refl = up_msg->GetReflection();
  if (!up_msg_desc || !up_msg_refl) {
    return false;
  }
  const google::protobuf::FieldDescriptor* header_field =
      up_msg_desc->FindFieldByName("header");
  if (!header_field) {
    return false;
  }
  if (header_field->type() != google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    return false;
  }
  const google::protobuf::Message& header_msg =
      up_msg_refl->GetMessage(*up_msg, header_field);
  const google::protobuf::Descriptor* header_msg_desc =
      header_msg.GetDescriptor();
  const google::protobuf::Reflection* header_msg_refl =
      header_msg.GetReflection();
  if (!header_msg_desc || !header_msg_refl) {
    return false;
  }
  const google::protobuf::FieldDescriptor* mt_field =
      header_msg_desc->FindFieldByName("measurement_timestamp");
  if (!mt_field) {
    return false;
  }
  if (mt_field->type() != google::protobuf::FieldDescriptor::TYPE_DOUBLE) {
    return false;
  }
  double mt = header_msg_refl->GetDouble(header_msg, mt_field);
  // LOG_ERROR << std::fixed << std::setprecision(6) << "mt:" << mt;
  return mt;
};

LogsimModule::LogsimModule(const Json::Value& module_j) {
  command = module_j["command"].asString();
  work_dir = module_j["work_dir"].asString();
}

Json::Value LogsimModule::toJson() const {
  Json::Value m;
  m["work_dir"] = work_dir;
  m["command"] = command;
  m["pubs"] = Json::Value(Json::arrayValue);
  return m;
}

std::optional<double> getDoubleFromString(const std::string& str,
                                          const std::string& name) {
  std::string pattern = std::regex_replace(
      name, std::regex(R"([\^\$\.\*\+\?\(\)\[\]\{\}\|\\])"), R"(\$&)");
  pattern += R"(([-+]?\d*\.\d+|[-+]?\d+))";

  std::regex regex_pattern(pattern);
  std::smatch matches;

  if (std::regex_search(str, matches, regex_pattern) && matches.size() > 1) {
    try {
      return std::stod(matches[1].str());
    } catch (...) {
      return std::nullopt;
    }
  }

  return std::nullopt;
}

void LogsimReprocessor::InitPubSub() {
  if (!logsim_cyber_node_) {
    logsim_cyber_node_ = apollo::cyber::CreateNode("logsim_reader_node");
  }
  for (const auto& topic_type : kTopicTypeMap) {
    const std::string& topic = topic_type.first;
    const std::string& msg_type = topic_type.second;
    replay_topic_pubs_[topic] =
        std::make_shared<BasePub>(logsim_cyber_node_, topic, msg_type);
    record_writer_ptr_->WriteChannel(
        topic, msg_type,
        supported_types_map_.at(msg_type)
            ->DebugString());  // TODO: add descriptor string
  }
  record_writer_ptr_->WriteChannel(
      "/st/pnc/pilot_planning_result", "byd.msg.planning.PLanningResultProto",
      supported_types_map_.at("byd.msg.planning.PLanningResultProto")
          ->DebugString());
  record_writer_ptr_->WriteChannel(
      "/st/pnc/planning_debugframe", "byd.msg.planning.DebugFrameProto",
      supported_types_map_.at("byd.msg.planning.DebugFrameProto")
          ->DebugString());
}

worldview::MsgBuffer::iterator LogsimReprocessor::FindTargetSeq(
    const std::string& topic, uint32_t tar_seq,
    worldview::MsgBuffer::iterator search_start_it) {
  if (record_->begin(topic) == record_->end(topic) ||
      search_start_it == record_->end(topic)) {
    LOG_ERROR << "[logsim] 1 Fail due to start iter, topic: " << topic
              << ", tar_seq: " << tar_seq;
    return record_->end(topic);
  }

  const auto& types_cyber = record_->getTopicsCyber();
  if (types_cyber.find(topic) == types_cyber.end()) {
    LOG_ERROR << "[logsim] Fail due to undefined type, topic: " << topic
              << ", tar_seq: " << tar_seq;
    return record_->end(topic);
  }
  auto rawFactory = apollo::cyber::message::ProtobufFactory::Instance();
  auto raw_msg_class = rawFactory->GenerateMessageByType(types_cyber.at(topic));
  if (!raw_msg_class) {
    LOG_ERROR << "[logsim] Fail due to msg type convert, topic: " << topic
              << ", tar_seq: " << tar_seq;
    return record_->end(topic);
  }
  std::shared_ptr<google::protobuf::Message> up_msg(raw_msg_class);
  std::string serialize_str;
  search_start_it->second->data_cyber()->SerializeToString(&serialize_str);
  up_msg->ParseFromString(serialize_str);

  auto getHeaderSequenceNum =
      [](std::shared_ptr<google::protobuf::Message> up_msg,
         uint32_t* sequence_num) -> bool {
    if (!up_msg || !sequence_num) {
      return false;
    }
    const google::protobuf::Descriptor* up_msg_desc = up_msg->GetDescriptor();
    const google::protobuf::Reflection* up_msg_refl = up_msg->GetReflection();
    if (!up_msg_desc || !up_msg_refl) {
      return false;
    }
    const google::protobuf::FieldDescriptor* header_field =
        up_msg_desc->FindFieldByName("header");
    if (!header_field) {
      return false;
    }
    if (header_field->type() !=
        google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      return false;
    }
    const google::protobuf::Message& header_msg =
        up_msg_refl->GetMessage(*up_msg, header_field);
    const google::protobuf::Descriptor* header_msg_desc =
        header_msg.GetDescriptor();
    const google::protobuf::Reflection* header_msg_refl =
        header_msg.GetReflection();
    if (!header_msg_desc || !header_msg_refl) {
      return false;
    }
    const google::protobuf::FieldDescriptor* seq_field =
        header_msg_desc->FindFieldByName("sequence_num");
    if (!seq_field) {
      return false;
    }
    if (seq_field->type() != google::protobuf::FieldDescriptor::TYPE_UINT32) {
      return false;
    }
    *sequence_num = header_msg_refl->GetUInt32(header_msg, seq_field);
    return true;
  };

  uint32_t start_seq = 0;
  if (!getHeaderSequenceNum(up_msg, &start_seq)) {
    LOG_ERROR << "[logsim] Fail due to no seq found, topic: " << topic
              << ", tar_seq: " << tar_seq;
    return record_->end(topic);
  }

  if (start_seq == tar_seq) {
    return search_start_it;
  }

  bool is_search_dir_dec = start_seq > tar_seq;
  auto search_it = search_start_it;
  while (search_it != record_->end(topic)) {
    search_it->second->data_cyber()->SerializeToString(&serialize_str);
    up_msg->ParseFromString(serialize_str);
    uint32_t search_seq = 0;
    if (!getHeaderSequenceNum(up_msg, &search_seq)) {
      LOG_ERROR << "[logsim] Fail due to no seq found, topic: " << topic
                << ", tar_seq: " << tar_seq;
      return record_->end(topic);
    }

    if (search_seq == tar_seq) {
      return search_it;
    }
    if ((search_seq > tar_seq && !is_search_dir_dec) ||
        (search_seq < tar_seq && is_search_dir_dec)) {
      LOG_WARN << "[logsim] 2 Fail due to seq skip, topic: " << topic
               << ", tar_seq: " << tar_seq;
      ;
      return record_->end(topic);
    }

    if (is_search_dir_dec) {
      if (search_it == record_->begin(topic)) {
        LOG_ERROR << "[logsim] 3 Fail due to search to begin, topic: " << topic
                  << ", tar_seq: " << tar_seq;
        ;
        return record_->end(topic);
      }
      --search_it;
    } else {
      ++search_it;
    }
  }

  // 没有有效消息
  LOG_ERROR << "[logsim] 4 Fail due to search to end, topic: " << topic
            << ", tar_seq: " << tar_seq;
  ;
  return record_->end(topic);
}

bool LogsimReprocessor::GetPlanReqSeq(
    const worldview::MsgBuffer::iterator debugframe_iter,
    std::map<std::string, uint32_t>& req_sub_seqs) {
  if (debugframe_iter == record_->end(kDebugframeTopic)) {
    return false;
  }
  std::string serialize_str;
  debugframe_iter->second->data_cyber()->SerializeToString(&serialize_str);
  std::shared_ptr<byd::msg::planning::DebugFrameProto> dbg_data =
      std::make_shared<byd::msg::planning::DebugFrameProto>();
  dbg_data->ParseFromString(serialize_str);

  const auto& debugframe_strings = dbg_data->strings();
  for (const auto& debug_str : debugframe_strings) {
    if (debug_str.has_name() &&
        kSeqTopicPairs.find(debug_str.name()) != kSeqTopicPairs.end() &&
        debug_str.has_value()) {
      std::string seq_str = debug_str.value();
      // 存在TIMEOUT，需检查sequence内容有效性
      try {
        uint32_t seq_num = std::stoul(seq_str);
        std::string debug_seq_name = debug_str.name();
        // key: topic name, value: sequence num
        // 适配vehicle_status新旧两个topic，使用multimap
        auto range = kSeqTopicPairs.equal_range(debug_seq_name);
        for (auto map_it = range.first; map_it != range.second; ++map_it) {
          req_sub_seqs[map_it->second] = seq_num;
        }
      } catch (const std::invalid_argument& e) {
        LOG_ERROR << "[logsim] Sequence transform failed: " << seq_str << ", "
                  << e.what();
        return false;
      } catch (...) {
        LOG_ERROR << "[logsim] Sequence transform failed: " << seq_str;
        return false;
      }
    }
  }
  if (req_sub_seqs.size() != kSeqTopicPairs.size()) {
    std::string unfound_seqs;
    for (const auto& seq_topic_pair : kSeqTopicPairs) {
      if (req_sub_seqs.find(seq_topic_pair.second) == req_sub_seqs.end()) {
        unfound_seqs = unfound_seqs + seq_topic_pair.second + ", ";
      }
    }
    LOG_WARN << "[logsim] Fail to find all seqs of required input topics: "
             << unfound_seqs;
    return false;
  }
  return true;
}

bool LogsimReprocessor::PreparePlannerState(
    worldview::MsgBuffer::iterator& planres_iter,
    worldview::MsgBuffer::iterator& debugframe_iter) {
  if (planres_iter == record_->end(kPlanResultTopic) ||
      debugframe_iter == record_->end(kDebugframeTopic)) {
    LOG_ERROR
        << "[logsim] Prepare planner_state fail: iter reaches record's end";
    return false;
  }
  // if (config_.command.empty()) {
  //   LOG_ERROR << "[logsim] Prepare planner_state quit: empty workspace";
  //   return false;
  // }
  // std::string real_work_dir = util::getRealPath(config_.work_dir);
  std::string real_work_dir = "/apollo";
  if (!fs::exists(real_work_dir) || !fs::is_directory(real_work_dir)) {
    LOG_ERROR << "[logsim] Prepare planner_state quit: empty workspace2";
    return false;
  }
  save_path_ = real_work_dir;
  if (save_path_.back() != '/') {
    save_path_ += "/";
  }

  if (!PreparePrevTraj(planres_iter)) {
    LOG_ERROR << "[logsim] Prepare prev traj failed";
  }

  // 获取上游信息
  std::map<std::string, uint32_t> prepare_sub_seqs;
  std::map<std::string, worldview::MsgBuffer::iterator> found_msgs;
  int64_t debugframe_time = debugframe_iter->first;
  if (GetPlanReqSeq(debugframe_iter, prepare_sub_seqs)) {
    for (const auto& topic_seq_pair : prepare_sub_seqs) {
      const auto& topic = topic_seq_pair.first;
      const auto& seq = topic_seq_pair.second;
      auto search_start_it = record_->floor(topic, debugframe_time);
      auto tar_msg_it = FindTargetSeq(topic, seq, search_start_it);
      if (tar_msg_it == record_->end(topic)) {
        LOG_ERROR << "[logsim] Prepare planner_state: Not find msg of " << topic
                  << " at seq: " << seq;
        break;
      }
      found_msgs[topic] = tar_msg_it;
    }
  }
  if (found_msgs.count("/prediction/trajectory_v2")) {
    if (!PrepareObsStopTime(debugframe_iter,
                            found_msgs["/prediction/trajectory_v2"])) {
      LOG_ERROR << "[logsim] Prepare obs stop time failed";
    }
  } else {
    LOG_ERROR << "[logsim] Not find prediction msg, obs stop time failed";
  }

  // 切到下一帧作为仿真起始帧
  ++debugframe_iter;
  ++planres_iter;
  return true;
}

bool LogsimReprocessor::PreparePrevTraj(
    const worldview::MsgBuffer::iterator& planres_iter) {
  const std::string traj_save_file = save_path_ + "prev_traj.data";
  // delete old data
  fs::path psave_file(traj_save_file);
  if (fs::exists(psave_file)) {
    fs::remove(psave_file);
    LOG_INFO << "[logsim] Delete " << traj_save_file << " succeed";
  }
  if (planres_iter == record_->end(kPlanResultTopic)) {
    LOG_ERROR << "[logsim] Prepare prev traj fail: iter reaches record's end";
    return false;
  }

  st::planning::TrajectoryProto prev_traj_proto;
  byd::msg::planning::PLanningResultProto prev_planres_data;
  std::string serialize_str;
  planres_iter->second->data_cyber()->SerializeToString(&serialize_str);
  prev_planres_data.ParseFromString(serialize_str);
  double prev_traj_start_timestamp = 0.0;

  // trajectory_start_timestamp
  if (prev_planres_data.has_debug() &&
      prev_planres_data.debug().has_start_point() &&
      prev_planres_data.debug().start_point().has_t()) {
    prev_traj_start_timestamp = prev_planres_data.debug().start_point().t();
    prev_traj_proto.set_trajectory_start_timestamp(prev_traj_start_timestamp);
  } else {
    LOG_ERROR << "[logsim] Prepare prev traj fail: Parse "
                 "trajectory_start_timestamp fail";
    return false;
  }

  // past_points & trajectory_point
  if (prev_planres_data.has_trajectory()) {
    const auto& traj_pts = prev_planres_data.trajectory().points();
    int traj_pts_num = 0;
    for (const auto& traj_pt : traj_pts) {
      try {
        st::ApolloTrajectoryPointProto* new_pt = nullptr;
        const double rel_t = traj_pt.t() - prev_traj_start_timestamp;
        if (rel_t >= -1e-5) {
          new_pt = prev_traj_proto.add_trajectory_point();
          traj_pts_num++;
        } else {
          new_pt = prev_traj_proto.add_past_points();
        }
        new_pt->mutable_path_point()->set_x(traj_pt.x());
        new_pt->mutable_path_point()->set_y(traj_pt.y());
        new_pt->mutable_path_point()->set_kappa(traj_pt.kappa());
        new_pt->mutable_path_point()->set_theta(traj_pt.theta());
        new_pt->mutable_path_point()->set_steer_angle(traj_pt.steering_angle());

        new_pt->set_relative_time(rel_t);
        new_pt->set_v(traj_pt.v());
        new_pt->set_a(traj_pt.a());
        new_pt->set_yaw_rate(traj_pt.yaw_rate());
        new_pt->set_is_extend(traj_pt.is_extend());
      } catch (std::exception& e) {
        LOG_ERROR << "[logsim] Prepare prev traj fail: Trajectory point "
                     "trans fail, "
                  << e.what();
        return false;
      } catch (...) {
        LOG_ERROR << "[logsim] Prepare prev traj fail: Trajectory point "
                     "trans fail";
        return false;
      }
    }
    // if (traj_pts_num != 80) {
    //   LOG_ERROR << "[logsim] Prepare prev traj fail: "
    //                "trajectory points size error: "
    //             << traj_pts_num;
    //   return false;
    // }
  } else {
    LOG_ERROR << "[logsim] Prepare prev traj fail: Parse "
                 "trajectory points fail";
    return false;
  }

  // 保存previous_trajectory
  std::ofstream os_traj_data(traj_save_file, std::ios::out | std::ios::binary);
  if (os_traj_data.is_open()) {
    if (!prev_traj_proto.SerializeToOstream(&os_traj_data)) {
      LOG_ERROR << "[logsim] Prepare prev traj fail: Proto data write fail";
      os_traj_data.close();
      return false;
    }
    os_traj_data.close();
  } else {
    LOG_ERROR << "[logsim] Prepare prev traj fail: Proto data open fail";
    return false;
  }

  return true;
}

bool LogsimReprocessor::PrepareObsStopTime(
    const worldview::MsgBuffer::iterator& debugframe_iter,
    const worldview::MsgBuffer::iterator& prediction_iter) {
  const std::string stoptime_save_file = save_path_ + "obs_stop_time.data";
  // delete old data
  fs::path psave_file(stoptime_save_file);
  if (fs::exists(psave_file)) {
    fs::remove(psave_file);
    LOG_INFO << "[logsim] Delete " << stoptime_save_file << " succeed";
  }
  if (debugframe_iter == record_->end(kDebugframeTopic) ||
      prediction_iter == record_->end("/prediction/trajectory_v2")) {
    LOG_ERROR
        << "[logsim] Prepare obs stoptime fail: iter reaches record's end";
    return false;
  }
  // prediction measurement timestamp (ObjectStopTimeResult.last_time需要)
  byd::msg::prediction::ObjectsPrediction prev_pred_data;
  std::string pred_serialize_str;
  prediction_iter->second->data_cyber()->SerializeToString(&pred_serialize_str);
  prev_pred_data.ParseFromString(pred_serialize_str);
  double pred_measurement_timestamp = 0.0;  // s
  if (prev_pred_data.has_header() &&
      prev_pred_data.header().has_measurement_timestamp()) {
    pred_measurement_timestamp =
        prev_pred_data.header().measurement_timestamp();
  } else {
    LOG_ERROR << "[logsim] Prepare obs stoptime fail: Get pred measurement "
                 "time failed";
    return false;
  }

  // 死活车 obs_stop_time
  std::string dbg_serialize_str;
  debugframe_iter->second->data_cyber()->SerializeToString(&dbg_serialize_str);
  byd::msg::planning::DebugFrameProto prev_debugframe_data;
  prev_debugframe_data.ParseFromString(dbg_serialize_str);

  st::ObjectsPredictionProto prev_pred_proto;
  // refer to msg_proxy.cpp, cast double (s) to int32 (us)
  prev_pred_proto.mutable_header()->set_timestamp(
      (pred_measurement_timestamp)*1e6);
  const auto& dbg_strs = prev_debugframe_data.strings();
  for (const auto& dbg_str : dbg_strs) {
    if (dbg_str.has_name() && dbg_str.name() == "obs_stop_time" &&
        dbg_str.has_value()) {
      std::string dbg_val = dbg_str.value();
      std::optional<double> obs_id = getDoubleFromString(dbg_val, "obs-id: ");
      std::optional<double> time_stop =
          getDoubleFromString(dbg_val, "time_stop: ");
      std::optional<double> previous_stop =
          getDoubleFromString(dbg_val, "previous_stop: ");
      std::optional<double> last_move =
          getDoubleFromString(dbg_val, "last_move: ");

      if (obs_id.has_value()) {
        st::ObjectPredictionProto* new_obj_proto =
            prev_pred_proto.add_objects();
        new_obj_proto->set_id(std::to_string(int32_t(obs_id.value())));
        st::ObjectStopTimeProto* new_stoptime_proto =
            new_obj_proto->mutable_stop_time();
        if (time_stop.has_value()) {
          new_stoptime_proto->set_time_duration_since_stop(time_stop.value());
        }
        if (previous_stop.has_value()) {
          new_stoptime_proto->set_previous_stop_time_duration(
              previous_stop.value());
        }
        if (last_move.has_value()) {
          new_stoptime_proto->set_last_move_time_duration(last_move.value());
        }
      }
    }
  }

  // 保存obs_stop_time
  std::ofstream os_stop_data(stoptime_save_file,
                             std::ios::out | std::ios::binary);
  if (os_stop_data.is_open()) {
    if (!prev_pred_proto.SerializeToOstream(&os_stop_data)) {
      LOG_ERROR << "[logsim] Prepare stoptime fail: Proto data write fail";
      os_stop_data.close();
      return false;
    }
    os_stop_data.close();
  } else {
    LOG_ERROR << "[logsim] Prepare stoptime fail: Proto data open fail";
    return false;
  }
  return true;
}

// 检查plan节点是否存在，若存在则用planning_result触发下一帧回灌
bool LogsimReprocessor::CheckPlanNodeExist() {
  auto topology = apollo::cyber::service_discovery::TopologyManager::Instance();
  std::vector<std::string> node_names;
  std::vector<apollo::cyber::proto::RoleAttributes> nodes;
  topology->node_manager()->GetNodes(&nodes);
  if (nodes.empty()) {
    LOG_ERROR << "no node found";
    return false;
  }
  for (auto& node : nodes) {
    if (node.node_name() == "byd_plan") {
      return true;
    }
  }
  return false;
}

void LogsimReprocessor::start() {
  record_writer_ptr_ = std::make_shared<RecordWriterCyber>();
  if (!record_writer_ptr_->Open(output_path_)) {
    record_writer_ptr_->Close();
    LOG_ERROR << "[logsim] Create record writer failed!";
    return;
  }
  record_writer_ptr_->SetSizeOfFileSegmentation(0);
  record_writer_ptr_->SetIntervalOfFileSegmentation(0);

  InitPubSub();

  if (!record_ || !record_->size()) {
    LOG_ERROR << "[logsim] Fillback failed: Record isn't loaded";
    stop();
    return;
  }

  if (!record_->hasTopic(kDebugframeTopic) ||
      !record_->hasTopic(kPlanResultTopic)) {
    LOG_ERROR << "[logsim] Fillback failed: No debugframe or plan result";
    stop();
    return;
  }

  for (const auto& seqtopic_pair : kSeqTopicPairs) {
    const auto& req_topic = seqtopic_pair.second;
    if (!record_->hasTopic(req_topic)) {
      LOG_ERROR << "[logsim] Fillback failed: No required topic: " << req_topic;
      stop();
      return;
    }
  }

  is_running_.store(true, std::memory_order_release);
  if (!inject_writer_) {
    if (!logsim_cyber_node_) {
      logsim_cyber_node_ = apollo::cyber::CreateNode("logsim_cyber_node");
    }
    inject_writer_ =
        logsim_cyber_node_->CreateWriter<apollo::cyber::proto::InjectInfo>(
            "/inject/sync");
  }
  apollo::cyber::proto::InjectInfo inject_msg =
      apollo::cyber::proto::InjectInfo();

  // 跳过开头0.5s，避免record开头处未找到更早的上游sequence
  const int64_t kStartSkipTime = 1 * 1000000000.0;
  const int64_t logsim_start_time = record_->minTime() + kStartSkipTime;
  auto debugframe_iter = record_->floor(kDebugframeTopic, logsim_start_time);
  int32_t plan_start_seq = -1;
  auto debug_start_data =
      record_->getRawStringJsonCyber(debugframe_iter->second);
  if (debug_start_data.isObject() && debug_start_data.isMember("header") &&
      debug_start_data["header"].isObject() &&
      debug_start_data["header"].isMember("sequence_num")) {
    plan_start_seq = debug_start_data["header"]["sequence_num"].asUInt();
  }
  auto plan_res_iter =
      FindTargetSeq(kPlanResultTopic, plan_start_seq,
                    record_->floor(kPlanResultTopic, logsim_start_time));
  if (!PreparePlannerState(plan_res_iter, debugframe_iter)) {
    LOG_ERROR << "[logsim] PreparePlannerState failed, logsim with no "
                 "previous_trajectory";
  }
  planres_start_t_ = plan_res_iter->first;
  plandbg_start_t_ = debugframe_iter->first;

  if (StartPlanNode()) {
    LOG_INFO << "[logsim] Start logsim plan node success";
  } else {
    LOG_ERROR << "[logsim] Start logsim plan node failed";
  }
  LOG_INFO << "Wait for plan node start...";
  // wait for plan start
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  if (CheckPlanNodeExist()) {
    LOG_INFO << "Detect plan node start!";
    if (!logsim_cyber_node_) {
      logsim_cyber_node_ = apollo::cyber::CreateNode("logsim_cyber_node");
    }
    if (logsim_cyber_node_ && !plan_res_reader_ && !plan_dbgframe_reader_) {
      plan_res_reader_ =
          logsim_cyber_node_
              ->CreateReader<byd::msg::planning::PLanningResultProto>(
                  "/st/pnc/pilot_planning_result");
      plan_dbgframe_reader_ =
          logsim_cyber_node_->CreateReader<byd::msg::planning::DebugFrameProto>(
              "/st/pnc/planning_debugframe");
      if (plan_res_reader_ && plan_dbgframe_reader_) {
        LOG_INFO << "create res_reader success";
        plan_res_reader_->SetHistoryDepth(1);
        plan_dbgframe_reader_->SetHistoryDepth(1);
      }
    }
    LOG_INFO << "[logsim] Planning node found, use planning_result as callback";
  } else {
    LOG_INFO << "[logsim] Planning node not found, timer logsim";
  }
  // 睡眠时间，最长不超过100ms
  const int64_t kSleepUnit = 0;

  PlanFillbackRecordedMessageQueue planres_fillback_message_queue;
  PlanFillbackRecordedMessageQueue plandbg_fillback_message_queue;
  while (running() && debugframe_iter != record_->end(kDebugframeTopic)) {
    int64_t cur_proc_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    std::map<std::string, uint32_t> request_sub_seqs;
    std::map<std::string, worldview::MsgBuffer::iterator> found_msgs;
    int64_t debugframe_time = debugframe_iter->first;
    if (GetPlanReqSeq(debugframe_iter, request_sub_seqs)) {
      for (const auto& topic_seq_pair : request_sub_seqs) {
        const auto& topic = topic_seq_pair.first;
        const auto& seq = topic_seq_pair.second;
        auto search_start_it = record_->floor(topic, debugframe_time);
        auto tar_msg_it = FindTargetSeq(topic, seq, search_start_it);
        if (tar_msg_it == record_->end(topic)) {
          LOG_WARN << "[logsim] Fail to find msg of " << topic
                   << " at seq: " << seq << ", don't send msg";
          continue;
          // stop();
          // return;
        }
        // LOG_INFO << "[logsim] Find msg of " << topic << " at seq: " << seq;
        found_msgs[topic] = tar_msg_it;
        if (tar_msg_it->first > 0) {
          record_start_t_ = std::min(record_start_t_, tar_msg_it->first);
          record_end_t_ = std::max(record_end_t_, tar_msg_it->first);
        }
      }
      if (config_.mode == LogsimMode::LOGSIM_CLOSED_LOOP) {
        for (const auto& hacked_topic : hacked_upstream_topics_) {
          if (last_upstream_iters_.count(hacked_topic) &&
              found_msgs.count(hacked_topic)) {
            for (auto it = std::next(last_upstream_iters_[hacked_topic]);
                 it != std::next(found_msgs[hacked_topic]); ++it) {
              // 闭环回灌时保持不overtake，修改sm_behavior
              if (hacked_topic == "/st/pnc/sm_behavior") {
                PreventTakeOver(it);
                continue;
              }
              // 闭环时插值，并修改原topic内容（车辆状态相关）
              auto mt = getMeasurementTimestamp(getSpPtrMsg(hacked_topic, it));
              // plan failed, empty traj, freeze ego vehicle
              if (fillback_traj_res_.points().size() < 1) {
                ModifyLocalizationMsgFallback(
                    it, last_upstream_iters_[hacked_topic]);
                continue;
              }

              auto prevPoint = fillback_traj_res_.points()[0];
              byd::msg::planning::TrajectoryPoint linearApproximationPoint;
              for (int index = 1; index < fillback_traj_res_.points().size();
                   index++) {
                auto curPoint = fillback_traj_res_.points()[index];
                if (mt < prevPoint.t()) {
                  auto w =
                      (mt - prevPoint.t()) / (curPoint.t() - prevPoint.t());
                  linearApproximationPoint = worldview::util::math::
                      InterpolateUsingLinearApproximation(prevPoint, curPoint,
                                                          w);
                  LOG_ERROR
                      << std::fixed << std::setprecision(6)
                      << "mt less than first, curPoint.t():" << curPoint.t()
                      << " prevPoint.t():" << prevPoint.t() << " mt:" << mt
                      << " w:" << w;
                  break;
                }

                if (curPoint.has_t() && mt <= curPoint.t()) {
                  // w = (t - t0) / (t1 - t0)
                  auto w =
                      (mt - prevPoint.t()) / (curPoint.t() - prevPoint.t());
                  // LOG_ERROR << std::fixed << std::setprecision(6)
                  //   << "find point, curPoint.t():" << curPoint.t()
                  //   << " prevPoint.t():" << prevPoint.t()
                  //   << " mt:" << mt
                  //   << " w:" << w;
                  linearApproximationPoint = worldview::util::math::
                      InterpolateUsingLinearApproximation(prevPoint, curPoint,
                                                          w);
                  // LOG_ERROR << std::fixed << std::setprecision(6)
                  //   << " resPoint.t():" << linearApproximationPoint.t();
                  break;
                } else {
                  prevPoint = curPoint;
                }
              }

              if ("/localization/dr" == hacked_topic) {
                ModifyLocalizationMsg(it, linearApproximationPoint);
              } else if ("/drivers/canbus/canbus_uplink" == hacked_topic) {
                ModifyCanbusMsg(it, linearApproximationPoint);
              } else if ("/mpc_interface_drv_vmc_Debug" == hacked_topic) {
                ModifyVmcMsg(it, linearApproximationPoint);
              } else {
              }
            }
          }
        }
      }
      // 找到所有上游msg后统一发送
      for (const auto& found_msg : found_msgs) {
        const auto& topic = found_msg.first;
        const auto& target_msg_it = found_msg.second;
        replay_topic_pubs_[topic]->pub(target_msg_it->second->data_cyber());
      }
      last_upstream_iters_ = found_msgs;
      // CHECK:这里是否需要sleep一小段时间等待plan节点接收消息？
      inject_writer_->Write(inject_msg);
    }
    uint64_t origin_plan_timestamp = debugframe_iter->first;
    if (replay_topic_pubs_.count("/st/pnc/planning_debugframe_old")) {
      replay_topic_pubs_["/st/pnc/planning_debugframe_old"]->pub(
          debugframe_iter->second->data_cyber());
    }
    if (debugframe_iter->first > 0) {
      record_start_t_ = std::min(record_start_t_, debugframe_iter->first);
      record_end_t_ = std::max(record_end_t_, debugframe_iter->first);
    }
    ++debugframe_iter;
    if (plan_res_iter != record_->end(kPlanResultTopic) &&
        replay_topic_pubs_.count("/st/pnc/pilot_planning_result_old")) {
      replay_topic_pubs_["/st/pnc/pilot_planning_result_old"]->pub(
          plan_res_iter->second->data_cyber());
      if (plan_res_iter->first > 0) {
        record_start_t_ = std::min(record_start_t_, plan_res_iter->first);
        record_end_t_ = std::max(record_end_t_, plan_res_iter->first);
      }
      ++plan_res_iter;
    }
    // plan节点运行中，则以plan result作为下一帧回灌的触发
    if (plan_res_reader_ && plan_dbgframe_reader_) {
      std::shared_ptr<byd::msg::planning::PLanningResultProto> fb_planres_msg =
          nullptr;
      std::shared_ptr<byd::msg::planning::DebugFrameProto> fb_plandgb_msg =
          nullptr;
      auto start_time = std::chrono::system_clock::now();
      bool is_timeout = false;
      constexpr int kTimeoutSec = 30;
      while (true) {
        try {
          plan_res_reader_->Observe();
          plan_dbgframe_reader_->Observe();
          if (!plan_res_reader_->Empty() && !plan_dbgframe_reader_->Empty()) {
            fb_planres_msg = plan_res_reader_->GetLatestObserved();
            fb_plandgb_msg = plan_dbgframe_reader_->GetLatestObserved();
            std::string planres_str, plandgb_str;
            fb_planres_msg->SerializeToString(&planres_str);
            fb_plandgb_msg->SerializeToString(&plandgb_str);
            // record_writer_ptr_->WriteMessage("/st/pnc/pilot_planning_result",
            //                                  *fb_planres_msg,
            //                                  origin_plan_timestamp);
            planres_fillback_message_queue.push(
                {"/st/pnc/pilot_planning_result", planres_str,
                 origin_plan_timestamp,
                 "byd.msg.planning.PLanningResultProto"});
            plandbg_fillback_message_queue.push(
                {"/st/pnc/planning_debugframe", plandgb_str,
                 origin_plan_timestamp, "byd.msg.planning.DebugFrameProto"});
            // save planning trajectory result
            if (config_.mode == LogsimMode::LOGSIM_CLOSED_LOOP) {
              fillback_traj_res_ = fb_planres_msg->trajectory();
            }
            plan_res_reader_->ClearData();
            plan_dbgframe_reader_->ClearData();
            break;
          }
          auto now_time = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsed_time = now_time - start_time;
          if (elapsed_time.count() > kTimeoutSec) {
            is_timeout = true;
            break;
          }
        } catch (std::exception& e) {
          LOG_ERROR << "[logsim] Get plan res failed: " << e.what();
          break;
        } catch (...) {
          LOG_ERROR << "[logsim] Get plan res failed";
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      if (is_timeout) {
        LOG_ERROR << "[logsim] Get Plan result timeout, stop logsim";
        break;
      }
    } else {
      int64_t time_to_sleep = prev_process_time_ == 0
                                  ? 100
                                  : 100 - (cur_proc_time - prev_process_time_);
      int64_t actual_sleep = std::max(std::min(time_to_sleep, kSleepUnit), 0L);
      prev_process_time_ = cur_proc_time;
      std::this_thread::sleep_for(std::chrono::milliseconds(actual_sleep));
    }
  }
  WriteOriginMsgs(planres_fillback_message_queue,
                  plandbg_fillback_message_queue);
  stop();
  return;
}

void LogsimReprocessor::stop() {
  if (running()) {
    is_running_.store(false, std::memory_order_release);
  }
  replay_topic_pubs_.clear();
  for (const auto& root_pid : root_pids_) {
    util::killDescendants(root_pid, SIGTERM);
  }
  if (record_writer_ptr_) {
    record_writer_ptr_->Close();
  }
}

bool LogsimReprocessor::StartPlanNode() {
  std::string cmd =
      "chmod 777 /apollo/modules/cnoa_pnc/planning/scripts/logsim_start.sh";
  int res = system(cmd.data());

  res = system(config_.command.data());
  std::string planning_process = "planning_logsim_batch_fillback.dag";
  root_pids_ = util::get_pids_by_command(planning_process);
  if (root_pids_.empty()) {
    LOG_INFO << "nof found'" << planning_process << "' process";
  } else {
    LOG_INFO << "found '" << planning_process << "' process PID: ";
    for (pid_t pid : root_pids_) {
      LOG_INFO << pid << " ";
    }
  }

  return true;
}

void LogsimReprocessor::WriteOriginMsgs(
    PlanFillbackRecordedMessageQueue& planres_fillback_queue,
    PlanFillbackRecordedMessageQueue& plandbg_fillback_queue) {
  auto pb_factory = apollo::cyber::message::ProtobufFactory::Instance();
  bool start_write_planres = false;
  bool start_write_plandbg = false;
  for (auto it = record_->begin(); it != record_->end(); ++it) {
    if (it->first < record_start_t_) {
      continue;
    }
    if (it->first > record_end_t_) {
      break;
    }
    std::string topic_name = it->second->topic();
    const auto& cyber_msg = it->second->data_cyber();
    // if (record_writer_ptr_->IsNewChannel(topic_name)) {
    //   continue;
    // }

    if (topic_name == "/st/pnc/pilot_planning_result" &&
        it->first == planres_start_t_) {
      start_write_planres = true;
    }
    if (topic_name == "/st/pnc/planning_debugframe" &&
        it->first == plandbg_start_t_) {
      start_write_plandbg = true;
    }

    const std::string& msg_type = kTopicTypeMap.at(topic_name);
    google::protobuf::Message* raw_msg =
        pb_factory->GenerateMessageByType(msg_type);
    if (!raw_msg) {
      AERROR << "Failed to create message for type: " << msg_type;
      continue;
    }

    if (!raw_msg->ParseFromString(cyber_msg->message)) {
      AERROR << "Failed to parse message for topic: " << topic_name;
      delete raw_msg;
      continue;
    }

    std::string new_topic = topic_name;
    if (topic_name == "/st/pnc/pilot_planning_result" &&
        !planres_fillback_queue.empty() && start_write_planres) {
      const auto& fillback_msg = planres_fillback_queue.top();
      new_topic += "_old";
      record_writer_ptr_->WriteMessage(topic_name, fillback_msg.plan_data,
                                       it->first, msg_type);
      // LOG_ERROR << "write new planres !!!" << " " << it->first;
      planres_fillback_queue.pop();
    } else if (topic_name == "/st/pnc/planning_debugframe" &&
               !plandbg_fillback_queue.empty() && start_write_plandbg) {
      const auto& fillback_msg = plandbg_fillback_queue.top();
      new_topic += "_old";
      record_writer_ptr_->WriteMessage(topic_name, fillback_msg.plan_data,
                                       it->first, msg_type);
      // LOG_ERROR << "write new plandbg !!!" << " " << it->first;
      plandbg_fillback_queue.pop();
    }

    record_writer_ptr_->WriteMessage(
        new_topic, GetRawMsgStr(cyber_msg->message), it->first, msg_type);
    delete raw_msg;
  }
}

void LogsimReprocessor::ModifyLocalizationMsg(
    worldview::MsgBuffer::iterator& dr_iter,
    const byd::msg::planning::TrajectoryPoint& lerp_point) {
  byd::msg::localization::LocalizationEstimate modified_dr_msg;
  std::string serialize_str;
  // copy origin msg
  dr_iter->second->data_cyber()->SerializeToString(&serialize_str);
  modified_dr_msg.ParseFromString(serialize_str);
  // Fill with closed-loop value
  modified_dr_msg.mutable_header()->set_frame_id("logsim-dr");
  modified_dr_msg.mutable_pose()->mutable_position()->set_x(lerp_point.x());
  modified_dr_msg.mutable_pose()->mutable_position()->set_y(lerp_point.y());

  util::Euler euler = util::Euler(0, 0, lerp_point.theta());
  util::Quaternion qte = euler.toQuaternion();
  modified_dr_msg.mutable_pose()->mutable_orientation()->set_qx(qte.x);
  modified_dr_msg.mutable_pose()->mutable_orientation()->set_qy(qte.y);
  modified_dr_msg.mutable_pose()->mutable_orientation()->set_qz(qte.z);
  modified_dr_msg.mutable_pose()->mutable_orientation()->set_qw(qte.w);

  modified_dr_msg.mutable_pose()->set_heading(lerp_point.theta() * 180.0 /
                                              M_PI);

  modified_dr_msg.mutable_pose()->mutable_linear_velocity()->set_x(
      lerp_point.v() * std::cos(lerp_point.theta()));
  modified_dr_msg.mutable_pose()->mutable_linear_velocity()->set_y(
      lerp_point.v() * std::sin(lerp_point.theta()));

  modified_dr_msg.mutable_pose()->mutable_linear_velocity()->set_z(
      lerp_point.yaw_rate());

  modified_dr_msg.SerializeToString(&serialize_str);
  dr_iter->second->data_cyber()->ParseFromString(serialize_str);
}

void LogsimReprocessor::ModifyCanbusMsg(
    worldview::MsgBuffer::iterator& canbus_iter,
    const byd::msg::planning::TrajectoryPoint& lerp_point) {
  byd::msg::drivers::CanbusUpLink modified_canbus_msg;
  std::string serialize_str;
  // copy origin msg
  canbus_iter->second->data_cyber()->SerializeToString(&serialize_str);
  modified_canbus_msg.ParseFromString(serialize_str);
  // Fill with closed-loop value
  modified_canbus_msg.mutable_header()->set_frame_id("logsim-uplink");
  modified_canbus_msg.mutable_eps_0x06d()->set_eps_steerwheelag(
      lerp_point.steering_angle() * 180.0 / M_PI);
  modified_canbus_msg.mutable_ipb_0x10c()->set_vehicle_speed(lerp_point.v());

  modified_canbus_msg.SerializeToString(&serialize_str);
  canbus_iter->second->data_cyber()->ParseFromString(serialize_str);
}

void LogsimReprocessor::ModifyVmcMsg(
    worldview::MsgBuffer::iterator& vmc_iter,
    const byd::msg::planning::TrajectoryPoint& lerp_point) {
  byd::msg::orin::vmc_msgs::MsgVmcDebug modified_vmc_msg;
  std::string serialize_str;
  // copy origin msg
  vmc_iter->second->data_cyber()->SerializeToString(&serialize_str);
  modified_vmc_msg.ParseFromString(serialize_str);
  // Fill with closed-loop value
  modified_vmc_msg.mutable_header()->set_frame_id("logsim-vmc");
  modified_vmc_msg.set_debug_cddaest(lerp_point.a());

  modified_vmc_msg.SerializeToString(&serialize_str);
  vmc_iter->second->data_cyber()->ParseFromString(serialize_str);
}

// Freeze ego vehicle pose when plan failed
void LogsimReprocessor::ModifyLocalizationMsgFallback(
    worldview::MsgBuffer::iterator& dr_iter,
    const worldview::MsgBuffer::iterator& origin_dr_iter) {
  byd::msg::localization::LocalizationEstimate modified_dr_msg, origin_dr_msg;
  std::string modify_serialize_str, origin_serialize_str;
  // copy origin msg
  dr_iter->second->data_cyber()->SerializeToString(&modify_serialize_str);
  modified_dr_msg.ParseFromString(modify_serialize_str);
  origin_dr_iter->second->data_cyber()->SerializeToString(&origin_serialize_str);
  origin_dr_msg.ParseFromString(origin_serialize_str);

  // Fill with closed-loop value
  modified_dr_msg.mutable_header()->set_frame_id("logsim-dr");
  // Copy the pose of last dr frame, freeze ego vehicle
  modified_dr_msg.mutable_pose()->CopyFrom(origin_dr_msg.pose());

  modified_dr_msg.SerializeToString(&modify_serialize_str);
  dr_iter->second->data_cyber()->ParseFromString(modify_serialize_str);
}

void LogsimReprocessor::PreventTakeOver(worldview::MsgBuffer::iterator& behavior_iter){
  byd::msg::planning::SMBehavior behavior_msg;
  std::string serialize_str;
  behavior_iter->second->data_cyber()->SerializeToString(&serialize_str);
  behavior_msg.ParseFromString(serialize_str);
  const auto& cur_func_id = behavior_msg.function_id();
  if (cur_func_id != byd::msg::planning::FunctionId::FUNCTION_HW_NOA &&
      cur_func_id != byd::msg::planning::FunctionId::FUNCTION_CITY_NOA) {
    // 未进功能，修改function_id为最接近一帧的有效值
    behavior_msg.mutable_header()->set_frame_id("logsim_sm_behavior");
    behavior_msg.set_function_id(last_valid_func_id_);
    behavior_msg.SerializeToString(&serialize_str);
    behavior_iter->second->data_cyber()->ParseFromString(serialize_str);
  } else {
    last_valid_func_id_ = cur_func_id;
  }
}

}  // namespace worldview