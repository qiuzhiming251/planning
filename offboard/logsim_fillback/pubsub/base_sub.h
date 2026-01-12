#pragma once
#include <google/protobuf/util/json_util.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "cyber/message/protobuf_factory.h"
#include "json/json.h"
#include "common/log.h"
#include "msg/typedefs.h"
#include "pubsub/common.h"

namespace worldview {
using ProcessFunc = std::function<bool(const DynamicDataCyberPtr &raw_msg)>;
class BaseSub {
 public:
  using Ptr = std::shared_ptr<BaseSub>;

  BaseSub(const NodeCyberPtr node_cyber_ptr, const std::string &topic_name,
          const std::string &topic_type)
      : node_cyber_ptr_(node_cyber_ptr),
        channel_name_cyber_(topic_name),
        type_cyber_(topic_type) {
    apollo::cyber::ReaderConfig config;
    config.channel_name = channel_name_cyber_;
    if (topic_type == "byd.msg.drivers.Image") {
      config.qos_profile.set_depth(100);
      config.pending_queue_size = 100;
    }
    channel_reader_cyber_ptr_ = node_cyber_ptr_->CreateReader<DynamicDataCyber>(
        config, [this](const DynamicDataCyberPtr &raw_msg) {
          msg_cnt_++;
          auto receive_time = std::chrono::system_clock::now();
          auto since_last_ms =
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  receive_time - last_update_time_)
                  .count();
          auto now = std::chrono::system_clock::now();
          auto duration = now.time_since_epoch();
          source_time_ =
              std::chrono::duration_cast<std::chrono::seconds>(duration)
                  .count();
          if (since_last_ms > 1000) {
            hz_ = 1000.0 * msg_cnt_ / since_last_ms;
            last_update_time_ = receive_time;
            msg_cnt_ = 0;
          }
          if (func_) {
            func_(raw_msg);
          }
          process(raw_msg);
        });
    if (channel_reader_cyber_ptr_ == nullptr) {
      LOG_ERROR << "CreateReader failed";
    }
  }

  virtual ~BaseSub() {
    if (channel_reader_cyber_ptr_) {
      channel_reader_cyber_ptr_.reset();
    }
  }

  double source_time() const { return source_time_; }
  int32_t hz() const { return hz_; }

  int32_t domainId() const { return domain_id_; }
  const NodeCyberPtr nodeCyberSubPtr() const { return node_cyber_ptr_; }
  const std::string channelNameCyber() const { return channel_name_cyber_; }
  const std::string typeCyber() const { return type_cyber_; }

  const SubSetting setting() const {
    if (!channel_reader_cyber_ptr_ && channelNameCyber().empty() &&
        typeCyber().empty()) {
      return {nodeCyberSubPtr(), domainId(), channelNameCyber(), typeCyber()};
    }
    return {nodeCyberSubPtr(), domainId(), channelNameCyber(), typeCyber()};
  }
  void setDomainId(int id) {
    if (domainId() == id) {
      return;
    }
    domain_id_ = id;
  }

  void setHandleFunc(const ProcessFunc &func) { func_ = func; }

 protected:
  const std::string topicNameCyber() const {
    if (channel_reader_cyber_ptr_) {
      return channel_reader_cyber_ptr_->GetChannelName();
    }
    return "";
  }
  const std::string getTypeCyber() const { return type_cyber_; }

  virtual bool process(const DynamicDataCyberPtr &raw_msg) { return true; }

  const NodeCyberPtr node_cyber_ptr_ = nullptr;
  std::string channel_name_cyber_;
  std::string type_cyber_;
  DynamicDataChannelReaderCyberPtr channel_reader_cyber_ptr_;

  int32_t domain_id_ = 0;
  int32_t hz_ = 0;
  int32_t msg_cnt_ = 0;
  double source_time_ = 0;
  std::chrono::system_clock::time_point last_update_time_ =
      std::chrono::system_clock::now();
  ProcessFunc func_;
};
}  // namespace worldview
