#pragma once

#include <memory>

#include "cyber/proto/role_attributes.pb.h"
#include "cyber/transport/qos/qos_profile_conf.h"
#include "msg/typedefs.h"
#include "pubsub/common.h"
namespace worldview {

class BasePub {
 public:
  using Ptr = std::shared_ptr<BasePub>;
  BasePub(const NodeCyberPtr node_cyber, const std::string &channel_name,
          const std::string &msg_type, int32_t domain_id = 0)
      : node_cyber_ptr_(node_cyber),
        channel_name_cyber_(channel_name),
        msg_type_cyber_(msg_type),
        domain_id_(domain_id) {
    apollo::cyber::proto::RoleAttributes attr;
    attr.set_channel_name(channel_name);
    attr.set_message_type(msg_type);
    const uint32_t kBufferSize = 2;
    auto qos_profile =
        apollo::cyber::transport::QosProfileConf::CreateQosProfile(
            apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST,
            kBufferSize,
            apollo::cyber::transport::QosProfileConf::QOS_MPS_SYSTEM_DEFAULT,
            apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_BEST_EFFORT,
            apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_VOLATILE);
    attr.mutable_qos_profile()->CopyFrom(qos_profile);
    writer_cyber_ = node_cyber_ptr_->CreateWriter<DynamicDataCyber>(attr);
  }

  virtual ~BasePub() {
    if (writer_cyber_) {
      writer_cyber_.reset();
    }
  }

  PubSetting setting() const {
    return {.node_cyber = node_cyber(),
            .domain_id = domain_id(),
            .name = channel_name_cyber(),
            .type = msg_type_cyber()};
  }

  bool setDomainId(int32_t domain_id) {
    domain_id_ = domain_id;
    return true;
  }

  void pub(const DynamicDataCyberPtr &data) { writer_cyber_->Write(data); }
  const std::string channel_name_cyber() const { return channel_name_cyber_; }
  const std::string msg_type_cyber() const { return msg_type_cyber_; }
  const NodeCyberPtr node_cyber() const { return node_cyber_ptr_; }
  const int32_t domain_id() const { return domain_id_; }

 private:
  const NodeCyberPtr node_cyber_ptr_;
  std::string channel_name_cyber_;
  std::string msg_type_cyber_;
  DynamicDataWriterCyberPtr writer_cyber_;
  int32_t domain_id_;
};
}  // namespace worldview