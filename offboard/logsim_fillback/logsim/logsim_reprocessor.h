#pragma once
#include <json/json.h>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "cyber/service_discovery/specific_manager/node_manager.h"
#include "modules/msg/st_msgs/planning_result.pb.h"
#include "modules/msg/st_msgs/planning_debug_frame.pb.h"
#include "modules/msg/st_msgs/sm_behavior.pb.h"
#include "bag/record.h"
#include "pubsub/base_pub.h"
#include "msg/cyber/supported_types_cyber.h"

namespace worldview {
enum LogsimMode {
    LOGSIM_OPEN_LOOP = 0,
    LOGSIM_CLOSED_LOOP = 1
};
struct LogsimModule {
  LogsimModule() = default;
  LogsimModule(const Json::Value&);

  std::string work_dir = "";
  std::string command = "";
  LogsimMode mode = LogsimMode::LOGSIM_OPEN_LOOP;

  Json::Value toJson() const;
};

struct PlanFillbackRecordedMessage {
  std::string topic;
  std::string plan_data;
  uint64_t timestamp;
  std::string msg_type;
};
struct CompareTimestamp {
  bool operator()(const PlanFillbackRecordedMessage& a,
                  const PlanFillbackRecordedMessage& b) const {
    return a.timestamp > b.timestamp;
  }
};

using PlanFillbackRecordedMessageQueue =
    std::priority_queue<PlanFillbackRecordedMessage,
                        std::vector<PlanFillbackRecordedMessage>,
                        CompareTimestamp>;

class LogsimReprocessor {
 public:
  using Ptr = std::shared_ptr<LogsimReprocessor>;

  LogsimReprocessor(const LogsimModule& config, Record::Ptr record)
      : config_(config), record_(record){};
  ~LogsimReprocessor() { stop(); }

  void start();
  void stop();
  void setOutputPath(const std::string& output_path) {
    output_path_ = output_path;
  }
  bool running() const { return is_running_.load(std::memory_order_acquire); };
  std::vector<pid_t> getPids() const { return root_pids_; }

 private:
  void InitPubSub();
  worldview::MsgBuffer::iterator FindTargetSeq(
      const std::string& topic, uint32_t tar_seq,
      worldview::MsgBuffer::iterator search_start_it);

  bool GetPlanReqSeq(const worldview::MsgBuffer::iterator debugframe_iter,
                     std::map<std::string, uint32_t>& req_sub_seqs);

  bool PreparePlannerState(worldview::MsgBuffer::iterator& planres_iter,
                           worldview::MsgBuffer::iterator& debugframe_iter);
  bool PreparePrevTraj(const worldview::MsgBuffer::iterator& planres_iter);
  bool PrepareObsStopTime(
      const worldview::MsgBuffer::iterator& debugframe_iter,
      const worldview::MsgBuffer::iterator& prediction_iter);
  bool CheckPlanNodeExist();
  bool StartPlanNode();
  void WriteOriginMsgs(
      PlanFillbackRecordedMessageQueue& planres_fillback_queue,
      PlanFillbackRecordedMessageQueue& plandbg_fillback_queue);
  void ModifyLocalizationMsg(
      worldview::MsgBuffer::iterator& dr_iter,
      const byd::msg::planning::TrajectoryPoint& lerp_point);
  void ModifyCanbusMsg(worldview::MsgBuffer::iterator& canbus_iter,
                       const byd::msg::planning::TrajectoryPoint& lerp_point);
  void ModifyVmcMsg(worldview::MsgBuffer::iterator& vmc_iter,
                    const byd::msg::planning::TrajectoryPoint& lerp_point);
  void ModifyLocalizationMsgFallback(
      worldview::MsgBuffer::iterator& dr_iter,
      const worldview::MsgBuffer::iterator& origin_dr_iter);

  void PreventTakeOver(worldview::MsgBuffer::iterator& behavior_iter);

  std::shared_ptr<google::protobuf::Message> getSpPtrMsg(
      const std::string topic, worldview::MsgBuffer::iterator iter);
  double getMeasurementTimestamp(
      std::shared_ptr<google::protobuf::Message> up_msg);

  LogsimModule config_;
  Record::Ptr record_;
  std::atomic<bool> is_running_{false};
  std::map<std::string, BasePub::Ptr> replay_topic_pubs_;
  NodeCyberPtr logsim_cyber_node_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<apollo::cyber::proto::InjectInfo>>
      inject_writer_ = nullptr;
  std::shared_ptr<
      apollo::cyber::Reader<byd::msg::planning::PLanningResultProto>>
      plan_res_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<byd::msg::planning::DebugFrameProto>>
      plan_dbgframe_reader_ = nullptr;
  // TODO: 从前端获取回灌所需的topic
  const std::multimap<std::string, std::string> kSeqTopicPairs{
      {"vehicle_status_seq", "/drivers/canbus/vehicle_info"},  // old topic
      {"vehicle_status_seq", "/drivers/canbus/canbus_uplink"},
      {"odometry_seq", "/localization/dr"},
      //   {"env_map_seq", "/perception/env/routing_map"},
      {"map_ptr_seq", "/perception/env/routing_map"},  // new seq
      {"lite_map_ptr_seq", "/noa_map/routing_map"},
      {"map_event_seq", "/noa_map/map_event"},
      {"vmc_debug_seq", "/mpc_interface_drv_vmc_Debug"},
      {"behavior_seq", "/st/pnc/sm_behavior"},
      {"prediction_obstacles_seq", "/prediction/trajectory_v2"},
  };

  const std::unordered_map<std::string, std::string> kTopicTypeMap{
      {"/drivers/canbus/canbus_uplink", "byd.msg.drivers.CanbusUpLink"},
      {"/drivers/canbus/vehicle_info", "byd.msg.drivers.VehInfo"},
      {"/localization/dr", "byd.msg.localization.LocalizationEstimate"},
      {"/perception/env/routing_map", "byd.msg.orin.routing_map.RoutingMap"},
      {"/noa_map/routing_map", "byd.msg.orin.routing_map.RoutingMap"},
      {"/noa_map/map_event", "byd.msg.orin.routing_map.MapEvent"},
      {"/mpc_interface_drv_vmc_Debug", "byd.msg.orin.vmc_msgs.MsgVmcDebug"},
      {"/st/pnc/sm_behavior", "byd.msg.planning.SMBehavior"},
      {"/prediction/trajectory_v2", "byd.msg.prediction.ObjectsPrediction"},
      {"/st/pnc/pilot_planning_result_old",
       "byd.msg.planning.PLanningResultProto"},
      {"/st/pnc/planning_debugframe_old", "byd.msg.planning.DebugFrameProto"},
      {"/st/pnc/pilot_planning_result", "byd.msg.planning.PLanningResultProto"},
      {"/st/pnc/planning_debugframe", "byd.msg.planning.DebugFrameProto"},
  };
  // 闭环时需修改的上游topic
  const std::vector<std::string> hacked_upstream_topics_{
      "/localization/dr",
      "/drivers/canbus/canbus_uplink",
      "/mpc_interface_drv_vmc_Debug",
      "/st/pnc/sm_behavior", // 仅在overtake时修改
  };

  const std::string kDebugframeTopic = "/st/pnc/planning_debugframe";
  const std::string kPlanResultTopic = "/st/pnc/pilot_planning_result";

  std::string save_path_ = "";
  std::string output_path_ = "";
  int64_t prev_process_time_ = 0;
  std::vector<pid_t> root_pids_{0};
  RecordWriterCyberPtr record_writer_ptr_ = nullptr;
  int64_t record_start_t_ = INT64_MAX;
  int64_t record_end_t_ = 0;
  int64_t planres_start_t_ = 0;
  int64_t plandbg_start_t_ = 0;
  const std::map<std::string, const google::protobuf::Descriptor*>
      supported_types_map_ = getSupportedCyberTypes();

  std::map<std::string, worldview::MsgBuffer::iterator> last_upstream_iters_;
  byd::msg::planning::Trajectory fillback_traj_res_;
  byd::msg::planning::FunctionId last_valid_func_id_ =
      byd::msg::planning::FunctionId::FUNCTION_NONE;
};
}  // namespace worldview