

#include "node_manager/task_runner/byd_planning_core.h"

#include <filesystem>

#include "glog/logging.h"
#include "cyber/time/time.h"
#include "plan_common/gflags.h"
#include "plan_common/git_version.h"
#include "plan_common/log.h"
#include "plan_common/log_data.h"
#include "plan_common/timer.h"
#include "plan_common/util/hr_timer.h"
#include "plan_common/util/utility.h"
#include "modules/cnoa_pnc/planning/proto/planner_status.pb.h"
namespace ad_byd {
namespace planning {

namespace {
constexpr int kPlanningAndPrediction = 0;
constexpr int kPlanningOnly = 1;
constexpr int kPredictionOnly = 2;

inline bool IsInputMapError(
    st::planning::PlannerStatusProto::PlannerStatusCode plan_result) {
  return plan_result ==
             st::planning::PlannerStatusProto::PLAN_MSG_MAP_TIMEOUT ||
         plan_result == st::planning::PlannerStatusProto::PLAN_FAIL_NO_MAP;
}

inline bool InputFrameValid(const PlanningInputFrame::Ptr& input_frame_ptr) {
  bool input_valid = true;
  if (input_frame_ptr) {
    if (!input_frame_ptr->behavior) {
      LOG_ERROR << "behavior is nullptr";
      input_valid = false;
    }
    if (!input_frame_ptr->env_map) {
      LOG_ERROR << "env_map is nullptr";
      input_valid = false;
    }
    if (!input_frame_ptr->prediction) {
      LOG_ERROR << "prediction is nullptr";
      input_valid = false;
    }
    if (!input_frame_ptr->vehicle_status) {
      LOG_ERROR << "vehicle_status is nullptr";
      input_valid = false;
    }
    if (!input_frame_ptr->odometry) {
      LOG_ERROR << "odometry is nullptr";
      input_valid = false;
    }
  } else {
    LOG_ERROR << "PlanningInputFrame is nullptr";
    input_valid = false;
  }

  return input_valid;
}

void FillPlanningMsgHeader(
    std::shared_ptr<planning_result_type>& planning_result,
    std::shared_ptr<debug_frame_type>& debug_frame, uint32_t sequence) {
  // for planning result
  planning_result->mutable_header()->set_sequence_num(sequence);
  planning_result->mutable_header()->set_publish_timestamp(
      apollo::cyber::Time::Now().ToSecond());
  // planning_result->mutable_header()->set_frame_id(
  //     absl::StrCat("NOA|", git_branch, "|", git_hash));

  // for debug frame
  debug_frame->mutable_header()->set_sequence_num(sequence);
  debug_frame->mutable_header()->set_publish_timestamp(
      apollo::cyber::Time::Now().ToSecond());
  debug_frame->mutable_header()->set_measurement_timestamp(
      planning_result->header().measurement_timestamp());
  // debug_frame->mutable_header()->set_frame_id(
  //     absl::StrCat("NOA|", git_branch, "|", git_hash));

  auto str = debug_frame->add_strings();
  str->set_name("planning_result_seq");
  str->set_value(absl::StrCat(sequence));
}
void FillPlanningAutoMode(planning_result_type* planning_result,
                          PlanningInputFrame::Ptr input_frame) {
  auto auto_mode = st::planning::AutoMode::Auto_None;
  if (input_frame && input_frame->behavior) {
    // 0:none, 1:noa, 2:apa, 3:acc, 4:acclka, 5:summon, 6:lka_plus, 7:city_noa
    const auto function_id = input_frame->behavior->function_id();
    // set function module and function_mode
    if (function_id == byd::msg::planning::FunctionId::FUNCTION_CITY_NOA ||
        function_id == byd::msg::planning::FunctionId::FUNCTION_MAPLESS_NOA ||
        function_id == byd::msg::planning::FunctionId::FUNCTION_HW_NOA ||
        function_id == byd::msg::planning::FunctionId::FUNCTION_LKA_PLUS) {
      auto_mode = st::planning::AutoMode::Auto_LaneChange;
    } else if (function_id == byd::msg::planning::FunctionId::FUNCTION_LKA) {
      auto_mode = st::planning::AutoMode::Auto_ACCLKA;
    } else if (function_id == byd::msg::planning::FunctionId::FUNCTION_ACC) {
      auto_mode = st::planning::AutoMode::Auto_ACC;
    } else {
      auto_mode = st::planning::AutoMode::Auto_None;
    }
  }
  planning_result->mutable_state()->set_auto_mode(auto_mode);
}

}  // namespace

bool BydPlanningCore::Init(std::string vehicle_config_file_path) {
  // google::InitGoogleLogging("byd-planning"); // called InitGoogleLogging()
  // twice!
  // FLAGS_log_dir = "/tmp";
  // if (std::filesystem::exists("/log/glog")) {
  //   FLAGS_log_dir = "/log/glog";
  // }
  // FLAGS_alsologtostderr = true;
  // FLAGS_minloglevel = 2;
  // FLAGS_max_log_size = 10;
  // google::EnableLogCleaner(1);
  // google::SetLogFilenameExtension("planning");
  // std::string log_prefix = "data/log/planning.log.";
  // google::SetLogDestination(FLAGS_minloglevel, log_prefix.c_str());
  // google::SetLogDestination(google::GLOG_WARNING, log_prefix.c_str());
  // google::SetLogDestination(google::GLOG_ERROR, log_prefix.c_str());
  // google::SetLogDestination(google::GLOG_FATAL, log_prefix.c_str());
  // LLOG_LEVEL(WARN);

  // std::filesystem::path ad_byd_dir = "./app/ad3/planning/";
  std::filesystem::path ad_byd_dir = "modules/cnoa_pnc/planning/";
  // std::string vehicle_model = "x01";
  int mode = kPlanningAndPrediction;
  /*
  for (int i = 1; i < argc; i += 2) {
    LOG_INFO << "argv[" << i << "] = " << argv[i];
    LOG_INFO << "argv[" << i + 1 << "] = " << argv[i + 1];
    std::string key = argv[i];
    if (key == "ad_byd_dir") {
      ad_byd_dir = argv[i + 1];
    } else if (key == "run_mode") {
      mode = std::stoi(argv[i + 1]);
    } else if (key == "vehicle_model") {
      vehicle_model = argv[i + 1];
    }
  }
*/
  LOG_INFO << "ad_byd_dir = " << ad_byd_dir << ", run_mode = " << mode;

  FLAGS_ad_byd_planning_cache_data =
      ad_byd_dir / FLAGS_ad_byd_planning_cache_data;
  FLAGS_ad_byd_planning_test_data =
      ad_byd_dir / FLAGS_ad_byd_planning_test_data;
  FLAGS_ad_byd_prediction_vehicle_reasoning_config =
      ad_byd_dir / FLAGS_ad_byd_prediction_vehicle_reasoning_config;
  FLAGS_ad_byd_prediction_pedestrian_reasoning_config =
      ad_byd_dir / FLAGS_ad_byd_prediction_pedestrian_reasoning_config;
  FLAGS_ad_byd_prediction_cyclist_reasoning_config =
      ad_byd_dir / FLAGS_ad_byd_prediction_cyclist_reasoning_config;
  FLAGS_ad_byd_prediction_config = ad_byd_dir / FLAGS_ad_byd_prediction_config;
  FLAGS_ad_byd_city_config = ad_byd_dir / FLAGS_ad_byd_city_config;

  FLAGS_ad_byd_vehicle_params_path =
      vehicle_config_file_path + FLAGS_ad_byd_vehicle_params_path;

  set_planning_enabled(mode == kPlanningAndPrediction || mode == kPlanningOnly);
  set_prediction_enabled(mode == kPlanningAndPrediction ||
                         mode == kPredictionOnly);

  LOG_INFO << "BydPlanningNode, mode: " << mode
           << ", planning enable: " << planning_enabled()
           << ", prediction enable: " << prediction_enabled();

  CHECK(Utility::GetProtoFromFile(FLAGS_ad_byd_prediction_config,
                                  &prediction_config_));
  CHECK(InitGflags());

  if (planning_enabled()) {
    city_planner_ = std::make_unique<st::planning::CityPlanner>(
        FLAGS_ad_byd_city_planner_pool_size);
    CHECK(city_planner_->Init(FLAGS_ad_byd_city_config));
  }

  return true;
}

bool BydPlanningCore::InitGflags() const {
  // todo: need to judge flags based on different platforms
  bool find_config = true;
  if (!find_config) {
    LOG_ERROR << "BydPlanningNode can not find platform config";
    return false;
  } else {
    AERROR << "BydPlanningNode platform: " << FLAGS_ad_byd_planning_platform
           << ", [num_threads] prediction: "
           << FLAGS_ad_byd_prediction_pool_size << ", [num_threads] planner: "
           << FLAGS_ad_byd_city_planner_pool_size;
    return true;
  }
}

bool BydPlanningCore::Exit() {
  // TODO: fix cuda runtime unload before destruction error
  // PluginManager::instance()->Exit();
  return true;
}

std::pair<std::shared_ptr<planning_result_type>,
          std::shared_ptr<debug_frame_type>>
BydPlanningCore::PlanningCallback(
    const std::pair<st::planning::PlannerStatusProto::PlannerStatusCode,
                    PlanningInputFrame::Ptr>& input_frame) {
  static uint32_t sequence = 0;
  sequence++;
  TIMELINE(absl::StrFormat("BydPlanningCore::PlanningCallback:%d", sequence));
  // byd::log::Warn("fsd-planning",
  //                 "fsd-planning callback, cntr: %" PRIu64 ", time: %f",
  //                 sequence, byd::utils::time::NowMilliseconds() / 1000.0);
  std::pair<std::shared_ptr<planning_result_type>,
            std::shared_ptr<debug_frame_type>>
      res{nullptr, nullptr};
#if defined(BYD_X2B) || defined(BYD_VCPB)
  auto top_state = input_frame.second ? input_frame.second->top_state : nullptr;

  if (!input_frame.second || !top_state) {
    city_planner_->Reset();
    LOGINFO_EVERY(5) << "Don't send topic cause no input_frame or top_state";
    st::planning::Log2DDS::Dump();
    return res;
  }

  const auto odd_type = top_state->odd_type();
  const auto state = top_state->state();

  if (!(odd_type == byd::msg::state_machine::TopState::HIGHWAY_MODE &&
        (state == byd::msg::state_machine::TopState::DEFAULT ||
         state == byd::msg::state_machine::TopState::ADAS))) {
    city_planner_->Reset();
    LOGINFO_EVERY(5) << "Don't send topic: odd_type = " << odd_type
                     << " state= " << state << " (need HIGHWAY + DEFATLT/ADAS)";
    st::planning::Log2DDS::Dump();
    return res;
  }
#endif
  if (input_frame.first == st::planning::PlannerStatusProto::OK ||
      (FLAGS_planner_enable_acc && IsInputMapError(input_frame.first) &&
       InputFrameValid(input_frame.second))) {
    if (FLAGS_planner_enable_acc && IsInputMapError(input_frame.first)) {
      LOGINFO_EVERY(5) << "Ignore map error if acc enabled, error code: "
                       << input_frame.first;
    }
  } else {
    auto fail_reason = input_frame.first;
    LOG_ERROR << "planning input frame invalid: " << fail_reason;

    res.first = std::make_shared<planning_result_type>();
    res.first->mutable_state()->set_result(st::planning::Result::RESULT_FAIL);
    res.first->mutable_state()->set_fail_reason(
        static_cast<byd::msg::planning::PlannerStatusCode>(fail_reason));

    st::planning::Log2DDS::LogDataV0("planner", -1);
    res.second = st::planning::Log2DDS::Dump();
    FillPlanningMsgHeader(res.first, res.second, sequence);
    FillPlanningAutoMode(res.first.get(), input_frame.second);
    city_planner_->Reset();
    // highway_planner_->Reset();
    return res;
  }

  Planner* planner{nullptr};
  const PlanningInputFrame* frame = std::get<1>(input_frame).get();
  bool next_is_city = true;
  bool func_id_valid = true;
  switch (frame->behavior->function_id()) {
    case byd::msg::planning::FunctionId::FUNCTION_HW_NOA:
    case byd::msg::planning::FunctionId::FUNCTION_LKA_PLUS:
    case byd::msg::planning::FunctionId::FUNCTION_LKA:
    case byd::msg::planning::FunctionId::FUNCTION_CITY_NOA:
    case byd::msg::planning::FunctionId::FUNCTION_MAPLESS_NOA:
    case byd::msg::planning::FunctionId::FUNCTION_ACC:
      planner = city_planner_.get();
      break;
    case byd::msg::planning::FunctionId::FUNCTION_APA:
    // case byd::msg::planning::FunctionId::FUNCTION_SUMMON:
    case byd::msg::planning::FunctionId::FUNCTION_NONE:
    default:
      func_id_valid = false;
      next_is_city = false;
      // planner = highway_planner_.get();
      planner = city_planner_.get();
      break;
  }
  if (is_city_ != next_is_city) {
    if (next_is_city) {
      LOG_WARN << "highway -> city";
      // highway_planner_->Reset();
    } else {
      LOG_WARN << "city -> highway";
      // city_planner_->Reset();
    }
    is_city_ = next_is_city;
  }
  if (frame->behavior->function_id() ==
      byd::msg::planning::FunctionId::FUNCTION_APA) {
    planner->Reset();
    return res;
  }

  // if (frame->behavior->function_id() ==
  //         byd::msg::planning::FunctionId::FUNCTION_ACC ||
  //     (prev_func_id_ != frame->behavior->function_id() &&
  //      prev_func_id_ == byd::msg::planning::FunctionId::FUNCTION_ACC)) {
  //   planner->ResetforAccEnterandExit();
  // }
  if ((prev_func_id_ != frame->behavior->function_id() &&
       prev_func_id_ == byd::msg::planning::FunctionId::FUNCTION_ACC)||
       (prev_func_id_ != frame->behavior->function_id() &&
       frame->behavior->function_id() == byd::msg::planning::FunctionId::FUNCTION_ACC)) {
    planner->ResetforAccEnterandExit();
  }
  prev_func_id_ = frame->behavior->function_id();

  planner->Run(frame);
  res.first = planner->GetPlanningResult();
  res.second = planner->GetDebugFrame();
  FillPlanningMsgHeader(res.first, res.second, sequence);

  if (!func_id_valid ||
      res.first->state().result() != st::planning::Result::RESULT_OK) {
    planner->Reset();
  }

  return res;
}
/*
std::shared_ptr<prediction_type> BydPlanningCore::PredictionCallback(
    const absl::StatusOr<PredictionInputFrame::Ptr>& input_frame) {
  st::Timer timer(__FUNCTION__);
  static size_t count = 0;
  bool input_ready = input_frame.ok();
  if (input_frame.ok()) {
    LOG_ERROR << "AD_BYD Prediction Input highway prediction "
               << static_cast<int>((*input_frame)->highway_prediction !=
                                   nullptr)
               << " behavior "
               << static_cast<int>((*input_frame)->behavior != nullptr);
    if ((*input_frame)->highway_prediction != nullptr) {
      LOG_ERROR << "AD_BYD highway prediction recieve obs size "
                 << (*input_frame)->highway_prediction->obstacles().size();
    }
  }
  if (input_ready && (*input_frame)->behavior != nullptr &&
      (*input_frame)->behavior->func_id() ==
          byd::msg::planning::FunctionId::FUNCTION_HW_NOA) {
    return (*input_frame)->highway_prediction;
  } else if (input_ready && (*input_frame)->behavior != nullptr &&
             ((*input_frame)->behavior->func_id() ==
                  byd::msg::planning::FunctionId::FUNCTION_LKA ||
              (*input_frame)->behavior->func_id() ==
                  byd::msg::planning::FunctionId::FUNCTION_LKA_PLUS) &&
             (*input_frame)->map_element.map_ptr != nullptr &&
             (*input_frame)->map_element.map_ptr->is_on_highway()) {
    // LKA or NOA on highway
    return (*input_frame)->highway_prediction;
  } else if (input_ready) {
    if (nullptr != (*input_frame)->behavior) {
      auto perception_state = (*input_frame)->behavior->perception_state();
      byd::log::Warn("BYD_PREDICTION", "perception_state: %d",
                      static_cast<int>(perception_state.underlying()));

      if (perception_state ==
              behavior_idls::idls::PerceptionState::type::none ||
          perception_state ==
              behavior_idls::idls::PerceptionState::type::parking ||
          perception_state ==
              behavior_idls::idls::PerceptionState::type::Pgear) {
        byd::log::Warn("BYD_PREDICTION",
                        "perception_state is one of [none, parking, Pgear]");
        if (0 == ++count % 4) {
          byd::log::Warn("BYD_PREDICTION", "prediction_frequency: 5Hz");
          // 每4帧pub一次消息，即降频到5Hz
          count = 0;
          component_->Run(*input_frame);
          auto prediction = std::make_shared<prediction_type>();
          component_->ConvertPredictionPubFrame(*input_frame, prediction.get());
          return prediction;
        } else {
          byd::log::Warn("BYD_PREDICTION", "skip prediction count: %lu",
                          count);
          return nullptr;
        }
      }
    }

    count = 0;
    component_->Run(*input_frame);
    auto prediction = std::make_shared<prediction_type>();
    component_->ConvertPredictionPubFrame(*input_frame, prediction.get());

    byd::log::Warn("BYD_PREDICTION",
                    "%s:%d: BYD-prediction end %" PRIu64
                    " receive header time %f local time %f, send header time "
                    "%f local time %f",
                    __FILE__, __LINE__, prediction->header().seq(),
                    (*input_frame)->head_ts, (*input_frame)->start_time,
                    prediction->header().stamp(),
                    byd::utils::time::NowMilliseconds() / 1000.0);

    return prediction;
  } else {
    LOG_ERROR << "prediction input frame invalid: "
               << input_frame.status().ToString();
    return nullptr;
  }
}
*/
}  // namespace planning
}  // namespace ad_byd
