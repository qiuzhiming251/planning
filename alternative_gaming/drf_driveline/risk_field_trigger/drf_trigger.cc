#include "drf_trigger.h"
#include "object_manager/planner_object_manager_builder.h"
namespace st::planning {
constexpr float kMiniStartAcc = 0.5;  // m/s*s
constexpr float kStartVel = 2.0;      // m/s
constexpr float kRightTurnSafeBuffer = 4.0;
// TODO(whj): unprotected left turn
bool RiskFieldTrigger::TriggerRiskField(
    const std::vector<RiskFieldKeyobj>& drf_keyobj,
    const ObjectHistoryManager& obs_history, const DiscretizedPath& init_path,
    const DrivelineResultProto* last_driveline_result,
    DrivelineResultProto* const cur_driveline_result,
    std::unordered_set<std::string>* yield_obj_ids) {
  bool yield_oppo_obj = false;
  for (const auto& keyobj : drf_keyobj) {
    auto obj = obs_history.GetObjLatestFrame(keyobj.id);
    if (obj != nullptr) {
      if (yield_obj_ids != nullptr &&
          obj->lon_decision == StBoundaryProto::YIELD) {
        yield_obj_ids->insert(keyobj.id);
      }
      yield_oppo_obj = true;
    } else if (CheckKeyObjLeave(keyobj, init_path)) {
      if (yield_obj_ids != nullptr) {
        yield_obj_ids->insert(keyobj.id);
      }
      yield_oppo_obj = true;
    }
  }
  int trigger_to_follow_car_count_ = 0;
  if (yield_oppo_obj) {
    cur_driveline_result->set_trigger_failed_counter(0);
    return true;
  } else {
    int failed_counter = last_driveline_result
                             ? last_driveline_result->trigger_failed_counter()
                             : 0;
    if (failed_counter < config_.rf_change_follow_car_mode_count_thresh &&
        last_driveline_result &&
        last_driveline_result->driveline_status() ==
            DrivelineStatus::GENERATE_SUCCEED) {
      cur_driveline_result->set_trigger_failed_counter(failed_counter + 1);
      return true;
    } else {
      cur_driveline_result->set_trigger_failed_counter(failed_counter);
      cur_driveline_result->set_driveline_status(
          DrivelineStatus::TRIGGER_FAILED);
      return false;
    }
  }
}

bool RiskFieldTrigger::TriggerLtRiskFieldDriveline(
    const std::vector<RiskFieldKeyobj>& drf_keyobj,
    const std::vector<RiskFieldKeyobj>& syn_left_turn_keycyclists,
    const ObjectHistoryManager& obs_history, const DiscretizedPath& init_path,
    const DrivelineResultProto* last_driveline_result,
    DrivelineResultProto* const cur_driveline_result,
    std::unordered_set<std::string>* yield_obj_ids) {
  bool yield_oppo_obj = false;
  for (const auto& keyobj : drf_keyobj) {
    auto obj = obs_history.GetObjLatestFrame(keyobj.id);
    if (!CheckLeftKeyobjBufferEnough(keyobj, init_path)) {
      yield_oppo_obj = true;
      if (yield_obj_ids != nullptr) {
        yield_obj_ids->insert(keyobj.id);
      }
    }
  }
  if (yield_oppo_obj) {
    cur_driveline_result->set_trigger_failed_counter(0);
    return true;
  } else {
    int failed_counter = last_driveline_result
                             ? last_driveline_result->trigger_failed_counter()
                             : 0;
    if (failed_counter < config_.rf_change_follow_car_mode_count_thresh &&
        last_driveline_result &&
        last_driveline_result->driveline_status() ==
            DrivelineStatus::GENERATE_SUCCEED) {
      cur_driveline_result->set_trigger_failed_counter(failed_counter + 1);
      return true;
    } else {
      cur_driveline_result->set_trigger_failed_counter(failed_counter);
      cur_driveline_result->set_driveline_status(
          DrivelineStatus::TRIGGER_FAILED);
      return false;
    }
  }
}

bool RiskFieldTrigger::CheckKeyObjLeave(
    const RiskFieldKeyobj& key_obj, const DiscretizedPath& init_path) const {
  if (key_obj.object_ptr == nullptr) {
    return false;
  }
  Vec2d pos(key_obj.object_ptr->planner_object().pose().pos().x(),
            key_obj.object_ptr->planner_object().pose().pos().y());
  auto sl = init_path.XYToSL(pos);
  return sl.l > 0.0;
}

bool RiskFieldTrigger::CheckLeftKeyobjBufferEnough(
    const RiskFieldKeyobj& key_obj, const DiscretizedPath& init_path) const {
  if (key_obj.object_ptr == nullptr) {
    return false;
  }
  Vec2d pos(key_obj.object_ptr->trajectory().points().front().pos().x(),
            key_obj.object_ptr->trajectory().points().front().pos().y());
  auto sl = init_path.XYToSL(pos);
  float test_buffer = 4.0;
  // before: kRightTurnSafeBuffer = 4.0
  // now: test_buffer
  // TODO: Use drive_passage coordinate
  if (sl.l > test_buffer) {
    return true;
  } else if (sl.l < test_buffer && sl.l > 0.0 && sl.s > -3.0) {
    return false;
  } else {
    return true;
  }
  return true;
}
}  // namespace st::planning