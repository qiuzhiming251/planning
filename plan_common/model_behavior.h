

#ifndef AD_BYD_PLANNING_COMMON_MODEL_BEHAVIOR_H
#define AD_BYD_PLANNING_COMMON_MODEL_BEHAVIOR_H
#include <memory>
#include <unordered_map>
#include <vector>

#include "planning_data.h"
#include "plan_common/maps/lane_sequence.h"
namespace ad_byd {
namespace planning {

struct LaneFormerBehavior {
  NavigableLaneSequencePtr top1_sequence = nullptr;
  int32_t keep_count = 0;
  NavigableLaneSequencePtr top2_sequence = nullptr;
};

struct DecisionFormerBehavior {
  std::unordered_map<std::string, DecisionLabel> obs_decision_map;
  double probability = 0.0;
};

class TaskModelBehavior {
 public:
  TaskModelBehavior() = default;
  ~TaskModelBehavior() = default;

  void SetLaneFormerBehavior(LaneFormerBehavior&& behavior) {
    has_lane_former_behavior_ = true;
    lane_former_behavior_ = std::move(behavior);
  }
  void SetDecisionFormerBahavior(DecisionFormerBehavior&& behavior) {
    has_decision_former_behavior_ = true;
    decision_former_behavior_ = std::move(behavior);
  }

  const LaneFormerBehavior& lane_former_behavior() const {
    return lane_former_behavior_;
  }
  const DecisionFormerBehavior& decision_former_behavior() const {
    return decision_former_behavior_;
  }

  bool HasLaneFormerBehavior() const { return has_lane_former_behavior_; }
  bool HasDecisionFormerBehavior() const {
    return has_decision_former_behavior_;
  }
  void SetLaneFormerFailReason(const std::string& fail_reason) {
    lane_former_fail_reason_ = fail_reason;
  }

  const std::string& GetLaneFormerFailReason() const {
    return lane_former_fail_reason_;
  }

 private:
  LaneFormerBehavior lane_former_behavior_;
  DecisionFormerBehavior decision_former_behavior_;

  std::string lane_former_fail_reason_ = "";
  bool has_lane_former_behavior_ = false;
  bool has_decision_former_behavior_ = false;
};

class ModelBehavior {
 public:
  ModelBehavior() = default;
  ~ModelBehavior() = default;

  void PushTaskModelBehavior(TaskModelBehavior&& task_model_behavior) {
    task_model_behaviors_.emplace_back(std::move(task_model_behavior));
  }

  const std::vector<TaskModelBehavior>& task_model_behaviors() const {
    return task_model_behaviors_;
  }

 private:
  std::vector<TaskModelBehavior> task_model_behaviors_;
};
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_COMMON_MODEL_BEHAVIOR_H
