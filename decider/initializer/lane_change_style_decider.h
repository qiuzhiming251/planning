#ifndef ONBOARD_PLANNER_INITIALIZER_LANE_CHANGE_STYLE_DECIDER_H_
#define ONBOARD_PLANNER_INITIALIZER_LANE_CHANGE_STYLE_DECIDER_H_

#include <map>
#include <string>
#include <vector>

#include "plan_common/math/frenet_frame.h"
#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"

namespace st::planning {

// concerned scenes
enum class LaneChangeStylePostDeciderSceneType : int {
  SCENE_NONE = 0,
  SCENE_FRONT_OBJ_ON_TARGET_LANE,
  SCENE_REAR_OBJ_ON_TARGET_LANE,
  SCENE_REAR_OBJ_IN_OPPOSITE_DIR,
  SCENE_REAR_OBJ_IN_SAME_DIR,
  SCENE_FRONT_SLOW_OBJ_ON_EGO_LANE,
  SCENE_FRONT_STATIC_OBJ_ON_EGO_LANE,
};

// internal lane change style decisions
// the first one must be NONE, then: CONSERVATIVE -> RADICAL
enum class LaneChangeStyleDesicion : int {
  DECISION_NONE = 0,
  DECISION_CONSERVATIVE,
  DECISION_NORMAL,
  DECISION_RADICAL,
};

// Lane change safety check conditions
enum class LaneChangeSafetyCheckingCondition : int {
  CHECK_COND_LON_DS = 0,
  CHECK_COND_LAT_DL,  // only for the rear obj lane changing in same direction
  CHECK_COND_FOLLOW_DECEL,
  CHECK_COND_TTC,
  CHECK_COND_PRV_LON_DS,  // only for the obj lane changing in opposite
                          // direction
};

// Safety level
enum class LaneChangeSafetyLevel {
  LEVEL_NOT_SAFE_ENOUGH = 0,
  LEVEL_SAFE_ENOUGH,
  LEVEL_SAFE_EXTREMELY,
};

// object info
class LaneChangeStyleDeciderObjectInfo {
 public:
  LaneChangeStyleDeciderObjectInfo(const std::string id, const double lon_v,
                                   const double lat_v,
                                   const FrenetBox& frenet_box)
      : id_(std::move(id)),
        lon_v_(lon_v),
        lat_v_(lat_v),
        frenet_box_(frenet_box) {}
  LaneChangeStyleDeciderObjectInfo() = default;
  ~LaneChangeStyleDeciderObjectInfo() = default;

 public:
  std::string id_ = "";
  double lon_v_ = 0.0;
  double lat_v_ = 0.0;
  FrenetBox frenet_box_;
};

// safety checking info
class LaneChangeSafetyCheckingInfo {
 public:
  LaneChangeSafetyCheckingInfo(
      const LaneChangeSafetyCheckingCondition condition,
      const double actual_value, const double threshold)
      : condition_(condition),
        actual_value_(actual_value),
        threshold_(threshold) {}
  ~LaneChangeSafetyCheckingInfo() = default;

 public:
  LaneChangeSafetyCheckingCondition condition_;
  double actual_value_;
  double threshold_;
};

// scene info
class LaneChangeStylePostDeciderSceneInfo {
 public:
  LaneChangeStylePostDeciderSceneInfo() = default;
  ~LaneChangeStylePostDeciderSceneInfo() = default;

  LaneChangeSafetyLevel GetSafeLevel(
      std::vector<std::pair<LaneChangeSafetyCheckingCondition,
                            LaneChangeSafetyLevel>>* safety_details = nullptr,
      std::string* debug_str = nullptr) const;

  void set_obj_info(const LaneChangeStyleDeciderObjectInfo& obj_info,
                    const LaneChangeStyleDeciderObjectInfo& ego_info) {
    obj_info_ = obj_info;
    ego_info_ = ego_info;
  }

  void set_scene_type(const LaneChangeStylePostDeciderSceneType scene_type) {
    scene_type_ = scene_type;
  }

  void add_safety_checking_info(
      const LaneChangeSafetyCheckingInfo& safety_checking_info) {
    safety_checking_infos_.emplace_back(safety_checking_info);
    if (safety_checking_info.condition_ ==
        LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS) {
      has_lon_ds_ = true;
      lon_ds_ = safety_checking_info.actual_value_;
    }
  }

  void ClearSafetyCheckingInfo() {
    safety_checking_infos_.clear();
    has_lon_ds_ = false;
    lon_ds_ = 0.0;
  }

  const LaneChangeStyleDeciderObjectInfo& obj_info() const { return obj_info_; }

  const LaneChangeStylePostDeciderSceneType scene_type() const {
    return scene_type_;
  }

  const std::vector<LaneChangeSafetyCheckingInfo>& safety_checking_infos()
      const {
    return safety_checking_infos_;
  }

 private:
  LaneChangeStyleDeciderObjectInfo ego_info_;
  LaneChangeStyleDeciderObjectInfo obj_info_;
  LaneChangeStylePostDeciderSceneType scene_type_ =
      LaneChangeStylePostDeciderSceneType::SCENE_NONE;
  std::vector<LaneChangeSafetyCheckingInfo> safety_checking_infos_;

  bool has_lon_ds_ = false;
  double lon_ds_ = 0.0;
};

using LaneChangeStylePostDeciderSceneInfos =
    std::vector<LaneChangeStylePostDeciderSceneInfo>;

const std::string GetLaneChangeStyleName(const LaneChangeStyle style);

const std::string GetLaneChangeStyleName(
    const LaneChangeStyleDesicion decision);

const std::string GetLaneChangeStyleName(
    const SpeedResponseStyle response_style);

const std::string GetLaneChangeStyleName(
    const PathResponseStyle response_style);

const PathResponseStyle GetPathResponseStyleByLcStyle(
    const LaneChangeStyle style);

const SpeedResponseStyle GetSpeedResponseStyleByLcStyle(
    const LaneChangeStyle style);

const PathResponseStyle GetPathResponseStyleByLcStyleDecision(
    const LaneChangeStyleDesicion decision);

const SpeedResponseStyle GetSpeedResponseStyleByLcStyleDecision(
    const LaneChangeStyleDesicion decision);

const std::string GetLaneChangeSceneTypeName(
    const LaneChangeStylePostDeciderSceneType scene_type);

const std::string GetLaneChangeSafetyCheckingConditionName(
    const LaneChangeSafetyCheckingCondition condition);

void SetDefaultResponseStyle(const LaneChangeStyle lc_style,
                             LaneChangeStyleDeciderResultProto* style_result);

absl::Status RunLaneChangeStylePostDecider(
    const LaneChangeStyle lc_style,
    const LaneChangeStylePostDeciderSceneInfo* target_front_obj_scene_info_ptr,
    const LaneChangeStylePostDeciderSceneInfos& scene_infos,
    std::pair<PathResponseStyle, SpeedResponseStyle>* post_lc_style,
    std::string& debug_str);

}  // namespace st::planning

#endif