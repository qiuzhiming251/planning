#include "decider/initializer/lane_change_style_decider.h"
#include "plan_common/log_data.h"

namespace st::planning {

constexpr double kEnoughSafeTimeBuffer = 3.0;      // s
constexpr double kEnoughSafePrvTimeBuffer = 0.5;   // s
constexpr double kEnoughSafeLonDsMinBuffer = 2.0;  // m
constexpr double kEnoughSafeLatDlBuffer = 0.5;     // m

constexpr double kExtremelySafeFactor = 3.0;

constexpr double kFollowerHeadTime = 0.5;  // s
constexpr double kFollowerDecelMax = 2.0;  // m/s2
constexpr double kLonSafeBuffer = 3.0;     // m

constexpr double kEpsilon = 1e-5;

/*******************************************
 * Scene info
 *******************************************/
// Check each safety checking is safe enough
// safety_details: checking_condition and safe_level
LaneChangeSafetyLevel LaneChangeStylePostDeciderSceneInfo::GetSafeLevel(
    std::vector<std::pair<LaneChangeSafetyCheckingCondition,
                          LaneChangeSafetyLevel>>* safety_details,
    std::string* debug_str) const {
  if (safety_details != nullptr) {
    safety_details->clear();
  }
  if (debug_str != nullptr) {
    debug_str->clear();
  }

  double lon_v_diff = obj_info_.lon_v_ - ego_info_.lon_v_;
  bool is_front_obj = false;
  if (scene_type_ ==
          LaneChangeStylePostDeciderSceneType::SCENE_FRONT_OBJ_ON_TARGET_LANE ||
      scene_type_ == LaneChangeStylePostDeciderSceneType::
                         SCENE_FRONT_SLOW_OBJ_ON_EGO_LANE ||
      scene_type_ == LaneChangeStylePostDeciderSceneType::
                         SCENE_FRONT_STATIC_OBJ_ON_EGO_LANE) {
    is_front_obj = true;
    lon_v_diff = -lon_v_diff;
  }

  bool is_safe_enough = true;
  bool is_safe_extremely = true;
  for (const auto& info : safety_checking_infos_) {
    double margin = 0.0;
    bool is_safe_enough_for_single_info = true;
    bool is_safe_extremely_for_single_info = true;
    switch (info.condition_) {
      case LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS: {
        margin = std::max(kEnoughSafeTimeBuffer * lon_v_diff,
                          kEnoughSafeLonDsMinBuffer);
        is_safe_enough_for_single_info =
            info.actual_value_ > (info.threshold_ + margin);
        is_safe_extremely_for_single_info =
            info.actual_value_ >
            (info.threshold_ + margin * kExtremelySafeFactor);
        break;
      }
      case LaneChangeSafetyCheckingCondition::CHECK_COND_PRV_LON_DS: {
        margin = std::max(kEnoughSafePrvTimeBuffer * lon_v_diff,
                          kEnoughSafeLonDsMinBuffer);
        is_safe_enough_for_single_info =
            info.actual_value_ > (info.threshold_ + margin);
        is_safe_extremely_for_single_info =
            info.actual_value_ >
            (info.threshold_ + margin * kExtremelySafeFactor);
        break;
      }
      case LaneChangeSafetyCheckingCondition::CHECK_COND_LAT_DL: {
        double abs_ego_l_min = std::min(std::fabs(ego_info_.frenet_box_.l_max),
                                        std::fabs(ego_info_.frenet_box_.l_min));
        double abs_obj_l_min = std::min(std::fabs(obj_info_.frenet_box_.l_max),
                                        std::fabs(obj_info_.frenet_box_.l_min));
        margin = kEnoughSafeLatDlBuffer;
        is_safe_enough_for_single_info =
            abs_ego_l_min < (abs_obj_l_min - margin);
        is_safe_extremely_for_single_info =
            abs_ego_l_min < (abs_obj_l_min - margin * kExtremelySafeFactor);
        break;
      }
      case LaneChangeSafetyCheckingCondition::CHECK_COND_FOLLOW_DECEL: {
        double abs_ds =
            has_lon_ds_
                ? lon_ds_
                : std::fabs(is_front_obj ? obj_info_.frenet_box_.s_min -
                                               ego_info_.frenet_box_.s_max
                                         : ego_info_.frenet_box_.s_min -
                                               obj_info_.frenet_box_.s_max);
        double dist =
            abs_ds -
            lon_v_diff * (kFollowerHeadTime -
                          (is_front_obj ? ego_info_.lon_v_ : obj_info_.lon_v_) /
                              kFollowerDecelMax) -
            kLonSafeBuffer;
        margin = std::clamp(
            kEnoughSafeTimeBuffer * lon_v_diff / std::max(dist, kEpsilon), 0.0,
            1.0);
        is_safe_enough_for_single_info =
            info.actual_value_ < info.threshold_ * (1.0 - margin);
        is_safe_extremely_for_single_info =
            info.actual_value_ <
            info.threshold_ *
                (1.0 - std::clamp(margin * kExtremelySafeFactor, 0.0, 1.0));
        break;
      }
      case LaneChangeSafetyCheckingCondition::CHECK_COND_TTC: {
        margin = kEnoughSafeTimeBuffer;
        is_safe_enough_for_single_info =
            info.actual_value_ > (info.threshold_ + margin);
        is_safe_extremely_for_single_info =
            info.actual_value_ >
            (info.threshold_ + margin * kExtremelySafeFactor);
        break;
      }
      default:
        break;
    }
    is_safe_enough = is_safe_enough && is_safe_enough_for_single_info;
    is_safe_extremely = is_safe_extremely && is_safe_extremely_for_single_info;
    LaneChangeSafetyLevel safe_level_for_single_info =
        is_safe_extremely_for_single_info
            ? LaneChangeSafetyLevel::LEVEL_SAFE_EXTREMELY
            : (is_safe_enough_for_single_info
                   ? LaneChangeSafetyLevel::LEVEL_SAFE_ENOUGH
                   : LaneChangeSafetyLevel::LEVEL_NOT_SAFE_ENOUGH);
    if (safety_details != nullptr) {
      safety_details->emplace_back(
          std::make_pair(info.condition_, safe_level_for_single_info));
    }
    if (debug_str != nullptr) {
      *debug_str =
          *debug_str +
          absl::StrFormat(
              "%s(%d):%.2f,%.2f,%.2f ",
              GetLaneChangeSafetyCheckingConditionName(info.condition_),
              (int)safe_level_for_single_info, info.actual_value_,
              info.threshold_, margin);
    }
  }

  LaneChangeSafetyLevel safe_level =
      is_safe_extremely
          ? LaneChangeSafetyLevel::LEVEL_SAFE_EXTREMELY
          : (is_safe_enough ? LaneChangeSafetyLevel::LEVEL_SAFE_ENOUGH
                            : LaneChangeSafetyLevel::LEVEL_NOT_SAFE_ENOUGH);

  if (debug_str != nullptr) {
    *debug_str = absl::StrFormat("safe_lvl:%d ", (int)safe_level) + *debug_str;
  }
  return safe_level;
}

/*******************************************
 * Make style decision for each scene
 *******************************************/
bool MakeDecisionForEachScene(
    const LaneChangeStylePostDeciderSceneInfo& scene_info,
    const bool has_target_front_obj, const bool is_target_front_obj_safe_enough,
    LaneChangeStyleDesicion* lat_style, LaneChangeStyleDesicion* lon_style,
    bool* is_lat_forced, bool* is_lon_forced,
    std::string& debug_str_for_each_scene) {
  debug_str_for_each_scene = "";
  // Check the safe level for the object
  std::vector<
      std::pair<LaneChangeSafetyCheckingCondition, LaneChangeSafetyLevel>>
      safety_details;
  LaneChangeSafetyLevel safe_level =
      scene_info.GetSafeLevel(&safety_details, &debug_str_for_each_scene);

  // Match the rules for each scene
  switch (scene_info.scene_type()) {
    case LaneChangeStylePostDeciderSceneType::SCENE_REAR_OBJ_ON_TARGET_LANE: {
      if (safe_level == LaneChangeSafetyLevel::LEVEL_SAFE_ENOUGH ||
          safe_level == LaneChangeSafetyLevel::LEVEL_SAFE_EXTREMELY) {
        *lat_style = LaneChangeStyleDesicion::DECISION_NORMAL;
        *lon_style = LaneChangeStyleDesicion::DECISION_NORMAL;
        *is_lat_forced = false;
        *is_lon_forced = false;
        break;
      }
      if (has_target_front_obj) {
        *lat_style = LaneChangeStyleDesicion::DECISION_RADICAL;
        if (is_target_front_obj_safe_enough) {
          *lon_style = LaneChangeStyleDesicion::DECISION_RADICAL;
        } else {
          *lon_style = LaneChangeStyleDesicion::DECISION_NORMAL;
        }
      } else {
        *lat_style = LaneChangeStyleDesicion::DECISION_NORMAL;
        *lon_style = LaneChangeStyleDesicion::DECISION_RADICAL;
      }
      *is_lat_forced = false;
      *is_lon_forced = false;
      break;
    }
    case LaneChangeStylePostDeciderSceneType::SCENE_REAR_OBJ_IN_OPPOSITE_DIR: {
      if (safe_level == LaneChangeSafetyLevel::LEVEL_SAFE_EXTREMELY) {
        *lat_style = LaneChangeStyleDesicion::DECISION_NORMAL;
        *lon_style = LaneChangeStyleDesicion::DECISION_NORMAL;
        *is_lat_forced = false;
        *is_lon_forced = false;
        break;
      }
      *lat_style = LaneChangeStyleDesicion::DECISION_NORMAL;
      if (!has_target_front_obj || is_target_front_obj_safe_enough) {
        *lon_style = LaneChangeStyleDesicion::DECISION_RADICAL;
      } else {
        *lon_style = LaneChangeStyleDesicion::DECISION_NORMAL;
      }
      // lateral style must be forced to NORMAL if it is not extremely safe
      *is_lat_forced = true;
      *is_lon_forced = false;
      break;
    }
    case LaneChangeStylePostDeciderSceneType::SCENE_REAR_OBJ_IN_SAME_DIR: {
      LaneChangeSafetyLevel lon_safe_level =
          LaneChangeSafetyLevel::LEVEL_SAFE_EXTREMELY;
      LaneChangeSafetyLevel lat_safe_level =
          LaneChangeSafetyLevel::LEVEL_SAFE_EXTREMELY;
      for (const auto& result : safety_details) {
        if (result.first ==
            LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS) {
          lon_safe_level = (LaneChangeSafetyLevel)std::min((int)lon_safe_level,
                                                           (int)result.second);
        } else if (result.first ==
                   LaneChangeSafetyCheckingCondition::CHECK_COND_TTC) {
          lon_safe_level = (LaneChangeSafetyLevel)std::min((int)lon_safe_level,
                                                           (int)result.second);
        } else if (result.first ==
                   LaneChangeSafetyCheckingCondition::CHECK_COND_LAT_DL) {
          lat_safe_level = (LaneChangeSafetyLevel)std::min((int)lat_safe_level,
                                                           (int)result.second);
        }
      }
      *lat_style =
          (lon_safe_level != LaneChangeSafetyLevel::LEVEL_SAFE_EXTREMELY &&
           lat_safe_level == LaneChangeSafetyLevel::LEVEL_NOT_SAFE_ENOUGH)
              ? LaneChangeStyleDesicion::DECISION_RADICAL
              : LaneChangeStyleDesicion::DECISION_NORMAL;
      if (!has_target_front_obj) {
        *lon_style =
            lon_safe_level == LaneChangeSafetyLevel::LEVEL_SAFE_EXTREMELY
                ? LaneChangeStyleDesicion::DECISION_NORMAL
                : LaneChangeStyleDesicion::DECISION_RADICAL;
      } else if (is_target_front_obj_safe_enough) {
        *lon_style =
            lon_safe_level == LaneChangeSafetyLevel::LEVEL_NOT_SAFE_ENOUGH
                ? LaneChangeStyleDesicion::DECISION_RADICAL
                : LaneChangeStyleDesicion::DECISION_NORMAL;
      } else {
        *lon_style = LaneChangeStyleDesicion::DECISION_NORMAL;
      }
      *is_lat_forced = false;
      *is_lon_forced = false;
      break;
    }
    case LaneChangeStylePostDeciderSceneType::
        SCENE_FRONT_SLOW_OBJ_ON_EGO_LANE: {
      *lat_style = safe_level == LaneChangeSafetyLevel::LEVEL_NOT_SAFE_ENOUGH
                       ? LaneChangeStyleDesicion::DECISION_RADICAL
                       : LaneChangeStyleDesicion::DECISION_NORMAL;
      *lon_style = LaneChangeStyleDesicion::DECISION_NORMAL;
      *is_lat_forced = false;
      // longitudinal style must be forced to NORMAL if it is not extremely safe
      *is_lon_forced =
          (safe_level != LaneChangeSafetyLevel::LEVEL_SAFE_EXTREMELY);
      break;
    }
    case LaneChangeStylePostDeciderSceneType::
        SCENE_FRONT_STATIC_OBJ_ON_EGO_LANE: {
      *lat_style = safe_level == LaneChangeSafetyLevel::LEVEL_NOT_SAFE_ENOUGH
                       ? LaneChangeStyleDesicion::DECISION_RADICAL
                       : LaneChangeStyleDesicion::DECISION_NORMAL;
      *lon_style = LaneChangeStyleDesicion::DECISION_NORMAL;
      *is_lat_forced = false;
      // longitudinal style must be forced to NORMAL if it is not extremely safe
      *is_lon_forced =
          (safe_level != LaneChangeSafetyLevel::LEVEL_SAFE_EXTREMELY);
      break;
    }
    default: {
      *lat_style = LaneChangeStyleDesicion::DECISION_NORMAL;
      *lon_style = LaneChangeStyleDesicion::DECISION_NORMAL;
      *is_lat_forced = false;
      *is_lon_forced = false;
      break;
    }
  }

  return true;
}

const std::string GetLaneChangeStyleName(const LaneChangeStyle style) {
  switch (style) {
    case LaneChangeStyle::LC_STYLE_NORMAL: {
      return "normal";
    }
    case LaneChangeStyle::LC_STYLE_RADICAL: {
      return "radical";
    }
    case LaneChangeStyle::LC_STYLE_CONSERVATIVE: {
      return "conservative";
    }
    default: {
      return "Unknown";
    }
  }
  return "unknown";
}

const std::string GetLaneChangeStyleName(
    const LaneChangeStyleDesicion decision) {
  switch (decision) {
    case LaneChangeStyleDesicion::DECISION_NORMAL: {
      return "normal";
    }
    case LaneChangeStyleDesicion::DECISION_RADICAL: {
      return "radical";
    }
    case LaneChangeStyleDesicion::DECISION_CONSERVATIVE: {
      return "conservative";
    }
    default: {
      return "unknown";
    }
  }
  return "unknown";
}

const std::string GetLaneChangeStyleName(
    const SpeedResponseStyle response_style) {
  switch (response_style) {
    case SpeedResponseStyle::SPEED_RESPONSE_NORMAL: {
      return "normal";
    }
    case SpeedResponseStyle::SPEED_RESPONSE_RADICAL: {
      return "radical";
    }
    case SpeedResponseStyle::SPEED_RESPONSE_CONSERVATIVE: {
      return "conservative";
    }
    case SpeedResponseStyle::SPEED_RESPONSE_FAST: {
      return "fast";
    }
    default: {
      return "unknown";
    }
  }
  return "unknown";
}

const std::string GetLaneChangeStyleName(
    const PathResponseStyle response_style) {
  switch (response_style) {
    case PathResponseStyle::PATH_RESPONSE_NORMAL: {
      return "normal";
    }
    case PathResponseStyle::PATH_RESPONSE_RADICAL: {
      return "radical";
    }
    case PathResponseStyle::PATH_RESPONSE_CONSERVATIVE: {
      return "conservative";
    }
    case PathResponseStyle::PATH_RESPONSE_FAST: {
      return "fast";
    }
    default: {
      return "unknown";
    }
  }
  return "unknown";
}

const PathResponseStyle GetPathResponseStyleByLcStyle(
    const LaneChangeStyle style) {
  switch (style) {
    case LaneChangeStyle::LC_STYLE_NORMAL: {
      return PathResponseStyle::PATH_RESPONSE_NORMAL;
    }
    case LaneChangeStyle::LC_STYLE_RADICAL: {
      return PathResponseStyle::PATH_RESPONSE_RADICAL;
    }
    case LaneChangeStyle::LC_STYLE_CONSERVATIVE: {
      return PathResponseStyle::PATH_RESPONSE_CONSERVATIVE;
    }
    default: {
      return PathResponseStyle::PATH_RESPONSE_NORMAL;
    }
  }
  return PathResponseStyle::PATH_RESPONSE_NORMAL;
}

const SpeedResponseStyle GetSpeedResponseStyleByLcStyle(
    const LaneChangeStyle style) {
  switch (style) {
    case LaneChangeStyle::LC_STYLE_NORMAL: {
      return SpeedResponseStyle::SPEED_RESPONSE_NORMAL;
    }
    case LaneChangeStyle::LC_STYLE_RADICAL: {
      return SpeedResponseStyle::SPEED_RESPONSE_RADICAL;
    }
    case LaneChangeStyle::LC_STYLE_CONSERVATIVE: {
      return SpeedResponseStyle::SPEED_RESPONSE_CONSERVATIVE;
    }
    default: {
      return SpeedResponseStyle::SPEED_RESPONSE_NORMAL;
    }
  }
  return SpeedResponseStyle::SPEED_RESPONSE_NORMAL;
}

const PathResponseStyle GetPathResponseStyleByLcStyleDecision(
    const LaneChangeStyleDesicion decision) {
  switch (decision) {
    case LaneChangeStyleDesicion::DECISION_NORMAL: {
      return PathResponseStyle::PATH_RESPONSE_NORMAL;
    }
    case LaneChangeStyleDesicion::DECISION_RADICAL: {
      return PathResponseStyle::PATH_RESPONSE_RADICAL;
    }
    case LaneChangeStyleDesicion::DECISION_CONSERVATIVE: {
      return PathResponseStyle::PATH_RESPONSE_CONSERVATIVE;
    }
    default: {
      return PathResponseStyle::PATH_RESPONSE_NORMAL;
    }
  }
  return PathResponseStyle::PATH_RESPONSE_NORMAL;
}

const SpeedResponseStyle GetSpeedResponseStyleByLcStyleDecision(
    const LaneChangeStyleDesicion decision) {
  switch (decision) {
    case LaneChangeStyleDesicion::DECISION_NORMAL: {
      return SpeedResponseStyle::SPEED_RESPONSE_NORMAL;
    }
    case LaneChangeStyleDesicion::DECISION_RADICAL: {
      return SpeedResponseStyle::SPEED_RESPONSE_RADICAL;
    }
    case LaneChangeStyleDesicion::DECISION_CONSERVATIVE: {
      return SpeedResponseStyle::SPEED_RESPONSE_CONSERVATIVE;
    }
    default: {
      return SpeedResponseStyle::SPEED_RESPONSE_NORMAL;
    }
  }
  return SpeedResponseStyle::SPEED_RESPONSE_NORMAL;
}

const std::string GetLaneChangeSceneTypeName(
    const LaneChangeStylePostDeciderSceneType scene_type) {
  switch (scene_type) {
    case LaneChangeStylePostDeciderSceneType::SCENE_FRONT_OBJ_ON_TARGET_LANE: {
      return "TargetFrontObj";
    }
    case LaneChangeStylePostDeciderSceneType::SCENE_REAR_OBJ_ON_TARGET_LANE: {
      return "TargetRearObj";
    }
    case LaneChangeStylePostDeciderSceneType::SCENE_REAR_OBJ_IN_OPPOSITE_DIR: {
      return "OppoDirRearObj";
    }
    case LaneChangeStylePostDeciderSceneType::SCENE_REAR_OBJ_IN_SAME_DIR: {
      return "SameDirRearObj";
    }
    case LaneChangeStylePostDeciderSceneType::
        SCENE_FRONT_SLOW_OBJ_ON_EGO_LANE: {
      return "EgoSlowFrontObj";
    }
    case LaneChangeStylePostDeciderSceneType::
        SCENE_FRONT_STATIC_OBJ_ON_EGO_LANE: {
      return "EgoStaticFrontObj";
    }
    default: {
      return "Unknown";
    }
  }
  return "Unknown";
}

const std::string GetLaneChangeSafetyCheckingConditionName(
    const LaneChangeSafetyCheckingCondition condition) {
  switch (condition) {
    case LaneChangeSafetyCheckingCondition::CHECK_COND_LON_DS: {
      return "lon_ds";
    }
    case LaneChangeSafetyCheckingCondition::CHECK_COND_LAT_DL: {
      return "lat_dl";
    }
    case LaneChangeSafetyCheckingCondition::CHECK_COND_PRV_LON_DS: {
      return "prv_lon_ds";
    }
    case LaneChangeSafetyCheckingCondition::CHECK_COND_FOLLOW_DECEL: {
      return "decel";
    }
    case LaneChangeSafetyCheckingCondition::CHECK_COND_TTC: {
      return "ttc";
    }
    default: {
      return "Unknown";
    }
  }
  return "Unknown";
}

void SetDefaultResponseStyle(const LaneChangeStyle lc_style,
                             LaneChangeStyleDeciderResultProto* style_result) {
  if (style_result == nullptr) {
    return;
  }
  PathResponseStyle default_path_response_style =
      GetPathResponseStyleByLcStyle(lc_style);
  SpeedResponseStyle default_speed_response_style =
      GetSpeedResponseStyleByLcStyle(lc_style);
  style_result->set_active_path_response_style(default_path_response_style);
  style_result->set_prepared_path_response_style(default_path_response_style);
  style_result->set_prepared_path_response_style_counter(0);
  style_result->set_active_speed_response_style(default_speed_response_style);
  style_result->set_prepared_speed_response_style(default_speed_response_style);
  style_result->set_prepared_speed_response_style_counter(0);
  style_result->set_active_path_response_style(default_path_response_style);
  style_result->set_prepared_path_response_style(default_path_response_style);
  style_result->set_prepared_path_response_style_counter(0);
  style_result->set_active_speed_response_style(default_speed_response_style);
  style_result->set_prepared_speed_response_style(default_speed_response_style);
  style_result->set_prepared_speed_response_style_counter(0);
  style_result->set_congestion_scene(false);
}

absl::Status RunLaneChangeStylePostDecider(
    const LaneChangeStyle lc_style,
    const LaneChangeStylePostDeciderSceneInfo* target_front_obj_scene_info_ptr,
    const LaneChangeStylePostDeciderSceneInfos& scene_infos,
    std::pair<PathResponseStyle, SpeedResponseStyle>* post_lc_style,
    std::string& debug_str) {
  CHECK_NOTNULL(post_lc_style);

  PathResponseStyle& lat_style = post_lc_style->first;
  SpeedResponseStyle& lon_style = post_lc_style->second;

  // Both lateral and longitudinal styles are radical if the lc_style is radical
  if (lc_style == LC_STYLE_RADICAL) {
    lat_style = PathResponseStyle::PATH_RESPONSE_RADICAL;
    lon_style = SpeedResponseStyle::SPEED_RESPONSE_RADICAL;
    debug_str = "set radical directly due to the original style";
    return absl::OkStatus();
  }

  std::string debug_str_temp = "";

  // Check if it is safe enough for the front obj on the target lane
  bool has_target_front_obj =
      target_front_obj_scene_info_ptr != nullptr &&
      target_front_obj_scene_info_ptr->scene_type() ==
          LaneChangeStylePostDeciderSceneType::SCENE_FRONT_OBJ_ON_TARGET_LANE;
  bool is_target_front_obj_safe_enough = true;
  if (has_target_front_obj) {
    std::string target_front_obj_debug_str = "";
    LaneChangeSafetyLevel target_front_obj_safe_level =
        target_front_obj_scene_info_ptr->GetSafeLevel(
            nullptr, &target_front_obj_debug_str);
    is_target_front_obj_safe_enough =
        (target_front_obj_safe_level !=
         LaneChangeSafetyLevel::LEVEL_NOT_SAFE_ENOUGH);
    debug_str_temp +=
        (absl::StrFormat("#%s id:%s ",
                         GetLaneChangeSceneTypeName(
                             target_front_obj_scene_info_ptr->scene_type()),
                         target_front_obj_scene_info_ptr->obj_info().id_) +
         target_front_obj_debug_str);
  } else {
    debug_str_temp += "#NoTargerFrontObj ";
  }

  // Make decisions For each scene
  std::vector<
      std::tuple<LaneChangeStylePostDeciderSceneType, LaneChangeStyleDesicion,
                 LaneChangeStyleDesicion, bool, bool>>
      lc_style_decisions;
  for (const auto& scene_info : scene_infos) {
    auto& decision = lc_style_decisions.emplace_back();
    std::get<0>(decision) = scene_info.scene_type();
    std::string debug_str_for_each_scene = "";
    if (!MakeDecisionForEachScene(
            scene_info, has_target_front_obj, is_target_front_obj_safe_enough,
            &std::get<1>(decision), &std::get<2>(decision),
            &std::get<3>(decision), &std::get<4>(decision),
            debug_str_for_each_scene)) {
      std::get<0>(decision) = LaneChangeStylePostDeciderSceneType::SCENE_NONE;
    }

    debug_str_temp +=
        (absl::StrFormat("#%s style:%s[%d],%s[%d] id:%s ",
                         GetLaneChangeSceneTypeName(scene_info.scene_type()),
                         GetLaneChangeStyleName(std::get<1>(decision)),
                         (int)std::get<3>(decision),
                         GetLaneChangeStyleName(std::get<2>(decision)),
                         (int)std::get<4>(decision),
                         scene_info.obj_info().id_) +
         debug_str_for_each_scene);
  }

  // Make final internal decision
  bool is_lat_forced = false, is_lon_forced = false;
  LaneChangeStyleDesicion lat_decision = LaneChangeStyleDesicion::DECISION_NONE;
  LaneChangeStyleDesicion lon_decision = LaneChangeStyleDesicion::DECISION_NONE;
  for (const auto& decision : lc_style_decisions) {
    if (std::get<0>(decision) ==
        LaneChangeStylePostDeciderSceneType::SCENE_NONE) {
      continue;
    }
    if (std::get<3>(decision)) {
      if (is_lat_forced) {
        lat_decision = (LaneChangeStyleDesicion)std::max(
            (int)lat_decision, (int)std::get<1>(decision));
      } else {
        lat_decision = std::get<1>(decision);
        is_lat_forced = true;
      }
    } else {
      if (!is_lat_forced) {
        lat_decision = (LaneChangeStyleDesicion)std::max(
            (int)lat_decision, (int)std::get<1>(decision));
      }
    }
    if (std::get<4>(decision)) {
      if (is_lon_forced) {
        lon_decision = (LaneChangeStyleDesicion)std::max(
            (int)lon_decision, (int)std::get<2>(decision));
      } else {
        lon_decision = std::get<2>(decision);
        is_lon_forced = true;
      }
    } else {
      if (!is_lon_forced) {
        lon_decision = (LaneChangeStyleDesicion)std::max(
            (int)lon_decision, (int)std::get<2>(decision));
      }
    }
  }
  // if no scene decision, make lat and lon decision normal
  if (lat_decision == LaneChangeStyleDesicion::DECISION_NONE) {
    lat_decision = LaneChangeStyleDesicion::DECISION_NORMAL;
  }
  if (lon_decision == LaneChangeStyleDesicion::DECISION_NONE) {
    lon_decision = LaneChangeStyleDesicion::DECISION_NORMAL;
  }

  // Modified at 20250925: if either lat or lon decision is radical, make both
  // radical
  if (lat_decision == LaneChangeStyleDesicion::DECISION_RADICAL ||
      lon_decision == LaneChangeStyleDesicion::DECISION_RADICAL) {
    lat_decision = LaneChangeStyleDesicion::DECISION_RADICAL;
    lon_decision = LaneChangeStyleDesicion::DECISION_RADICAL;
  }

  // Make final decision according to current lc_style
  if (lc_style == LC_STYLE_CONSERVATIVE) {
    // lat_decision = (LaneChangeStyleDesicion)std::max(
    //     (int)LaneChangeStyleDesicion::DECISION_CONSERVATIVE,
    //     (int)lat_decision - 1);
    // lon_decision = (LaneChangeStyleDesicion)std::max(
    //     (int)LaneChangeStyleDesicion::DECISION_CONSERVATIVE,
    //     (int)lon_decision - 1);
    if (lat_decision != LaneChangeStyleDesicion::DECISION_RADICAL) {
      lat_decision = (LaneChangeStyleDesicion)std::max(
          (int)LaneChangeStyleDesicion::DECISION_CONSERVATIVE,
          (int)lat_decision - 1);
    }
    if (lon_decision != LaneChangeStyleDesicion::DECISION_RADICAL) {
      lon_decision = (LaneChangeStyleDesicion)std::max(
          (int)LaneChangeStyleDesicion::DECISION_CONSERVATIVE,
          (int)lon_decision - 1);
    }
  } else if (lc_style == LC_STYLE_RADICAL) {
    lat_decision = (LaneChangeStyleDesicion)std::min(
        (int)LaneChangeStyleDesicion::DECISION_RADICAL, (int)lat_decision + 1);
    lon_decision = (LaneChangeStyleDesicion)std::min(
        (int)LaneChangeStyleDesicion::DECISION_RADICAL, (int)lon_decision + 1);
  }

  // Match LaneChangeStyleDesicion to PathResponseStyle and SpeedResponseStyle
  lat_style = GetPathResponseStyleByLcStyleDecision(lat_decision);
  lon_style = GetSpeedResponseStyleByLcStyleDecision(lon_decision);

  debug_str = absl::StrFormat("style:%s,%s | inner_style:%s,%s | ",
                              GetLaneChangeStyleName(lat_style),
                              GetLaneChangeStyleName(lon_style),
                              GetLaneChangeStyleName(lat_decision),
                              GetLaneChangeStyleName(lon_decision)) +
              debug_str_temp;

  return absl::OkStatus();
}

}  // namespace st::planning