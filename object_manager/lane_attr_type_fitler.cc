#include "lane_attr_type_filter.h"

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/util.h"
#include "plan_common/log_data.h"
#include "predictor/prediction_util.h"
#include "plan_common/util/status_macros.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"

namespace st {
namespace planning {

FilterReason::Type LaneAttrTypeFilter::Filter(
    const PlannerObject& object,
    const prediction::PredictedTrajectory& traj) const {
  if (prediction::IsStationaryTrajectory(traj) &&
      (object.type() == st::ObjectType::OT_BARRIER ||
       object.type() == st::ObjectType::OT_CONE ||
       object.type() == st::ObjectType::OT_WARNING_TRIANGLE)) {
    std::string object_debug =
        " obj_id: " + object.id() +
        ", obj_type: " + ObjectType_Name(object.type()) + ", fusion_source: " +
        FusionSource_Name(object.object_proto().fusion_source());

    // const auto frenet_box =
    //     drive_passage_->QueryFrenetBoxAt(object.bounding_box());
    // const auto frenet_ego_box = drive_passage_->QueryFrenetBoxAt(ego_box_);
    // if (!frenet_box.ok() || !frenet_ego_box.ok()) {
    //   object_debug =
    //       object_debug +
    //       ", Not Filter because failed to get frenet box information";
    //   Log2DDS::LogDataV2("attr_type_filter", object_debug);
    //   return FilterReason::NONE;
    // }
    const auto& contour = object.contour();
    const Vec2d obj_center = contour.CircleCenter();
    const Vec2d av_to_obj = obj_center - back_pos_;
    const double dist_to_ego = tangent_.Dot(av_to_obj);
    // const double dist_to_ego =
    //     frenet_box.value().s_min - frenet_ego_box.value().s_max;

    auto lane_attr_type =
        object.object_proto().vision_attribute().lane_attr_type();
    auto lane_attr_conf =
        object.object_proto().vision_attribute().lane_attr_conf();
    // for not on highway
    if (!is_on_highway_) {
      constexpr double kNeardist = 50.0;
      constexpr double kFardist = 80.0;
      constexpr double kAttrConfThr = 0.5;

      if (object.object_proto().fusion_source() ==
              st::FusionSource::VISION_FUSION ||
          object.object_proto().fusion_source() ==
              st::FusionSource::RADAR_VISION_FUSION) {
        object_debug =
            absl::StrCat(object_debug, ", dist_to_ego: ", dist_to_ego);
        if (dist_to_ego > kNeardist && dist_to_ego <= kFardist) {
          if (lane_attr_type == LaneAttrType::LANEATTR_OTHER &&
              lane_attr_conf > kAttrConfThr) {
            object_debug =
                absl::StrCat(object_debug, ", lane_attr_type: ", lane_attr_type,
                             ", lane_attr_conf: ", lane_attr_conf);
            Log2DDS::LogDataV2("attr_type_filter",
                               object_debug + " Filtered when LANEATTR_OTHER");
            return FilterReason::STATIONARY_OBJECT_NOT_ON_TARGET_LANE;
          }
        } else if (dist_to_ego > kFardist) {
          Log2DDS::LogDataV2("attr_type_filter",
                             object_debug + " Filtered when fardist");
          return FilterReason::STATIONARY_OBJECT_NOT_ON_TARGET_LANE;
        }
      }
      // Log2DDS::LogDataV2("attr_type_filter", object_debug + "Not
      // Filtered");
      // for on highway
    } else {
      constexpr double kNeardist = 80.0;
      constexpr double kFardist = 120.0;
      if (dist_to_ego > kFardist) {
        if ((lane_attr_type == LaneAttrType::LANEATTR_UNKNOWN ||
             lane_attr_type == LaneAttrType::LANEATTR_OTHER ||
             lane_attr_type == LaneAttrType::LANEATTR_ON_LINE) &&
            lane_attr_conf > 0.5) {
          object_debug =
              absl::StrCat(object_debug, ", dist_to_ego: ", dist_to_ego);
          Log2DDS::LogDataV2("attr_type_filter",
                             object_debug + " Filtered when fardist");
          return FilterReason::STATIONARY_OBJECT_NOT_ON_TARGET_LANE;
        }
      } else if (dist_to_ego > kNeardist) {
        if (lane_attr_type == LaneAttrType::LANEATTR_OTHER &&
            lane_attr_conf > 0.5) {
          object_debug =
              absl::StrCat(object_debug, ", lane_attr_type: ", lane_attr_type,
                           ", lane_attr_conf: ", lane_attr_conf);
          Log2DDS::LogDataV2("attr_type_filter",
                             object_debug + " Filtered when LANEATTR_OTHER");
          return FilterReason::STATIONARY_OBJECT_NOT_ON_TARGET_LANE;
        }
      }
      // if (lc_state_.stage() == LaneChangeStage::LCS_NONE) {  // ||
      //   //   lc_state_->stage() == LaneChangeStage::LCS_PAUSE ||
      //   //   lc_state_->stage() == LaneChangeStage::LCS_RETURN) {
      //   if ((lane_attr_type == LaneAttrType::LANEATTR_LEFT ||
      //        lane_attr_type == LaneAttrType::LANEATTR_RIGHT) &&
      //       (lane_attr_conf > 0.5)) {
      //     object_debug =
      //         absl::StrCat(object_debug, ", lane_attr_type: ",
      //         lane_attr_type,
      //                      ", lane_attr_conf: ", lane_attr_conf);
      //     Log2DDS::LogDataV2(
      //         "attr_type_filter",
      //         object_debug + " Filtered when LANEATTR_LEFT/RIGHT");
      //     return FilterReason::STATIONARY_OBJECT_NOT_ON_TARGET_LANE;
      //   }
      // }
    }
  }

  // constexpr double kAttrConfThr = 0.5;
  // constexpr double kAttrConfDist = 70.0;
  // if (prediction::IsStationaryTrajectory(traj)) {
  //   auto lane_attr_type =
  //       object.object_proto().vision_attribute().lane_attr_type();
  //   auto lane_attr_conf =
  //       object.object_proto().vision_attribute().lane_attr_conf();
  //   std::string object_debug = absl::StrCat("obj_id: ", object.id(),
  //                                           ", obj_type: ", object.type());
  //   object_debug = absl::StrCat(object_debug,
  //                               ", lane_attr_type: ", lane_attr_type,
  //                               ", lane_attr_conf: ", lane_attr_conf);
  //   const auto frenet_box =
  //       drive_passage_->QueryFrenetBoxAt(object.bounding_box());
  //   const auto frenet_ego_box = drive_passage_->QueryFrenetBoxAt(ego_box_);
  //   if (!frenet_box.ok() || !frenet_ego_box.ok()) {
  //     object_debug =
  //         object_debug +
  //         ", Filter failed because failed to get frenet box information";
  //     Log2DDS::LogDataV2("attr_type_filter", object_debug);
  //     return FilterReason::NONE;
  //   }
  //   const double dist_to_ego =
  //       frenet_box.value().s_min - frenet_ego_box.value().s_max;

  //   // patch for misidentificaiton the reflector panel as CONE in front of
  //   // junction
  //   double dist_to_junction = 0.0;
  //   if (drive_passage_->lane_seq_info()->dist_to_junction > 0.0 &&
  //       drive_passage_->lane_seq_info()->dist_to_junction > dist_to_ego) {
  //     dist_to_junction =
  //         drive_passage_->lane_seq_info()->dist_to_junction - dist_to_ego;
  //   }
  //   if (object.type() == st::ObjectType::OT_CONE && dist_to_junction > 0.0 &&
  //       dist_to_junction < 20.0 &&
  //       lane_attr_type == LaneAttrType::LANEATTR_ON_LINE &&
  //       lane_attr_conf > kAttrConfThr) {
  //     Log2DDS::LogDataV2("attr_type_filter",
  //                        object_debug + ", Filter cone on_line near
  //                        junction");
  //     return FilterReason::STATIONARY_OBJECT_NOT_ON_TARGET_LANE;
  //   }

  //   if (object.type() == st::ObjectType::OT_BARRIER ||
  //       object.type() == st::ObjectType::OT_CONE ||
  //       object.type() == st::ObjectType::OT_WARNING_TRIANGLE) {
  //     // considered only in vision fusion and radar vision fusion
  //     if (object.object_proto().fusion_source() ==
  //             st::FusionSource::VISION_FUSION ||
  //         object.object_proto().fusion_source() ==
  //             st::FusionSource::RADAR_VISION_FUSION ||
  //         object.object_proto().fusion_source() ==
  //             st::FusionSource::LIDAR_VISION_FUSION ||
  //         object.object_proto().fusion_source() ==
  //             st::FusionSource::RADAR_LIDAR_VISION_FUSION) {
  //       object_debug =
  //           absl::StrCat(object_debug,
  //                        ", obj_smin: ", frenet_box.value().s_min,
  //                        ", ego_smax: ", frenet_ego_box.value().s_max,
  //                        ", dist_to_ego: ", dist_to_ego);
  //       if (dist_to_ego < kAttrConfDist) {
  //         object_debug = absl::StrCat(object_debug, ", Filter failed because
  //         dist < ",
  //                                     kAttrConfDist);
  //         Log2DDS::LogDataV2("attr_type_filter", object_debug);
  //         return FilterReason::NONE;
  //       } else {
  //         if (lane_attr_conf < kAttrConfThr) {
  //           object_debug = absl::StrCat(object_debug, ",Filter failed because
  //           conf < ",
  //                                       kAttrConfThr);
  //           Log2DDS::LogDataV2("attr_type_filter", object_debug);
  //           return FilterReason::NONE;
  //         } else {
  //           if (lane_attr_type == LaneAttrType::LANEATTR_OTHER) {
  //             Log2DDS::LogDataV2("attr_type_filter", object_debug);
  //             return FilterReason::STATIONARY_OBJECT_NOT_ON_TARGET_LANE;
  //           } else {
  //             // for keep / lc pause / lc return
  //             object_debug =
  //                 absl::StrCat(object_debug,
  //                              ", lc_stage: ", lc_state_->stage()),
  //                              ", lc_left: ", lc_state_->lc_left());
  //             if ((lc_state_->stage() == LaneChangeStage::LCS_NONE ||
  //                  lc_state_->stage() == LaneChangeStage::LCS_PAUSE ||
  //                  lc_state_->stage() == LaneChangeStage::LCS_RETURN) &&
  //                 (lane_attr_type == LaneAttrType::LANEATTR_LEFT ||
  //                  lane_attr_type == LaneAttrType::LANEATTR_RIGHT)) {
  //               Log2DDS::LogDataV2("attr_type_filter", object_debug);
  //               return FilterReason::STATIONARY_OBJECT_NOT_ON_TARGET_LANE;
  //             } else if (lc_state_->lc_left() &&
  //                        lane_attr_type == LaneAttrType::LANEATTR_RIGHT) {
  //               Log2DDS::LogDataV2("attr_type_filter", object_debug);
  //               return FilterReason::STATIONARY_OBJECT_NOT_ON_TARGET_LANE;
  //             } else if ((!lc_state_->lc_left()) &&
  //                        lane_attr_type == LaneAttrType::LANEATTR_LEFT) {
  //               Log2DDS::LogDataV2("attr_type_filter", object_debug);
  //               return FilterReason::STATIONARY_OBJECT_NOT_ON_TARGET_LANE;
  //             }
  //           }
  //         }
  //       }
  //     }
  //   }
  // }

  return FilterReason::NONE;
}

}  // namespace planning
}  // namespace st
