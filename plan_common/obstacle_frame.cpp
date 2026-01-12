#include "obstacle_frame.h"
#include "plan_common/log.h"

namespace ad_byd {
namespace planning {

////////////////////////////////
/********* LaneFeature ********/
void LaneFeature::SetLaneId(const std::string& lane_id) {
  lane_id_ = lane_id;
  has_lane_id_ = true;
}
void LaneFeature::SetLanePtr(const LaneConstPtr& lane_info) {
  lane_ptr_ = lane_info;
  has_lane_ptr_ = true;
}
void LaneFeature::SetLaneS(const double lane_s) {
  lane_s_ = lane_s;
  has_lane_s_ = true;
}
void LaneFeature::SetLaneL(const double lane_l) {
  lane_l_ = lane_l;
  has_lane_l_ = true;
}
void LaneFeature::SetAngleDiff(const double angle_diff) {
  angle_diff_ = angle_diff;
  has_angle_diff_ = true;
}
void LaneFeature::SetLaneHeading(const double heading) {
  has_lane_heading_ = true;
  lane_heading_ = heading;
}
void LaneFeature::SetNearbyType(NearbyType nearby_type) {
  nearby_type_ = nearby_type;
  has_nearby_type_ = true;
}
void LaneFeature::SetLaneRelation(const ObstacleLaneRelation& lane_relation) {
  lane_relation_ = lane_relation;
  has_lane_relation_ = true;
}

//////////////////////////////////////////
/************JunctionFeature*************/
void JunctionFeature::SetId(const std::string& id) {
  has_id_ = true;
  id_ = id;
}
void JunctionFeature::PushExitLaneFeature(const TurnType& turn_type,
                                          const LaneFeature& lane_fea) {
  std::shared_ptr<LaneFeature> lane_fea_ptr =
      std::make_shared<LaneFeature>(lane_fea);
  if (turn_exit_lane_map_.find(turn_type) != turn_exit_lane_map_.end()) {
    turn_exit_lane_map_[turn_type].emplace_back(lane_fea_ptr);
  } else {
    std::vector<std::shared_ptr<LaneFeature>> lane_feas = {lane_fea_ptr};
    turn_exit_lane_map_[turn_type] = lane_feas;
  }
  exit_lane_turn_map_[lane_fea_ptr->lane_id()] = turn_type;
  exit_lanes_[lane_fea_ptr->lane_id()] = lane_fea_ptr;
}
void JunctionFeature::PushReverseExitLaneFeature(const TurnType& turn_type,
                                                 const LaneFeature& lane_fea) {
  std::shared_ptr<LaneFeature> lane_fea_ptr =
      std::make_shared<LaneFeature>(lane_fea);
  if (turn_reverse_exit_lane_map_.find(turn_type) !=
      turn_reverse_exit_lane_map_.end()) {
    turn_reverse_exit_lane_map_[turn_type].emplace_back(lane_fea_ptr);
  } else {
    std::vector<std::shared_ptr<LaneFeature>> lane_feas = {lane_fea_ptr};
    turn_reverse_exit_lane_map_[turn_type] = lane_feas;
  }
  reverse_exit_lane_turn_map_[lane_fea_ptr->lane_id()] = turn_type;
  reverse_exit_lanes_[lane_fea_ptr->lane_id()] = lane_fea_ptr;
}
void JunctionFeature::SetEntryHeading(const double heading) {
  has_entry_heading_ = true;
  entry_heading_ = heading;
}
void JunctionFeature::SetDistance(const double distance) {
  has_distance_ = true;
  distance_ = distance;
}
void JunctionFeature::SetEntryLaneFeature(const LaneFeature& lane_feat) {
  has_entry_lane_feature_ = true;
  entry_lane_feature_ = lane_feat;
}
void JunctionFeature::SetJunctionPtr(const JunctionConstPtr& junction_ptr) {
  has_junction_ptr_ = true;
  junction_ptr_ = junction_ptr;
}
void JunctionFeature::SetEntryHeadingConverged(const bool converged) {
  has_entry_heading_converged_ = true;
  entry_heading_converged_ = converged;
}

bool JunctionFeature::GetExitLaneFeatures(
    const TurnType& turn_type,
    std::vector<LaneFeaturePtr>* const lane_features) const {
  if (turn_exit_lane_map_.find(turn_type) == turn_exit_lane_map_.end() ||
      turn_exit_lane_map_.at(turn_type).empty()) {
    return false;
  }
  lane_features->clear();
  const auto& lane_feas = turn_exit_lane_map_.at(turn_type);
  lane_features->assign(lane_feas.begin(), lane_feas.end());
  return true;
}
void JunctionFeature::SetIsRoundAbout(const bool is_round_about) {
  is_roundabout_ = is_round_about;
  has_is_roundabout_ = true;
}
bool JunctionFeature::GetReverseExitLaneFeatures(
    const TurnType& turn_type,
    std::vector<LaneFeaturePtr>* const lane_features) const {
  if (turn_reverse_exit_lane_map_.find(turn_type) ==
          turn_reverse_exit_lane_map_.end() ||
      turn_reverse_exit_lane_map_.at(turn_type).empty()) {
    return false;
  }
  lane_features->clear();
  const auto& lane_feas = turn_reverse_exit_lane_map_.at(turn_type);
  lane_features->assign(lane_feas.begin(), lane_feas.end());
  return true;
}

//////////////////////////////////////////
/*********** ObstacleFrame***************/
void ObstacleFrame::SetTimestamp(double timestamp) {
  timestamp_ = timestamp;
  has_timestamp_ = true;
}
void ObstacleFrame::SetSeqNum(int64_t seq_num) {
  seq_num_ = seq_num;
  has_seq_num_ = true;
}
void ObstacleFrame::SetId(const std::string& id) {
  id_ = id;
  has_id_ = true;
}
void ObstacleFrame::SetType(const ObstacleType& type) {
  type_ = type;
  has_type_ = true;
}
void ObstacleFrame::SetSuperType(const ObstacleSuperType& type) {
  super_type_ = type;
  has_super_type_ = true;
}
void ObstacleFrame::SetFusionType(const int8_t type) { fusion_type_ = type; }
void ObstacleFrame::SetObstacleState(int8_t state) { obstacle_state_ = state; }
void ObstacleFrame::SetPosition(const MathVec2d& location) {
  position_.set_x(static_cast<double>(location.x()));
  position_.set_y(static_cast<double>(location.y()));
  has_position_ = true;
}
void ObstacleFrame::SetPolygon(const std::vector<Vec2d>& polygon) {
  polygon_ = polygon;
  has_polygon_ = true;
}
void ObstacleFrame::SetAnchorPoints(const AnchorPoints& anchor_pts) {
  anchor_points_ = anchor_pts;
  has_anchor_points_ = true;
}
void ObstacleFrame::SetVelocity(const MathVec2d& velocity) {
  velocity_.set_x(static_cast<double>(velocity.x()));
  velocity_.set_y(static_cast<double>(velocity.y()));
  has_velocity_ = true;
  speed_ = velocity_.Length();
  has_speed_ = true;
}
void ObstacleFrame::SetVecAcc(const MathVec2d& acc) {
  vec_acc_.set_x(static_cast<double>(acc.x()));
  vec_acc_.set_y(static_cast<double>(acc.y()));
  has_vec_acc_ = true;
}
void ObstacleFrame::SetAdcPos(const MathVec2d& adc_pos) {
  adc_pos_ = adc_pos;
  has_adc_pos_ = true;
}
void ObstacleFrame::SetHeading(double heading) {
  heading_ = heading;
  has_heading_ = true;
}
void ObstacleFrame::SetLength(double length) {
  length_ = length;
  has_length_ = true;
}
void ObstacleFrame::SetWidth(double width) {
  width_ = width;
  has_width_ = true;
}

void ObstacleFrame::SetHeight(const double height) {
  has_height_ = true;
  height_ = height;
}

void ObstacleFrame::SetSpeed(double speed) {
  speed_ = speed;
  has_speed_ = true;
}
void ObstacleFrame::SetFilterSpeed(double speed) {
  filter_speed_ = speed;
  has_filter_speed_ = true;
}
void ObstacleFrame::SetScalarAcc(double scalar_acc) {
  scalar_acc_ = scalar_acc;
  has_scalar_acc_ = true;
}
void ObstacleFrame::SetVecPoint(const VectorPoint& vp) {
  vec_point_ = vp;
  has_vec_point_ = true;
}
void ObstacleFrame::SetSkipHardBoundary(const bool flag) {
  skip_hard_boundary_ = flag;
}
void ObstacleFrame::SetRightMergedAgent() { right_merged_ = true; }
void ObstacleFrame::SetIsStatic(bool is_static) {
  is_static_ = is_static;
  has_is_static_ = true;
}
void ObstacleFrame::SetCurrentLanes(
    const std::vector<LaneFeature>& current_lanes) {
  current_lanes_ = current_lanes;
  has_current_lanes_ = true;
}
void ObstacleFrame::SetLeftFrontLanes(
    const std::vector<LaneFeature>& current_lanes) {
  left_front_lanes_ = current_lanes;
  has_left_front_lanes_ = true;
}
void ObstacleFrame::SetRightFrontLanes(
    const std::vector<LaneFeature>& current_lanes) {
  right_front_lanes_ = current_lanes;
  has_right_front_lanes_ = true;
}
void ObstacleFrame::SetNearbyLanes(
    const std::vector<LaneFeature>& nearby_lanes) {
  nearby_lanes_ = nearby_lanes;
  has_nearby_lanes_ = true;
}
void ObstacleFrame::SetLaneFeature(const LaneFeature& lane_feature) {
  lane_feature_ = lane_feature;
  has_lane_feature_ = true;
}
void ObstacleFrame::SetLeftFrontLaneFeature(const LaneFeature& lane_feature) {
  left_front_lane_feature_ = lane_feature;
  has_left_front_lane_feature_ = true;
}
void ObstacleFrame::SetRightFrontLaneFeature(const LaneFeature& lane_feature) {
  right_front_lane_feature_ = lane_feature;
  has_right_front_lane_feature_ = true;
}
void ObstacleFrame::SetMapFeatureFlag() {
  map_feature_flag_ = true;
  has_map_feature_flag_ = true;
}
void ObstacleFrame::SetIsReverse(const bool is_reverse) {
  is_reverse_ = is_reverse;
  has_is_reverse_ = true;
}
void ObstacleFrame::SetJunctionFeatures(
    const std::vector<JunctionFeature>& junction_features) {
  junction_features_.clear();
  junction_features_.assign(junction_features.begin(), junction_features.end());
  has_junction_features_ = true;
}
void ObstacleFrame::SetClosestJunctionFeature(
    const JunctionFeature& junction_feature) {
  closest_junction_feature_ = junction_feature;
  has_closest_junction_feature_ = true;
}
void ObstacleFrame::ClearCurrentLanes() {
  has_current_lanes_ = false;
  has_lane_feature_ = false;
  current_lanes_.clear();
}
void ObstacleFrame::SetRankerType(const RankerType& ranker_type) {
  ranker_type_ = ranker_type;
  has_ranker_type_ = true;
}
void ObstacleFrame::SetThetaToAdc(const double theta) {
  theta_to_adc_ = theta;
  has_theta_to_adc_ = true;
}
void ObstacleFrame::SetDistToAdc(const double dist) {
  dist_to_adc_ = dist;
  has_dist_to_adc_ = true;
}
void ObstacleFrame::SetRelativeSL(const SLPoint& sl_pt) {
  relative_sl_ = sl_pt;
  has_relative_sl_ = true;
}
void ObstacleFrame::SetStaticScene(const ObstacleStaticScene& obstacle_scene) {
  static_scene_ = obstacle_scene;
  has_static_scene_ = true;
}

void ObstacleFrame::SetIsMotion(const bool is_motion) {
  is_motion_ = is_motion;
  has_is_motion_ = true;
}

void ObstacleFrame::SetObstacleBehavior(
    obstacle::ObstacleBehavior&& obstacle_behavior) {
  obstacle_behavior_ = std::move(obstacle_behavior);
  has_obstacle_behavior_ = true;
}

void ObstacleFrame::SetPredictionTrajectories(
    std::vector<PredictionTrajectory>&& prediction_trajectories) {
  has_prediction_trajectory_ = true;
  prediction_trajectories_ = std::move(prediction_trajectories);
}

void ObstacleFrame::SetSuperByType(ObstacleFrame* const current_feature) {
  CHECK_NOTNULL(current_feature);
  if (static_type_set_.find(current_feature->type()) !=
      static_type_set_.end()) {
    current_feature->SetSuperType(ObstacleSuperType::STATIC);
  } else if (vehicle_type_set_.find(current_feature->type()) !=
             vehicle_type_set_.end()) {
    current_feature->SetSuperType(ObstacleSuperType::VEHICLE);
  } else if (cyclist_type_set_.find(current_feature->type()) !=
             cyclist_type_set_.end()) {
    current_feature->SetSuperType(ObstacleSuperType::CYCLIST);
  } else if (pedestrian_type_set_.find(current_feature->type()) !=
             pedestrian_type_set_.end()) {
    current_feature->SetSuperType(ObstacleSuperType::PEDESTRIAN);
  } else {
    current_feature->SetSuperType(ObstacleSuperType::UNKNOWN);
  }
}

void ObstacleFrame::SetObstacleLight(
    const std::vector<ObstacleLightType>& obstacle_lights) {
  obstacle_lights_.clear();
  obstacle_lights_.assign(obstacle_lights.begin(), obstacle_lights.end());
}

void ObstacleFrame::SetReverseFlag() { reverse_flag_ = true; }

void ObstacleFrame::SetJudgeCutin(const bool judge_cutin) {
  has_judge_cutin_ = true;
  judge_cutin_ = judge_cutin;
}
void ObstacleFrame::SetAdcTurnIntentionInJunction(const TurnType turn_type) {
  has_adc_turn_intention_in_junction_ = true;
  adc_turn_intention_in_junction_ = turn_type;
}
void ObstacleFrame::SetJunctionIntentionLane(const LaneConstPtr lane_ptr) {
  has_junction_intention_lane_ = true;
  junction_intention_lane_ = lane_ptr;
}
void ObstacleFrame::SetCutinIntention(const int32_t flag) {
  cutin_intention_ = flag;
}
void ObstacleFrame::SetRerPtrLeftCornerL(double value) {
  has_ref_ptr_left_corner_l_ = true;
  ref_ptr_left_corner_l_ = value;
}
void ObstacleFrame::SetRefPtrRIghtCornerL(double value) {
  has_ref_ptr_right_corner_l_ = true;
  ref_ptr_right_corner_l_ = value;
}
}  // namespace planning
}  // namespace ad_byd