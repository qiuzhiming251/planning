

#ifndef AD_BYD_PLANNING_COMMON_OBSTACLE_FRAME_H
#define AD_BYD_PLANNING_COMMON_OBSTACLE_FRAME_H

#include <string>
#include <unordered_set>

#include "input_frame.h"
#include "plan_common/obstacle/lane_graph.h"
#include "plan_common/obstacle/obstacle_behavior.h"
#include "prediction_trajectory.h"
#include "plan_common/type_def.h"
#include "plan_common/maps/polyline.h"
#include "plan_common/maps/map_graph.h"
#include "plan_common/math/vec2d.h"

namespace ad_byd {
namespace planning {

using MathVec2d = ::ad_byd::planning::math::Vec2d;

struct AnchorPoints {
  MathVec2d left_front_position;
  MathVec2d right_front_position;
  MathVec2d left_back_position;
  MathVec2d right_back_position;
};

class LaneFeature {
 public:
  LaneFeature() = default;
  ~LaneFeature() = default;
  std::map<NearbyType, std::string> nearby_type_2_name_ = {
      {NearbyType::UNSET, "UNSET"},
      {NearbyType::LEFT, "LEFT"},
      {NearbyType::RIGHT, "RIGHT"},
  };

  void SetLaneId(const std::string& lane_id);
  void SetLanePtr(const LaneConstPtr& lane_ptr);
  void SetLaneS(const double lane_s);
  void SetLaneL(const double lane_l);
  void SetAngleDiff(const double angle_diff);
  void SetNearbyType(NearbyType nearby_type);
  void SetLaneHeading(const double heading);
  void SetLaneRelation(const ObstacleLaneRelation& lane_relation);

  bool HasLaneId() const { return has_lane_id_; }
  bool HasLanePtr() const { return has_lane_ptr_; }
  bool HasLaneS() const { return has_lane_s_; }
  bool HasLaneL() const { return has_lane_l_; }
  bool HasAngleDiff() const { return has_angle_diff_; }
  bool HasLaneHeading() const { return has_lane_heading_; }
  bool HasNearbyType() const { return has_nearby_type_; }
  bool HasLaneRelation() const { return has_lane_relation_; }

  const std::string& lane_id() const { return lane_id_; }
  const LaneConstPtr& lane_ptr() const { return lane_ptr_; }
  double lane_s() const { return lane_s_; }
  double lane_l() const { return lane_l_; }
  double angle_diff() const { return angle_diff_; }
  const double lane_heading() const { return lane_heading_; }
  const NearbyType& nearby_type() const { return nearby_type_; }
  const ObstacleLaneRelation& lane_relation() const { return lane_relation_; }
  std::string DebugString() const {
    std::ostringstream oss;
    oss << " LaneFeature: lane_id:" << lane_id_ << " lane_s:" << lane_s_
        << " lane_l:" << lane_l_ << " angle_diff:" << angle_diff_
        << " nearby_type:" << nearby_type_2_name_.at(nearby_type_);
    return oss.str();
  }

 private:
  std::string lane_id_ = "";
  LaneConstPtr lane_ptr_ = nullptr;
  double lane_s_ = 0.0;
  double lane_l_ = 0.0;
  double angle_diff_ = 0.0;
  double lane_heading_ = 0.0;
  NearbyType nearby_type_ = NearbyType::UNSET;
  ObstacleLaneRelation lane_relation_ = ObstacleLaneRelation::UNKNOWN;

  bool has_lane_id_ = false;
  bool has_lane_ptr_ = false;
  bool has_lane_s_ = false;
  bool has_lane_l_ = false;
  bool has_angle_diff_ = false;
  bool has_nearby_type_ = false;
  bool has_lane_heading_ = false;
  bool has_lane_relation_ = false;
};
using LaneFeaturePtr = std::shared_ptr<LaneFeature>;

class JunctionFeature {
 public:
  JunctionFeature() = default;
  ~JunctionFeature() = default;

  void SetId(const std::string& id);
  void SetEntryHeading(const double heading);
  void SetDistance(const double distance);
  void SetEntryLaneFeature(const LaneFeature& lane_fea);
  void PushExitLaneFeature(const TurnType& turn_type,
                           const LaneFeature& lane_fea);
  void PushReverseExitLaneFeature(const TurnType& turn_type,
                                  const LaneFeature& lane_fea);
  void SetJunctionPtr(const JunctionConstPtr& junction_ptr);
  void SetEntryHeadingConverged(const bool converged);
  void SetIsRoundAbout(const bool is_round_about);

  const std::string& id() const { return id_; }
  const double entry_heading() const { return entry_heading_; }
  const double distance() const { return distance_; }
  const std::unordered_map<TurnType, std::vector<LaneFeaturePtr>>&
  turn_exit_lane_map() const {
    return turn_exit_lane_map_;
  }
  const LaneFeature& entry_lane_feature() const { return entry_lane_feature_; }
  bool GetExitLaneFeatures(
      const TurnType& turn_type,
      std::vector<LaneFeaturePtr>* const lane_features) const;
  bool GetReverseExitLaneFeatures(
      const TurnType& turn_type,
      std::vector<LaneFeaturePtr>* const lane_features) const;
  const JunctionConstPtr& JunctionPtr() const { return junction_ptr_; }
  bool EntryHeadingConverged() const { return entry_heading_converged_; }
  const std::unordered_map<std::string, LaneFeaturePtr>& exit_lanes() const {
    return exit_lanes_;
  }
  const std::unordered_map<std::string, TurnType>& exit_lane_turn_map() const {
    return exit_lane_turn_map_;
  }
  const std::unordered_map<std::string, LaneFeaturePtr>& reverse_exit_lanes()
      const {
    return reverse_exit_lanes_;
  }
  const std::unordered_map<std::string, TurnType>& reverse_exit_lane_turn_map()
      const {
    return reverse_exit_lane_turn_map_;
  }
  const bool is_roundabout() const { return is_roundabout_; }

  bool HasId() const { return has_id_; }
  bool HasEntryLaneFeature() const { return has_entry_lane_feature_; }
  bool HasEntryHeading() const { return has_entry_heading_; }
  bool HasDistance() const { return has_distance_; }
  bool HasJunctionPtr() const { return has_junction_ptr_; }
  bool HasEntryHeadingConverged() const { return has_entry_heading_converged_; }

 private:
  std::string id_;
  double entry_heading_ = 0.0;
  double distance_ = 0.0;
  LaneFeature entry_lane_feature_;
  bool is_roundabout_ = false;
  std::unordered_map<TurnType, std::vector<LaneFeaturePtr>> turn_exit_lane_map_;
  std::unordered_map<std::string, TurnType> exit_lane_turn_map_;
  std::unordered_map<std::string, LaneFeaturePtr> exit_lanes_;
  std::unordered_map<TurnType, std::vector<LaneFeaturePtr>>
      turn_reverse_exit_lane_map_;
  std::unordered_map<std::string, LaneFeaturePtr> reverse_exit_lanes_;
  std::unordered_map<std::string, TurnType> reverse_exit_lane_turn_map_;

  JunctionConstPtr junction_ptr_ = nullptr;
  bool entry_heading_converged_ = false;

  bool has_id_ = false;
  bool has_entry_heading_ = false;
  bool has_distance_ = false;
  bool has_entry_lane_feature_ = false;
  bool has_junction_ptr_ = false;
  bool has_entry_heading_converged_ = false;
  bool has_is_roundabout_ = false;
};
using JunctionFeaturePtr = std::shared_ptr<JunctionFeature>;

class ObstacleFrame {
 public:
  // set functions
  void SetTimestamp(double timestamp);
  void SetSeqNum(int64_t seq_num);
  void SetId(const std::string& id);
  void SetType(const ObstacleType& type);
  void SetSuperType(const ObstacleSuperType& type);
  void SetFusionType(const int8_t type);
  void SetObstacleState(int8_t state);
  void SetPosition(const MathVec2d& location);
  void SetAnchorPoints(const AnchorPoints& anchor_pts);
  void SetVelocity(const MathVec2d& velocity);
  void SetFilterVelocity(const MathVec2d& velocity);
  void SetVecAcc(const MathVec2d& acc);
  void SetFilterVecAcc(const MathVec2d& acc);
  void SetHeading(double heading);
  void SetLength(double length);
  void SetWidth(double width);
  void SetHeight(const double height);
  void SetPolygon(const std::vector<Vec2d>& polygon);
  void SetAdcPos(const MathVec2d& adc_pos);
  void SetRelativeSL(const SLPoint& relative_sl);
  void SetThetaToAdc(double theta_to_adc);
  void SetDistToAdc(double dist_to_adc);
  void SetSpeed(double speed);
  void SetFilterSpeed(double speed);
  void SetYawRate(double yaw_rate);
  void SetScalarAcc(double scalar_acc);
  void SetIsStatic(bool is_static);
  void SetIsReverse(const bool is_reverse);
  void SetIsKeyObject(const bool is_key_obj);
  void SetIsMotion(const bool is_motion);

  void SetRankerType(const RankerType& ranker_type);
  void SetVecPoint(const VectorPoint& vp);
  void SetSkipHardBoundary(const bool flag);
  void SetRightMergedAgent();

  void SetCurrentLanes(const std::vector<LaneFeature>& current_lanes);
  void SetLeftFrontLanes(const std::vector<LaneFeature>& current_lanes);
  void SetRightFrontLanes(const std::vector<LaneFeature>& current_lanes);
  void SetNearbyLanes(const std::vector<LaneFeature>& nearby_lanes);
  void SetLaneFeature(const LaneFeature& lane_feature);
  void SetLeftFrontLaneFeature(const LaneFeature& lane_feature);
  void SetRightFrontLaneFeature(const LaneFeature& lane_feature);
  void SetMapFeatureFlag();
  void SetJunctionFeatures(
      const std::vector<JunctionFeature>& junction_features);
  void SetClosestJunctionFeature(const JunctionFeature& junction_feature);
  void ClearCurrentLanes();
  void SetStaticScene(const ObstacleStaticScene& obstacle_scene);
  void SetObstacleBehavior(obstacle::ObstacleBehavior&& obstacle_behavior);
  void SetPredictionTrajectories(
      std::vector<PredictionTrajectory>&& prediction_trajectories);
  void SetSuperByType(ObstacleFrame* const current_feature);
  void SetObstacleLight(const std::vector<ObstacleLightType>& obstacle_lights);
  void SetReverseFlag();
  void SetJudgeCutin(const bool judge_cutin);
  void SetAdcTurnIntentionInJunction(const TurnType turn_type);
  void SetJunctionIntentionLane(const LaneConstPtr lane_ptr);
  void SetCutinIntention(const int32_t flag);

  void SetRerPtrLeftCornerL(double value);
  void SetRefPtrRIghtCornerL(double value);

  // get functions
  /*   osbtacle   */
  double timestamp() const { return timestamp_; };
  int64_t seq_num() const { return seq_num_; };
  const std::string& id() const { return id_; };
  ObstacleType type() const { return type_; };
  ObstacleSuperType super_type() const { return super_type_; };
  int8_t fusion_type() const { return fusion_type_; };
  int8_t obstacle_state() const { return obstacle_state_; };
  const MathVec2d& position() const { return position_; };
  const std::vector<Vec2d>& polygon() const { return polygon_; };
  const AnchorPoints& anchor_points() const { return anchor_points_; };
  const MathVec2d& velocity() const { return velocity_; };
  const MathVec2d& filter_velocity() const { return filter_velocity_; };
  const MathVec2d& vec_acc() const { return vec_acc_; };
  const MathVec2d& filter_vec_acc() const { return filter_vec_acc_; };
  double heading() const { return heading_; };
  double length() const { return length_; };
  double width() const { return width_; };
  double height() const { return height_; };
  const MathVec2d& adc_pos() const { return adc_pos_; };
  SLPoint relative_sl() { return relative_sl_; };
  double theta_to_adc() const { return theta_to_adc_; };
  double dist_to_adc() const { return dist_to_adc_; };
  double speed() const { return speed_; };
  double filter_speed() const { return filter_speed_; };
  double yaw_rate() const { return yaw_rate_; };
  double scalar_acc() const { return scalar_acc_; };
  double ref_ptr_left_corner_l() const { return ref_ptr_left_corner_l_; }
  double ref_ptr_right_corner_l() const { return ref_ptr_right_corner_l_; }
  bool is_static() const { return is_static_; };
  bool is_reverse() const { return is_reverse_; };
  bool is_key_object() const;
  bool is_position_smooth() const;
  bool is_interpolation() const { return is_interpolation_; };
  double life_time() const;
  bool is_velcity_converged() const { return is_velocity_converged_; };
  /*  predcition  */
  const RankerType& ranker_type() const { return ranker_type_; };
  RankerType* mutable_ranker_type() { return &ranker_type_; };
  const VectorPoint& vec_point() const { return vec_point_; };
  /*  map feature  */
  bool in_roundabout() const { return in_roundabout_; };
  const obstacle::LaneGraph& lane_graph() const { return lane_graph_; };
  obstacle::LaneGraph* lane_graph_ptr() { return &lane_graph_; };
  const std::vector<LaneFeature>& current_lanes() const {
    return current_lanes_;
  };
  const std::vector<LaneFeature>& left_front_lanes() const {
    return left_front_lanes_;
  };
  const std::vector<LaneFeature>& right_front_lanes() const {
    return right_front_lanes_;
  };
  const std::vector<LaneFeature>& nearby_lanes() const {
    return nearby_lanes_;
  };
  const std::vector<JunctionFeature>& junction_features() const {
    return junction_features_;
  };
  const LaneFeature& lane_feature() const { return lane_feature_; };
  const LaneFeature& left_front_lane_feature() const {
    return left_front_lane_feature_;
  };
  const LaneFeature& right_front_lane_feature() const {
    return right_front_lane_feature_;
  };
  const JunctionFeature& closest_junction_feature() const {
    return closest_junction_feature_;
  };
  bool map_feature_flag() const { return map_feature_flag_; };
  ObstacleStaticScene static_scene() const { return static_scene_; };
  bool is_motion() const { return is_motion_; }
  bool skip_hard_boundary() const { return skip_hard_boundary_; }
  bool right_merged() const { return right_merged_; }
  const obstacle::ObstacleBehavior& obstacle_behavior() const {
    return obstacle_behavior_;
  }
  obstacle::ObstacleBehavior* mutable_obstacle_behavior() {
    return &obstacle_behavior_;
  }
  const std::vector<PredictionTrajectory> prediction_trajectories() const {
    return prediction_trajectories_;
  }
  std::vector<PredictionTrajectory>* mutable_prediction_trajectories() {
    return &prediction_trajectories_;
  }
  const std::vector<ObstacleLightType> obstacle_lights() const {
    return obstacle_lights_;
  }
  // std::string debug_string() const;
  bool reverse_flag() const { return reverse_flag_; }
  bool judge_cutin() const { return judge_cutin_; }
  TurnType adc_turn_intention_in_junction() const {
    return adc_turn_intention_in_junction_;
  }
  LaneConstPtr junction_intention_lane() const {
    return junction_intention_lane_;
  }
  int32_t cutin_intention() const { return cutin_intention_; }

  // has functions
  /*   obstacle  */
  bool HasTimestamp() const { return has_timestamp_; };
  bool HasSeqNum() const { return has_seq_num_; };
  bool HasId() const { return has_id_; };
  bool HasType() const { return has_type_; };
  bool HasSuperType() const { return has_super_type_; };
  bool HasPosition() const { return has_position_; };
  bool HasAnchorPoints() const { return has_anchor_points_; };
  bool HasVelocity() const { return has_velocity_; };
  bool HasVecAcc() const { return has_vec_acc_; };
  bool HasFilterVelocity() const { return has_filter_velocity_; };
  bool HasFilterVecAcc() const { return has_filter_vec_acc_; };
  bool HasAdcPos() const { return has_adc_pos_; };
  bool HasHeading() const { return has_heading_; };
  bool HasLength() const { return has_length_; };
  bool HasWidth() const { return has_width_; };
  bool HasHeight() const { return has_height_; };
  bool HasThetaToAdc() const { return has_theta_to_adc_; };
  bool HasDistToAdc() const { return has_dist_to_adc_; };
  bool HasLifeTime() const { return has_life_time_; };
  bool HasYawRate() const { return has_yaw_rate_; };
  bool HasIsKeyObject() const { return has_is_key_object_; };
  bool HasIsPositionSmooth() const { return has_is_position_smooth_; };
  bool HasSpeed() const { return has_speed_; };
  bool HasFilterSpeed() const { return has_filter_speed_; };
  bool HasScalarAcc() const { return has_scalar_acc_; };
  bool HasVecPoint() const { return has_vec_point_; };
  bool HasIsInterpolation() const { return has_is_interpolation_; };
  bool HasRelativeSL() const { return has_relative_sl_; };
  bool HasIsVelocityConverged() const { return has_is_velocity_converged_; };
  /*  prediction  */
  bool HasPolylineInfos() const { return has_polyline_infos_; };
  bool HasObstacleBehavior() const { return has_obstacle_behavior_; };
  bool HasNetworkFeatureExtractFlag() const {
    return has_network_feature_extract_flag_;
  };
  bool HasSkip() const { return has_skip_; };
  bool HasIsStatic() const;
  bool HasModelType() const;
  bool HasCutinInfo() const;
  bool HasPredictTrajectories() const;
  bool HasFusionType() const;
  bool HasVsObstacleState() const;
  bool HasRankerType() const { return has_ranker_type_; };
  bool HasIsReverse() const;
  bool HasJudgeCutin() const { return has_judge_cutin_; };
  /*  map feature  */
  bool HasInRoundabout() const { return has_in_roundabout_; };
  bool HasStaticScene() const { return has_static_scene_; };
  bool HasLaneGraph() const { return has_lane_graph_; };
  bool HasCurrentLanes() const { return has_current_lanes_; };
  bool HasLeftFrontLanes() const { return has_left_front_lanes_; };
  bool HasRightFrontLanes() const { return has_right_front_lanes_; };
  bool HasNearbyLanes() const { return has_nearby_lanes_; };
  bool HasJunctionFeatures() const { return has_junction_features_; };
  bool HasLaneFeature() const { return has_lane_feature_; };
  bool HasLeftFrontLaneFeature() const { return has_left_front_lane_feature_; };
  bool HasRightFrontLaneFeature() const {
    return has_right_front_lane_feature_;
  };
  bool HasClosestJunctionFeature() const {
    return has_closest_junction_feature_;
  };
  bool HasMapFeatureFlag() const { return has_map_feature_flag_; };
  bool HasPredictionTrajectory() const { return has_prediction_trajectory_; }
  bool HasAdcTurnIntentionInJunction() const {
    return has_adc_turn_intention_in_junction_;
  }
  bool HasJunctionIntentionLane() const { return has_junction_intention_lane_; }
  bool HasRefPtrLeftCornerL() const { return has_ref_ptr_left_corner_l_; }
  bool HasRefPtrRightCornerL() const { return has_ref_ptr_right_corner_l_; }

 private:
  /*   obstacle   */
  double timestamp_ = 0;
  int64_t seq_num_ = -1;
  std::string id_;
  ObstacleType type_ = ObstacleType::OBJECT_UNKNOWN;
  ObstacleSuperType super_type_ = ObstacleSuperType::UNKNOWN;
  int8_t fusion_type_ = 0;
  int8_t obstacle_state_ = 0;
  MathVec2d position_;
  std::vector<Vec2d> polygon_;
  AnchorPoints anchor_points_;
  MathVec2d velocity_;
  MathVec2d filter_velocity_;
  MathVec2d vec_acc_;
  MathVec2d filter_vec_acc_;
  double heading_ = 0;
  double length_ = 0;
  double width_ = 0;
  double height_ = 0;
  MathVec2d adc_pos_;
  SLPoint relative_sl_;
  double theta_to_adc_ = 0;
  double dist_to_adc_ = 0;
  double speed_ = 0;
  double filter_speed_ = 0;
  double yaw_rate_ = 0.0;
  double scalar_acc_ = 0;
  bool is_static_ = false;
  bool is_reverse_ = false;
  bool is_key_object_ = false;
  bool is_position_smooth_ = true;
  bool is_interpolation_ = false;
  double life_time_ = 0.0;
  bool is_velocity_converged_ = false;
  bool is_motion_ = false;
  bool reverse_flag_ = false;

  double ref_ptr_left_corner_l_ = 0.0;
  double ref_ptr_right_corner_l_ = 0.0;
  int32_t cutin_intention_ = 0;
  std::vector<ObstacleLightType> obstacle_lights_;

  std::unordered_set<ObstacleType> static_type_set_{
      ObstacleType::OBJECT_CONE,
      ObstacleType::OBJECT_BARREL,
      ObstacleType::OBJECT_BARRIER,
      ObstacleType::OBJECT_SAFETY_TRIANGLE,
      ObstacleType::OBJECT_PARKING_LOCK,
      ObstacleType::OBJECT_SPACE_LIMITER,
      ObstacleType::OBJECT_UNKNOWN_UNMOVABLE,
  };

  std::unordered_set<ObstacleType> vehicle_type_set_{
      ObstacleType::OBJECT_CAR,
      ObstacleType::OBJECT_TRUCK,
      ObstacleType::OBJECT_SPECIALVEHICLE,
      ObstacleType::OBJECT_BUS,
      ObstacleType::OBJECT_MINIVEHICLE,
      ObstacleType::OBJECT_CONSTRUCTION_VEHICLE,
      ObstacleType::OBJECT_UNKNOWN_MOVABLE,
  };

  std::unordered_set<ObstacleType> cyclist_type_set_{
      ObstacleType::OBJECT_MOTORCYCLE,
      ObstacleType::OBJECT_BICYCLE,
      ObstacleType::OBJECT_TRICYCLE,
  };

  std::unordered_set<ObstacleType> pedestrian_type_set_{
      ObstacleType::OBJECT_PEDESTRIAN,
  };

  /*   prediction   */
  // ModelType model_type_ = ModelType::NONE;
  bool network_feature_extract_flag_ = false;
  VectorPoint vec_point_;
  // std::vector<PolylineInfo> polyline_infos_;
  // std::vector<TupleTensor> vectornet_tensor_;
  // ObstacleBehavior behavior_;
  // std::vector<Trajectory> predict_trajectories_;
  RankerType ranker_type_;
  // VSFusionType fusion_type_ = VSFusionType::FusionType_UNKNOWN;
  bool is_fused_lidar_ = false;
  int8_t vs_obstacle_state_ = 0;
  bool skip_ = false;

  /*    map feature    */
  bool in_roundabout_ = false;
  ObstacleStaticScene static_scene_ = ObstacleStaticScene::OFF_JUNCTION;
  obstacle::LaneGraph lane_graph_;
  std::vector<LaneFeature> current_lanes_;
  std::vector<LaneFeature> left_front_lanes_;
  std::vector<LaneFeature> right_front_lanes_;
  std::vector<LaneFeature> nearby_lanes_;
  std::vector<JunctionFeature> junction_features_;
  LaneFeature lane_feature_;
  LaneFeature left_front_lane_feature_;
  LaneFeature right_front_lane_feature_;
  JunctionFeature closest_junction_feature_;
  bool map_feature_flag_ = false;
  bool skip_hard_boundary_ = false;
  bool right_merged_ = false;
  obstacle::ObstacleBehavior obstacle_behavior_;
  std::vector<PredictionTrajectory> prediction_trajectories_;
  TurnType adc_turn_intention_in_junction_ = TurnType::NO_TURN;
  LaneConstPtr junction_intention_lane_ = nullptr;
  /*   obstacle   */
  bool has_timestamp_ = false;
  bool has_seq_num_ = false;
  bool has_id_ = false;
  bool has_type_ = false;
  bool has_super_type_ = false;
  bool has_position_ = false;
  bool has_polygon_ = false;
  bool has_anchor_points_ = false;
  bool has_velocity_ = false;
  bool has_filter_velocity_ = false;
  bool has_vec_acc_ = false;
  bool has_filter_vec_acc_ = false;
  bool has_heading_ = false;
  bool has_length_ = false;
  bool has_width_ = false;
  bool has_height_ = false;
  bool has_adc_pos_ = false;
  bool has_relative_sl_ = false;
  bool has_theta_to_adc_ = false;
  bool has_dist_to_adc_ = false;
  bool has_speed_ = false;
  bool has_filter_speed_ = false;
  bool has_yaw_rate_ = false;
  bool has_scalar_acc_ = false;
  bool has_is_static_ = false;
  bool has_is_reverse_ = false;
  bool has_is_key_object_ = false;
  bool has_is_position_smooth_ = false;
  bool has_is_interpolation_ = false;
  bool has_life_time_ = false;
  bool has_is_velocity_converged_ = false;
  bool has_is_motion_ = false;

  /*   prediction   */
  bool has_model_type_ = false;
  bool has_network_feature_extract_flag_ = false;
  bool has_vec_point_ = false;
  bool has_polyline_infos_ = false;
  bool has_vectornet_tensor_ = false;
  bool has_obstacle_behavior_ = false;
  bool has_predict_trajectories_ = false;
  bool has_predictor_type_ = false;
  bool has_ranker_type_ = false;
  bool has_fusion_type_ = false;
  bool has_is_fusion_lidar_ = false;
  bool has_vs_obstacle_state_ = false;
  bool has_skip_ = false;
  bool has_prediction_trajectory_ = false;
  bool has_judge_cutin_ = false;
  bool judge_cutin_ = false;
  bool has_adc_turn_intention_in_junction_ = false;
  bool has_junction_intention_lane_ = false;

  /*   map feature   */
  bool has_in_roundabout_ = false;
  bool has_static_scene_ = false;
  bool has_lane_graph_ = false;
  bool has_current_lanes_ = false;
  bool has_left_front_lanes_ = false;
  bool has_right_front_lanes_ = false;
  bool has_nearby_lanes_ = false;
  bool has_junction_features_ = false;
  bool has_lane_feature_ = false;
  bool has_left_front_lane_feature_ = false;
  bool has_right_front_lane_feature_ = false;
  bool has_closest_junction_feature_ = false;
  bool has_map_feature_flag_ = false;
  bool has_ref_ptr_left_corner_l_ = false;
  bool has_ref_ptr_right_corner_l_ = false;
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_COMMON_OBSTACLE_FRAME_H