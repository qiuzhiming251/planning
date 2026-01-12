
#ifndef AD_BYD_PLANNING_COMMON_VEHICLE_STATE_H
#define AD_BYD_PLANNING_COMMON_VEHICLE_STATE_H

#include "input_frame.h"
#include "plan_common/type_def.h"

namespace ad_byd {
namespace planning {

class VehicleState {
 public:
  VehicleState() = default;
  VehicleState(const EgoInfo& ego_info) { ego_info_ = ego_info; }
  ~VehicleState() = default;

  void SetCurrentLane(const std::string& lane_id) {
    current_lane_id_ = lane_id;
  }
  void SetDistanceToJunction(const double distance_to_junction) {
    distance_to_junction_ = distance_to_junction;
  }
  void SetDistanceToNextJunction(const double distance_to_next_junction) {
    distance_to_next_junction_ = distance_to_next_junction;
  }
  void SetDistanceToTurn(const double distance_to_turn) {
    distance_to_turn_ = distance_to_turn;
  }
  void SetTurnType(const TurnType& turn_type) { turn_type_ = turn_type; }
  void SetJunctionType(const TurnType& junction_type) {
    junction_type_ = junction_type;
  }
  void SetRidedDistance(const double rided_distance) {
    rided_distance_ = rided_distance;
  }
  void SetLeftBoundaryDistance(const double left_boundary_distance) {
    left_boundary_distance_ = left_boundary_distance;
  }
  void SetRightBoundaryDistance(const double right_boundary_distance) {
    right_boundary_distance_ = right_boundary_distance;
  }
  void SetThetaToEgo(const double theta_to_ego) {
    theta_to_ego_ = theta_to_ego;
  }
  void SetFinalLaneId(const std::string& final_lane_id) {
    final_lane_id_ = final_lane_id;
  }
  void SetFirstJunctionId(const std::string first_junction_id) {
    first_junction_id_ = first_junction_id;
  }
  void SetLCCountForJunction(const double lc_count_for_junction) {
    lc_count_for_junction_ = lc_count_for_junction;
  }
  void SetLCCountForTurn(const double lc_count_for_turn) {
    lc_count_for_turn_ = lc_count_for_turn;
  }
  void SetBehindNormalLaneId(const std::string& behind_normal_lane_id) {
    behind_normal_lane_id_ = behind_normal_lane_id;
  }
  void SetV2NextTurnInfo(const V2TurnInfo& v2_next_turn_info) {
    v2_next_turn_info_ = v2_next_turn_info;
  }
  void SetFunctionId(const byd::msg::planning::FunctionId& function_id) {
    function_id_ = function_id;
  }
  void AddFailReason(const std::string& fail_reason) {
    fail_reason_ = fail_reason;
  }
  void SetRoundaboutInfo(const RoundaboutInfo roundabout_info) {
    roundabout_info_ = roundabout_info;
  }
  void SetCurrentIndex(const int& current_index) {
    current_index_ = current_index;
  }
  void SetAllIndex(const int& all_index) { all_index_ = all_index; }

  const byd::msg::planning::FunctionId& GetFunctionId() const {
    return function_id_;
  }
  const std::string& GetFailReason() const { return fail_reason_; }
  const V2TurnInfo& GetV2NextTurnInfo() const { return v2_next_turn_info_; }
  const double GetDistanceToJunction() const { return distance_to_junction_; }
  const TurnType& GetJunctionType() const { return junction_type_; }
  const double GetDistanceToTurn() const { return distance_to_turn_; }
  const TurnType& GetTurnType() const { return turn_type_; }
  const double& GetRidedDistance() const { return rided_distance_; }
  const double& GetLeftBoundaryDistance() const {
    return left_boundary_distance_;
  }
  const double& GetRightBoundaryDistance() const {
    return right_boundary_distance_;
  }
  const double& GetThetaToEgo() const { return theta_to_ego_; }
  const std::string& GetFinalLaneId() const { return final_lane_id_; }
  const std::string& GetFirstJunctionId() const { return first_junction_id_; }
  const double& GetLCCountForJunction() const { return lc_count_for_junction_; }
  const double& GetLCCountForTurn() const { return lc_count_for_turn_; }
  const RoundaboutInfo& GetRoundaboutInfo() const { return roundabout_info_; }
  const int& GetCurrentIndex() const { return current_index_; }
  const int& GetAllIndex() const { return all_index_; }

  const double GetDistanceToNextJunction() const {
    return distance_to_next_junction_;
  }
  void SetSequenceIndex(const float& sequence_index) {
    sequence_index_ = sequence_index;
  }
  const double GetSequenceIndex() const { return sequence_index_; }
  void SetNaviDiffDistance(const float& navi_diff_distance) {
    navi_diff_distance_ = navi_diff_distance;
  }
  LaneConstPtr GetCurrentLanePtr() const { return current_lane_ptr_; }
  void SetCurrentLanePtr(LaneConstPtr current_lane_ptr) {
    current_lane_ptr_ = current_lane_ptr;
  }
  const double GetNaviDiffDistance() const { return navi_diff_distance_; }
  const bool IsPhysicalSplitCondition() const {
    return is_physical_split_condition_;
  }
  void SetIsPhysicalSplitCondition(bool is_physical_split_condition) {
    is_physical_split_condition_ = is_physical_split_condition;
  }
  const V2TurnInfo& GetPhysicalSplitTurn() const {
    return physical_split_turn_;
  }
  void SetPhysicalSplitTurn(const V2TurnInfo& physical_split_turn) {
    physical_split_turn_ = physical_split_turn;
  }
  const double GetEnvDistToJunctionInPlanningKeep() const {
    return env_dist_to_junction_for_planning_keep_;
  }
  void SetEnvDistToJunctionInPlanningKeep(double env_dist_to_junction) {
    env_dist_to_junction_for_planning_keep_ = env_dist_to_junction;
  }

  const int seq_num() const { return ego_info_.seq_num; }
  const double& stamp() const { return ego_info_.stamp; }
  const Point2d& position() const { return ego_info_.pos; }
  const Point2d& velocity() const { return ego_info_.vel; }
  const Point2d& acceleration() const { return ego_info_.acc; }
  const double& heading() const { return ego_info_.heading; }
  const double& steering_angle() const { return ego_info_.steering_angle; }
  const EgoInfo& ego_info() const { return ego_info_; }
  const std::string& current_lane_id() const { return current_lane_id_; }
  const double& length() const { return ego_info_.length; }
  const double& width() const { return ego_info_.width; }
  const double& height() const { return ego_info_.height; }
  const double speed() const { return ego_info_.vel.Length(); }
  const std::string& final_lane_id() const { return final_lane_id_; }
  const std::string& behind_normal_lane_id() const {
    return behind_normal_lane_id_;
  }

 private:
  EgoInfo ego_info_;
  std::string current_lane_id_;
  double distance_to_junction_ = 1000.0;
  double distance_to_turn_ = 1000.0;
  TurnType junction_type_ = NO_TURN;
  TurnType turn_type_ = NO_TURN;
  double rided_distance_ = 0.0;
  double left_boundary_distance_ = 0.0;
  double right_boundary_distance_ = 0.0;
  double theta_to_ego_ = 0.0;
  std::string final_lane_id_;
  float sequence_index_ = 0.0;
  float navi_diff_distance_ = 0.0;
  std::string first_junction_id_;
  double lc_count_for_junction_ = 0.0;
  double lc_count_for_turn_ = 0.0;
  std::string behind_normal_lane_id_;
  V2TurnInfo v2_next_turn_info_;
  std::string fail_reason_;
  LaneConstPtr current_lane_ptr_ = nullptr;
  RoundaboutInfo roundabout_info_;
  byd::msg::planning::FunctionId function_id_ =
      byd::msg::planning::FunctionId::FUNCTION_NONE;
  V2TurnInfo physical_split_turn_;
  double env_dist_to_junction_for_planning_keep_ = 1000.0;
  bool is_physical_split_condition_ = false;
  double distance_to_next_junction_ = 1000.0;
  int current_index_ = -1;
  int all_index_ = -1;
};

using VehicleStatePtr = std::shared_ptr<VehicleState>;
using VehicleStateConstPtr = std::shared_ptr<const VehicleState>;

}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_COMMON_VEHICLE_STATE_H
