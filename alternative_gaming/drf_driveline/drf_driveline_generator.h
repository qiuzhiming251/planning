/**
 * @file drf_driveline_generator.h
 * @author zhang.xing (zhang.xing43@byd.com)
 * @brief
 * @version 0.1
 * @date 2025-06-20
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once
#include "plan_common/util/status_macros.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "plan_common/planning_macros.h"
#include "plan_common/plan_start_point_info.h"
#include "vehicle_state.h"
#include "drf_keyobj_decider/keyobj_decider.h"
#include "risk_field_trigger/drf_trigger.h"
#include "pure_pursuit/go_point_pursuit.h"
#include "risk_field.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/object_history.h"
#include "modules/cnoa_pnc/planning/proto/gaming.pb.h"

namespace st::planning {
struct DRFDrivelineInput {
  const SpacetimeTrajectoryManager* traj_mgr = nullptr;  //障碍物
  const DiscretizedPath* init_path = nullptr;         // initializer输出轨迹
  const ObjectHistoryManager* obs_history = nullptr;  //障碍物历史结果
  const StPathPlanStartPointInfo* plan_start_point = nullptr;
  int plan_id;
  const DrivePassage* drive_passage = nullptr;
  const DrivelineResultProto* last_driveline_result = nullptr;
};
struct DRFDrivelineOutput {
  std::vector<ApolloTrajectoryPointProto> dynamic_drive_line;
  DrivelineResultProto driveline_result;
};

class DRFDynamicDrivelineGenerator {
 public:
  DRFDynamicDrivelineGenerator(
      const VehicleGeometryParamsProto* vehicle_geo_params)
      : vehicle_geo_params_(vehicle_geo_params){};
  ~DRFDynamicDrivelineGenerator() = default;
  /**
   * @brief 外部调用主函数
   *
   */
  absl::StatusOr<DRFDrivelineOutput> RunRiskFieldDrivelineGenerator(
      const DRFDrivelineInput& input);
  const std::string& get_debug() const { return debug_; };

 private:
  void TrajPointToAvState(const ApolloTrajectoryPointProto& point,
                          VehicleState* av_state);

  bool GenerateRiskFieldDriveline(
      const VehicleState& av_state, const RoadInfo& road_info,
      const DiscretizedPath& init_path,
      const std::vector<RiskFieldKeyobj>& syn_leftturn_keyobjs,
      const std::vector<RiskFieldKeyobj>& oppo_straight_keyobjs,
      const std::unordered_set<std::string>& yield_obj_ids,
      const std::unordered_set<std::string>& oppo_yield_obj_ids,
      std::string* most_risk_obj_id,
      std::vector<ApolloTrajectoryPointProto>* const rf_driveline,
      std::vector<Vec2d>* const rf_driveline_keypoints, std::string& debug);

  std::optional<std::vector<std::pair<double, const RiskFieldKeyobj*>>>
  ChoseMostRiskKeyobjs(const std::vector<RiskFieldKeyobj>& keyobjs,
                       const VehicleState& av_state,
                       const std::unordered_set<std::string>& yield_obj_ids,
                       const ObjectDecisionType& obj_lat_decision,
                       std::string& debug);

  ObjectDecisionType CalcGopointEnvRiskLatDecision(const RoadInfo& road_info);

  ObjectDecisionType CalcStartpointEnvRiskLatDecision(
      const RoadInfo& road_info);

  VehicleAction CalcActionByPurePursuit(const VehicleState& vehicle_state,
                                        const DiscretizedPath& path,
                                        std::string* debug);

  double CalcTargetJ(double a, double target_a, double virtual_time);

  ActionRange CalcActionRangeForEnvRisk(
      const Risk& enm_risk, const VehicleState& av_state,
      const VehicleAction& pure_pursuit_action,
      const ObjectDecisionType& enm_lateral_decision, double target_a,
      std::string* debug);

  void AddLateralJerkConstraint(const VehicleState& av_state,
                                bool need_fast_rtc, double* dkappa);

  double CalcTargetDkappa(const VehicleState& av_state, double target_kappa,
                          double virtual_time);

  ActionRange CalcActionRangeForLongitudinalCrossYield(
      const Risk& lon_risk, double go_point_risk_val,
      const VehicleState& av_state, const AgentPoseInfo& agent_pose_info,
      double agent_speed, const VehicleAction& PurePursuitAction,
      const ObjectDecisionType& obj_lateral_decision,
      const ObjBehavior& obj_behavior,
      const PiecewiseLinearFunction<double>& min_acc_plf,
      double look_forward_kappa, std::string* debug);

  double GetCompensateTargetAcc(const ObjectDecisionType& obj_lateral_decision,
                                const ObjBehavior& obj_behavior);

  // double GetLookForwardCompensateTargetAcc(const ObjectDecisionType&
  // obj_lateral_decision,const ObjBehavior& obj_behavior);

  ActionRange CalcActionRangeForLateralRisk(
      const Risk& lat_risk, double go_point_risk_val,
      const VehicleState& av_state, const AgentPoseInfo& agent_pose_info,
      double agent_speed, const VehicleAction& PurePursuitAction,
      const ObjectDecisionType& obj_lateral_decision,
      const ObjBehavior& obj_behavior, std::string* debug);

  ActionRange CalcActionRangeForLookForwardLongitudinalCrossYield(
      const Risk& lon_risk, double go_point_risk_val,
      const VehicleState& av_state, const AgentPoseInfo& agent_pose_info,
      double agent_speed, const VehicleAction& PurePursuitAction,
      const ObjectDecisionType& obj_lateral_decision,
      const ObjBehavior& obj_behavior,
      const PiecewiseLinearFunction<double>& min_acc_plf,
      double look_forward_kappa, std::string* debug);

  ActionRange AdjustActionByEnvRiskActionRange(
      const Risk& enm_risk, const ObjectDecisionType& enm_lat_decision,
      const ActionRange& action_range,
      const ActionRange& enm_risk_action_range);

  VehicleAction AdjustActionByRiskActionRange(const VehicleAction& action,
                                              const ActionRange& action_range);

  void AddLateralAccConstraint(const VehicleState& av_state,
                               VehicleAction& vehicle_action);

  VehicleState CalcNextVehicleState(const VehicleState& vehicle_state,
                                    const VehicleAction& vehicle_action);

  bool IsPointBackWard(const VehicleState& av_state, double x, double y);

  void ConnectOcpDriveline(const DiscretizedPath& ocp_driveline,
                           std::vector<VehicleState>* rf_traj);

  void CalcRightNudgeDrivelineKeypoints(
      const std::vector<VehicleState>& traj,
      std::vector<Vec2d>* rf_driveline_keypoints, double start_point_heading);

  void TrajToDriveline(const std::vector<VehicleState>& traj,
                       std::vector<ApolloTrajectoryPointProto>* rf_driveline);

  bool IsRoadInfoValid(const RoadInfo& road_info, const double& ego_heading);

  ActionRange CalcActionRangeForOneAgent(
      const RoadInfo& road_info, const Box2d& av_box,
      const VehicleState& av_state, const VehicleAction& PurePursuitAction,
      const Box2d& agent_box, double agent_speed,
      const ObjBehavior& obj_behavior,
      const ObjectDecisionType& obj_lateral_decision,
      const ObjectDecisionType& go_point_lat_decision,
      const ObjectDecisionType& start_point_lat_decision, double cur_simu_time,
      std::string* debug);

  void AddFrictionCircleConstraint(const VehicleState& av_state,
                                   VehicleAction& vehicle_action);

  void ReSampleDriveline(std::vector<ApolloTrajectoryPointProto>& dr_line,
                         double dt);

  void RefinePredTraj(const SpacetimeTrajectoryManager& origin_traj_mgr,
                      SpacetimeTrajectoryManager* const modified_traj_mgr);

 private:
  std::string debug_;
  // double left_nudge_compensate_acc_;
  double sync_right_nudge_compensate_acc_;
  double oppo_right_nudge_compensate_acc_;
  double sync_left_nudge_compensate_acc_;
  double oppo_left_nudge_compensate_acc_;
  RiskFieldTrigger drf_trigger_;
  const VehicleGeometryParamsProto* vehicle_geo_params_;
  double enm_risk_virtual_time_;
  bool use_longitude_risk_;
  bool use_lookforward_risk_;
  bool use_lat_virtual_time_;
};

}  // namespace st::planning