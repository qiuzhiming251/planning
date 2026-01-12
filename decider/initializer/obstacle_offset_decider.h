#include <cmath>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "object_manager/object_history.h"
#include "object_manager/spacetime_planner_object_trajectories.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/drive_passage.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/util/decision_info.h"

#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st::planning {

struct ObstacleInfo {
  enum OffsetType : int {
    OffsetIgnore = 0,
    OffsetNudge,
    OffsetNotice,
    OffsetBorrow,
    OffsetBlock
  };
  bool valid = false;
  std::string id = "";
  double v = 0.0;
  double offset = 0.0;
  OffsetType offset_type = OffsetIgnore;
  double s_max = 0.0;
  double s_min = 0.0;
  double center_s = 0.0;
  double l_max = 0.0;
  double l_min = 0.0;
  double center_l = 0.0;
  // double lat_speed = 0.0;
  double length = 0.0;
  double width = 0.0;
  double safe_lat_dist = 0.0;
  double lat_dist = 0.0;
  double ds = 0.0;
  double dl = 0.0;
  double vs = 0.0;
  double vl = 0.0;
  bool emergency_nudge = false;
  bool is_stationary = false;
  ObjectType type = OT_UNKNOWN_STATIC;
};

struct HistoricalNudgeInfo {
  double nudge_offset;
  std::string nudge_obj_id;
  ObjectType obs_type;
};

class ObstacleOffsetDecider {
 public:
  ObstacleOffsetDecider() = default;

  absl::StatusOr<bool> OffsetDecide(
      const DrivePassage& drive_passage,
      const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
      const SpacetimeTrajectoryManager& st_traj_mgr,
      const VehicleGeometryParamsProto& vehicle_geometry,
      const std::vector<LeadingGroup>& leading_groups,
      const FrenetCoordinate& ego_sl, const FrenetBox& ego_box,
      const ApolloTrajectoryPointProto& plan_start_point,
      LargeVehicleAvoidStateProto* cur_large_vehicle_avoid_state,
      PathSlBoundary* path_sl_boundary, const LaneChangeStage& prev_lc_stage,
      bool lc_left, const ObjectHistoryManager* obs_history);

 private:
  bool IsVRUType(ObjectType type) const;
  bool IsLaneChange(const LaneChangeStage& prev_lc_stage,
                    LargeVehicleAvoidStateProto* cur_large_vehicle_avoid_state,
                    bool* lc_left);

  double CalculateAverageCurvature(const DrivePassage& drive_passage,
                                   const FrenetCoordinate& ego_sl,
                                   const double& ego_v) const;

  void GetOffsetLimit(const DrivePassage& drive_passage,
                      const FrenetCoordinate& ego_sl,
                      const HistoricalNudgeInfo& historical_nudge_info,
                      const double& ego_half_width, const double& ego_v,
                      double* offset_min, double* offset_max,
                      double* range_min_lane, double* range_max_lane);

  ObstacleInfo GetOffsetInfo(const DrivePassage& drive_passage,
                             const SpacetimeObjectTrajectory& traj,
                             const HistoricalNudgeInfo& historical_nudge_info,
                             const ObjectHistoryManager* obs_history,
                             const FrenetBox& obs_frenet_box,
                             const FrenetCoordinate& ego_sl,
                             const FrenetBox& ego_box, const double& nudge_min,
                             const double& nudge_max, double ego_width,
                             double ego_v, bool is_lane_change, bool lc_left);

  double GetObstaclePredLatSpeed(
      const DrivePassage& drive_passage,
      const prediction::PredictedTrajectory& pred_traj, double preview_time);

  double GetObstacleCurLatSpeed(const DrivePassage& drive_passage,
                                const double speed, const double theta,
                                const double s);
  void GetObstacleLongLatSpeed(const DrivePassage& drive_passage,
                               const SpacetimeObjectTrajectory& traj,
                               ObstacleInfo* obstacle_info);

  bool CheckObstacleOutsideCurb(const DrivePassage& drive_passage,
                                const FrenetBox& obs_box);

  void UpdateNudgeCandidate(const DrivePassage& drive_passage,
                            ObstacleInfo& obstacle_info,
                            const SpacetimeObjectTrajectory& traj,
                            const std::vector<LeadingGroup>& leading_groups,
                            const HistoricalNudgeInfo& historical_nudge_info,
                            const ApolloTrajectoryPointProto& plan_start_point,
                            const FrenetCoordinate& ego_sl,
                            const FrenetBox& ego_box, const bool is_lane_change,
                            const bool lc_left,
                            std::vector<ObstacleInfo>* candidate_nudge_left,
                            std::vector<ObstacleInfo>* candidate_nudge_right);

  void AvoidObstacleFilter(
      const DrivePassage& drive_passage,
      const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
      const SpacetimeTrajectoryManager& st_traj_mgr,
      const std::vector<LeadingGroup>& leading_groups,
      const FrenetCoordinate& ego_sl,
      const HistoricalNudgeInfo& historical_nudge_info,
      const ObjectHistoryManager* obs_history, const FrenetBox& ego_box,
      const ApolloTrajectoryPointProto& plan_start_point,
      const double ego_half_width, const bool is_lane_change,
      const bool lc_left, const double nudge_min, const double nudge_max,
      double* block_s, double* block_v,
      std::vector<ObstacleInfo>* candidate_nudge_left,
      std::vector<ObstacleInfo>* candidate_nudge_right,
      std::vector<ObstacleInfo>* all_obs_info);

  HistoricalNudgeInfo GetHistoricalNudgeInfo(
      LargeVehicleAvoidStateProto* cur_large_vehicle_avoid_state);

  void DecideAvoidSituation(
      const DrivePassage& drive_passage, const FrenetCoordinate& ego_sl,
      const HistoricalNudgeInfo& historical_nudge_info,
      const FrenetBox& ego_box,
      const ApolloTrajectoryPointProto& plan_start_point,
      const double ego_half_width, const bool is_lane_change,
      const double nudge_min, const double nudge_max, const double& block_s,
      const double& block_v,
      const std::vector<ObstacleInfo>& candidate_nudge_left,
      const std::vector<ObstacleInfo>& candidate_nudge_right,
      const std::vector<ObstacleInfo>& all_obs_info,
      ObstacleInfo* selected_obstacle);

  bool IsFarAwayRoadBoundary(const DrivePassage& drive_passage,
                             const FrenetCoordinate& ego_sl,
                             const ApolloTrajectoryPointProto& plan_start_point,
                             double half_ego_width, bool is_left,
                             ObstacleInfo* avoid_obstacle);

  bool GetObstacleMeanLOfHistory(const DrivePassage& drive_passage,
                                 const ObjectHistoryManager* obs_history,
                                 const SpacetimeObjectTrajectory& obj_traj,
                                 double* center_l, double* box_l_min,
                                 double* box_l_max, double* lat_speed);

  void UpdateAvoidState(
      const ObstacleInfo& selected_obstacle, const FrenetCoordinate& ego_sl,
      LargeVehicleAvoidStateProto* cur_large_vehicle_avoid_state,
      const HistoricalNudgeInfo& historical_nudge_info,
      double* executing_avoid_dist, bool is_lane_change);

  double RoundToNearest(double value, double last_value) {
    if (std::abs(value) < kEpsilon) {
      return 0.0;
    }
    if (std::abs(value - last_value) < kHysteresisThreshold) {
      return last_value;
    }
    double sign = value / std::abs(value);
    value = std::abs(value);
    const double step = 0.2;
    int index = static_cast<int>(std::floor((value + kEpsilon) / step));
    index = std::min(index, 3);  // 最大值限制在0.6
    index = std::max(index, 0);
    return sign * index * step;
  }

  bool is_collision_free(const DrivePassage& drive_passage,
                         const std::vector<ObstacleInfo>& all_obs_info,
                         const FrenetBox& ego_box,
                         const HistoricalNudgeInfo& historical_nudge_info,
                         double ego_vs, double half_ego_width,
                         ObstacleInfo* selected_obstacle);

  std::pair<int, std::vector<double>> cur_saved_dist_ =
      std::make_pair(0, std::vector<double>(kMaxDistSavingNum, 0));
  double debug_avg_translate_l_ = 0.0;  // Debug variable
  double debug_translate_l_ = 0.0;      // Debug variable

 private:
  // veh filter thiesholds
  static constexpr double SROIMaxHi = 150.0;
  static constexpr double SROIMaxLow = 40.0;
  static constexpr double SROIMin = -50.0;
  static constexpr double SROITime = 10.0;
  // epsilon value
  static constexpr double kEpsilon = 1e-7;
  // average curvature calculate constants
  static constexpr double kBackCheckCurveDistance = 20.0;
  static constexpr double kCalculateCurvatureStep = 4.0;
  static constexpr double kMinPreviewTime = 5.0;
  static constexpr double kMinPreviewDist = 60.0;
  // offset limit calculate constants
  static constexpr double kDefaultLaneWidth = 3.5;  // m
  static constexpr double kSmallCurvatureThreshold = 0.0008;
  static constexpr double kLaneNearCurbThreshold = 0.5;
  static constexpr double kNearCurbNudgeOffsetMax = 0.4;
  static constexpr double kNudgeOffsetMax = 0.6;
  static constexpr double kOffsetLimitMaxPreviewDist = 60.0;
  static constexpr int64_t kMaxUsingHistoryLengthUs = 1e6;
  // GetOffsetInfo constants
  static constexpr double NudgeEnableLatDist = 0.1;
  static constexpr double LargeTruckLengthThreshold = 5;

  // output value stability control constants
  static constexpr int kMaxDistSavingNum = 10;
  static constexpr double MaxAvoidLateralMoveOffset = 0.13;
  static constexpr double MaxEndAvoidLateralMoveOffset = 0.1;
  static constexpr double kHysteresisThreshold = 0.1;
  static constexpr int kInvalidInputCntThreshold = 10;
  static constexpr int kAvoidEndSmoothMaxCnt = 0;
  // cutin/out
  static constexpr double CutInOutlatSpeed = 0.45;
  //
  static constexpr int kLCStateHoldNum = 10;
};

}  // namespace st::planning