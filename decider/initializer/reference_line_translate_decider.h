#include <cmath>
#include <string>
#include <utility>
#include <vector>
#include <numeric>

#include "absl/status/statusor.h"
#include "object_manager/spacetime_planner_object_trajectories.h"
#include "plan_common/drive_passage.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/util/decision_info.h"

#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st::planning {

struct SimpleObstacle {
  enum VehicleAttribute : int {
    NoCrossing = 0,
    LeftSlightCrossing,
    LeftCrossing,
    RightSlightCrossing,
    RightCrossing,
    LargeVehicleNoSpecialTreat,
    StationaryVehicle
  };
  std::string id;
  FrenetBox frenet_box;
  double speed = 0.0;
  VehicleAttribute vehicle_attribute = LargeVehicleNoSpecialTreat;

  SimpleObstacle(
      const std::string& _id = "", const FrenetBox& _box = FrenetBox(),
      double _speed = 0.0,
      VehicleAttribute _vehicle_attribute = LargeVehicleNoSpecialTreat)
      : id(_id),
        frenet_box(_box),
        speed(_speed),
        vehicle_attribute(_vehicle_attribute) {}
};

class ReferenceLineTranslateDecider {
 public:
  ReferenceLineTranslateDecider() = default;

  absl::StatusOr<bool> Decide(
      const DrivePassage& drive_passage,
      const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
      const VehicleGeometryParamsProto& vehicle_geometry,
      const std::vector<LeadingGroup>& leading_groups,
      const FrenetCoordinate& ego_sl, const FrenetBox& ego_box,
      const double& ego_v,
      LargeVehicleAvoidStateProto* cur_large_vehicle_avoid_state,
      PathSlBoundary* path_sl_boundary, const LaneChangeStage& prev_lc_stage,
      const bool lc_left);

 private:
  constexpr static double kTranslateCenteLineBuffer = 2.2;
  constexpr static double kSmallVehicleCrosslineTranslateCenteLineBuffer = 0.7;
  constexpr static double kMinDistForInAdvanceAvoid = 20.0;
  constexpr static double kMaxDistForInAdvanceAvoid = 35.0;
  constexpr static double kDistForBehindAvoid = 5.0;
  constexpr static double kLevelSignificant = 0.30;
  constexpr static double kLevelSlight = 0.15;
  constexpr static double kConsiderDistance = 100.0;
  constexpr static double kLengthOfGap = 40.0;
  constexpr static int kSuccessiveCountThr = 4;
  constexpr static double kEpsilon = 1e-7;
  constexpr static double kDefaultLaneWidth = 3.5;  // m
  constexpr static double kDefaultHalfLaneWidth = 0.5 * kDefaultLaneWidth;
  constexpr static int kMaxUsedDistMemory = 10;
  constexpr static int kMaxAvoidLastTime = 3000;
  constexpr static int kMaxDistSavingNum = 10;
  constexpr static double kBackCheckCurveDistance = 20.0;
  constexpr static double kCalculateCurvatureStep = 4.0;
  constexpr static double kMinPreviewTime = 5.0;
  constexpr static double kMinPreviewDist = 60.0;
  constexpr static double kExtraDistForInAdvanceAvoidWhenAvoiding = 15.0;  // m
  constexpr static double kSmallCurvatureThreshold = 0.0008;
  constexpr static double kLargeCurvatureThreshold = 0.002;
  constexpr static double kLCExcutingLargeVehNearThreshold = 10.0;
  constexpr static double kCrossLineSlightThreshold = 0.05;
  constexpr static double kCrossLineLateralVelThreshold = 0.2;
  constexpr static double kMaxLateralMoveOffset = 0.15;
  constexpr static double kMaxLateralOffsetThdNearAvoidEnd = 0.15;
  constexpr static int kMaxAvoidEndCnt = 10;

  std::pair<int, std::vector<double>> cur_saved_dist_ =
      std::make_pair(0, std::vector<double>(kMaxDistSavingNum, 0));
  double left_min_obj_abs_dist_ = std::numeric_limits<double>::max();
  double right_min_obj_abs_dist_ = std::numeric_limits<double>::max();

  void UpdateSavedDist(
      LargeVehicleAvoidStateProto* cur_large_vehicle_avoid_state);

  bool IsLaneChange(const LaneChangeStage& prev_lc_stage) const;

  bool IsAvoiding(
      LargeVehicleAvoidStateProto* cur_large_vehicle_avoid_state) const;

  bool IsLCExcutingLargeVehNearCond(const bool& lc_left,
                                    const LaneChangeStage& prev_lc_stage) const;

  double CalculateAverageCurvature(const DrivePassage& drive_passage,
                                   const FrenetCoordinate& ego_sl,
                                   const double& ego_v) const;

  std::tuple<double, LargeVehicleAvoidStateProto::AvoidDir,
             LargeVehicleAvoidStateProto::AvoidStage>
  HysteresisControl(double l, bool is_avoiding,
                    LargeVehicleAvoidStateProto::AvoidStage prev_avoid_stage,
                    double average_curvature) const;

  SimpleObstacle::VehicleAttribute GetVehicleAttribute(
      const DrivePassage& drive_passage, const SpacetimeObjectTrajectory& traj,
      const double& ego_v, const FrenetBox& ego_box, const FrenetBox& obs_box,
      const bool& is_lane_change, std::string& cut_in_obj_id) const;

  double CalculateMaxTranslateL(const std::vector<SimpleObstacle>& side_vector,
                                const FrenetBox& ego_box, const double& ego_v,
                                const double& half_av_width,
                                const std::string& cur_saved_obstacle_id,
                                const bool& lc_excute_large_veh_near_cond,
                                const double& dist_for_inadvance_avoid,
                                double& cur_avoid_obs_s,
                                SimpleObstacle& cur_avoid_obstacle,
                                int& nearest_cross_line,
                                bool is_left_side) const;
};

}  // namespace st::planning
