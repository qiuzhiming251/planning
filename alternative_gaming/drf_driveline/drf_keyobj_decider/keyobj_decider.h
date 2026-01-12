#pragma once
#include "object_manager/spacetime_trajectory_manager.h"
#include "keyobj_data_structure.h"
#include <optional>
#include <array>
#include <vector>
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "plan_common/math/util.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "plan_common/path_approx.h"
#include "plan_common/util/vehicle_geometry_util.h"
#include "plan_common/path_approx_overlap.h"
#include "modules/cnoa_pnc/planning/proto/gaming.pb.h"
namespace st::planning {
class RiskFieldKeyobjDecider {
 public:
  RiskFieldKeyobjDecider(const VehicleGeometryParamsProto vehicle_geo_params)
      : vehicle_geo_params_(vehicle_geo_params){};
  ~RiskFieldKeyobjDecider() = default;

  void GetOppoKeyobjs(
      const SpacetimeTrajectoryManager* st_mgr,
      const DiscretizedPath* init_path, double ego_v,
      std::vector<RiskFieldKeyobj>* const oppo_straight_keyvehicles,
      std::vector<RiskFieldKeyobj>* const oppo_straight_keycyclists);

  void GetLeftTurnOutsideOncommingKeyobjs(
      const SpacetimeTrajectoryManager* st_mgr,
      const DiscretizedPath* init_path, double ego_v,
      std::vector<RiskFieldKeyobj>* const outside_risk_keyobjs);

  void GetOppoStaticKeyobjs(
      const SpacetimeTrajectoryManager* st_mgr,
      const DiscretizedPath* init_path, double ego_v,
      std::vector<RiskFieldKeyobj>* const oppo_static_keyvehicles,
      std::vector<RiskFieldKeyobj>* const oppo_static_keycyclists);

  void CalSameDirectLeftSideKeyobjs(
      const SpacetimeTrajectoryManager* st_mgr,
      const DiscretizedPath* init_path, double av_half_width,
      std::vector<RiskFieldKeyobj>* left_side_keyvehicles,
      std::vector<RiskFieldKeyobj>* left_side_keycyclists);

  void GetLeftTurnKeyobjs(
      const SpacetimeTrajectoryManager* st_mgr,
      const DiscretizedPath* init_path, double ego_v,
      const PathPoint start_point,
      const DrivelineResultProto* last_driveline_result,
      /*std::vector<RiskFieldKeyobj>* oppo_straight_keyvehicles,*/
      std::vector<RiskFieldKeyobj>* syn_left_turn_keyvehicles,
      std::vector<RiskFieldKeyobj>* syn_left_turn_keycyclists);

 private:
  bool CheckPoseInLeft(const Vec2d& start_point, const Vec2d& end_point,
                       const Vec2d& pose);

  bool IsObjectSameDirectWithEgo(const SpacetimeObjectTrajectory st_traj,
                                 const PathPoint start_point);

  bool IsObjectCrossWithAvPath(const SpacetimeObjectTrajectory st_traj,
                               const DiscretizedPath av_path);

  std::optional<RiskFieldCollisionArea> CalcCollisionAreaByPred(
      const SpacetimeObjectTrajectory& st_traj, const DiscretizedPath& av_path);

  std::optional<double> CalcAgentHwtByCollisionArea(
      const SpacetimeObjectTrajectory st_traj,
      const RiskFieldCollisionArea& collision_area);

  double CalcEgoHwtByCollisionArea(
      double ego_v, const RiskFieldCollisionArea& collision_area);

  // void CalculateExactVehicleHwt(std::vector<RiskFieldKeyobj>*
  // oppo_straight_keyvehicles);

  double GetCollisionLength(const SpacetimeObjectTrajectory& st_traj,
                            PathPoint& insert_point);

  double EstObjAcc(const SpacetimeObjectTrajectory st_traj);

  bool IsObjStart(const SpacetimeObjectTrajectory st_traj);

  bool IsObjStatic(const SpacetimeObjectTrajectory& st_traj);

  bool IsRightOfAvPath(const SpacetimeObjectTrajectory& st_traj,
                       const DiscretizedPath& av_path);

  bool IsLeftOfAvPath(const SpacetimeObjectTrajectory& st_traj,
                      const DiscretizedPath& av_path);

  bool IsFrontOfAvpath(const SpacetimeObjectTrajectory& st_traj,
                       const DiscretizedPath& av_path);

  bool IsTotallyLeftOfAvPath(const SpacetimeObjectTrajectory st_traj,
                             const DiscretizedPath av_path, double half_width);

  bool IsTotalLeftOfAv(const SpacetimeObjectTrajectory st_traj,
                       double av_half_width);

  bool IsOppoGoStraight(const SpacetimeObjectTrajectory& st_traj,
                        const DiscretizedPath& av_path);

  bool IsOppoStaticLeftAvPath(const SpacetimeObjectTrajectory& st_traj,
                              const DiscretizedPath& av_path);

  bool IsLeftTurnKeyObj(const SpacetimeObjectTrajectory st_traj,
                        const DiscretizedPath& av_path,
                        const PathPoint start_point);

  bool IsVehicle(const SpacetimeObjectTrajectory st_traj);

  bool IsPedestrian(const SpacetimeObjectTrajectory st_traj);

  bool IsLargeVehicle(const SpacetimeObjectTrajectory st_traj);

  bool IsCyclist(const SpacetimeObjectTrajectory st_traj);

  std::optional<PathPoint> GetCollisionPoint(
      const SpacetimeObjectTrajectory& st_traj, const DiscretizedPath& av_path);

  std::optional<std::pair<double, double>> ConvertToOverlapRange(
      absl::Span<const AgentOverlap> agent_overlaps);

  const VehicleGeometryParamsProto vehicle_geo_params_;
};
}  // namespace st::planning
