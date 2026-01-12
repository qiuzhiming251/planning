

#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_PATH_TIME_CORRIDOR_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_PATH_TIME_CORRIDOR_H_

#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "plan_common/path_sl_boundary.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_planner_object_trajectories.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/trajectory_point.h"
#include "plan_common/drive_passage.h"

namespace st {
namespace planning {

namespace optimizer {

class PathTimeCorridor {
 public:
  struct BoundaryInfo {
    enum Type {
      CURB = 0,
      LANE_BOUNDARY = 1,
      VRU = 2,
      VEHICLE = 3,
      LARGE_VEHICLE = 4,
      STATIC = 5,
    };
    Type type;
    double l_outer_boundary;  // sl_boundary.l
    double l_boundary;        // target_sl_boundary.l
    double l_outer_object;    // min(sl_boundary.l, l_object)
    double l_object;          // min(target_sl_boundary.l, l_object)
    double l_curb;            // min(10.0, l_curb)
    const SpacetimeObjectTrajectory* object_ptr = nullptr;

    static Type GetType(StationBoundaryType type) {
      switch (type) {
        case StationBoundaryType::BROKEN_WHITE:
        case StationBoundaryType::SOLID_WHITE:
        case StationBoundaryType::BROKEN_YELLOW:
        case StationBoundaryType::SOLID_YELLOW:
        case StationBoundaryType::SOLID_DOUBLE_YELLOW:
        case StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE:
        case StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE:
        case StationBoundaryType::VIRTUAL_LANE:
        case StationBoundaryType::UNKNOWN_TYPE:
          return Type::LANE_BOUNDARY;
          break;
        case StationBoundaryType::CURB:
        case StationBoundaryType::VIRTUAL_CURB:
          return Type::CURB;
          break;
        default:
          throw std::runtime_error("switch case on enum unexpected");
      }
    }

    static Type GetObjectType(const ObjectType& object_type) {
      switch (object_type) {
        case ObjectType::OT_MOTORCYCLIST:
        case ObjectType::OT_PEDESTRIAN:
        case ObjectType::OT_CYCLIST:
        case ObjectType::OT_TRICYCLIST:
          return Type::VRU;
          break;
        case ObjectType::OT_VEHICLE:
          return Type::VEHICLE;
          break;
        case ObjectType::OT_LARGE_VEHICLE:
          return Type::LARGE_VEHICLE;
          break;
        case ObjectType::OT_UNKNOWN_STATIC:
        case ObjectType::OT_FOD:
        case ObjectType::OT_UNKNOWN_MOVABLE:
        case ObjectType::OT_VEGETATION:
        case ObjectType::OT_BARRIER:
        case ObjectType::OT_CONE:
        case ObjectType::OT_WARNING_TRIANGLE:
          return Type::STATIC;
          break;
        default:
          throw std::runtime_error("switch object case on enum unexpected");
      }
    }
  };
  enum PositionType {
    POSITION_UNKNOWN = 0,
    POSITION_LEFT = 1,
    POSITION_RIGHT = 2
  };
  struct ObjectPositionInfo {
    bool is_static;
    double check_time;
    double object_s;
    double object_l;
    double init_object_s;
    double init_object_l;
    PositionType position_type;
  };
  using ObjectPositionInfos =
      std::unordered_map<std::string, std::vector<ObjectPositionInfo>>;

  PathTimeCorridor(const DrivePassage* drive_passage,
                   const PathSlBoundary* path_sl_boundary,
                   std::vector<std::vector<BoundaryInfo>> left_boundary,
                   std::vector<std::vector<BoundaryInfo>> right_boundary,
                   std::vector<int> time_indices,
                   ObjectPositionInfos object_position_infos);

  std::pair<BoundaryInfo, BoundaryInfo> QueryBoundaryL(double s,
                                                       double t) const;
  /// @brief Query a vector of object position info or empty vector if not found
  std::vector<ObjectPositionInfo> QueryObjectPositionInfo(
      std::string object_id) const;

 private:
  const DrivePassage* drive_passage_;
  const PathSlBoundary* path_sl_boundary_;
  std::vector<std::vector<BoundaryInfo>> left_boundary_;
  std::vector<std::vector<BoundaryInfo>> right_boundary_;
  std::vector<int> time_indices_;
  ObjectPositionInfos object_position_infos_;
};

absl::StatusOr<PathTimeCorridor> BuildPathTimeCorridor(
    const int plan_id, std::string_view base_name,
    const std::vector<TrajectoryPoint>& init_traj,
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const std::map<std::string, ConstraintProto::LeadingObjectProto>&
        leading_trajs,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const SpacetimePlannerObjectTrajectories& st_planner_object_traj,
    const VehicleGeometryParamsProto& veh_geo_params,
    const double trajectory_time_step);

}  // namespace optimizer
}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_PATH_TIME_CORRIDOR_H_
