

#ifndef ST_PLANNING_SCHEDULER_SCHEDULER_OUTPUT
#define ST_PLANNING_SCHEDULER_SCHEDULER_OUTPUT

#include <tuple>
#include <limits>
#include <string>
#include <vector>
#include <utility>
#include <optional>

#include "absl/hash/hash.h"

#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/turn_signal.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "plan_common/type_def.h"
#include "plan_common/drive_passage.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/math/frenet_frame.h"

#include "object_manager/spacetime_trajectory_manager.h"

namespace st::planning {

struct LaneObsInfo {
  std::string id;
  FrenetBox frenet_box;
  double obs_v;
};

struct SchedulerOutput {
  bool is_fallback = false;
  bool is_expert = false;
  // TODO: Move drive_passage.h to scheduler folder.
  DrivePassage drive_passage{};
  // l-s boundaries on drive passage center.
  PathSlBoundary sl_boundary{};
  // Should only be updated in async high-freq module and remains 0.0 otherwise.
  double target_offset_from_start = 0.0;
  LaneChangeStateProto lane_change_state{};
  ad_byd::planning::LcReason lc_reason = ad_byd::planning::LC_REASON_NONE;
  // For constraints on lane change pause stage, empty if not changing lane.
  mapping::LanePath lane_path_before_lc{};
  double length_along_route = std::numeric_limits<double>::max();
  double max_reach_length =
      std::numeric_limits<double>::max();  // current lane seq navi dist
  int lc_num = 0;                          // current lane seq navi lc num
  std::string leading_id = "";
  double standard_congestion_factor = 0.0;
  double traffic_congestion_factor = 0.0;
  bool should_smooth = false;
  bool borrow_lane = false;
  FrenetBox av_frenet_box_on_drive_passage{};
  // Whether there is enough distance to lane change.
  bool request_help_lane_change_by_route = false;
  bool switch_alternate_route = false;
  TurnSignal planner_turn_signal = TURN_SIGNAL_NONE;
  TurnSignalReason turn_signal_reason = TURN_SIGNAL_OFF;
  bool miss_navi_scenario = false;

  using HashType = std::tuple<mapping::ElementId, bool, bool>;
  HashType Hash() const {
    return std::make_tuple(drive_passage.lane_path().front().lane_id(),
                           borrow_lane, is_fallback);
  }
  PausePushSavedOffsetProto saved_offset{};
  std::map<double, LaneObsInfo> lane_obstacles_info{};
  // TODO: add constraints

  template <typename Archive>
  friend void serialize(Archive& ar, SchedulerOutput& scheduler_output);
};

struct RouteTargetInfo {
  int plan_id = 0;
  KdTreeFrenetFrame frenet_frame;
  FrenetBox ego_frenet_box{};

  DrivePassage drive_passage{};
  PathSlBoundary sl_boundary{};
  SpacetimeTrajectoryManager st_traj_mgr{};

  template <typename Archive>
  friend void serialize(Archive& ar, RouteTargetInfo& route_target_info);
};

}  // namespace st::planning

#endif  // ST_PLANNING_SCHEDULER_SCHEDULER_OUTPUT
