

#ifndef ONBOARD_PLANNER_INITIALIZER_INITIALIZER_OUTPUT_H_
#define ONBOARD_PLANNER_INITIALIZER_INITIALIZER_OUTPUT_H_

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "decider/initializer/cost_provider.h"
#include "decider/initializer/motion_searcher_defs.h"
#include "decider/initializer/motion_graph.h"
#include "decider/initializer/motion_state.h"
#include "decider/initializer/select_nudge_object.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
//#include "plan_common/util/hmi_content_util.h"
//#include "object_manager/planner_object.h"
namespace st::planning {

struct MotionSearchOutput {
  absl::Status result_status = absl::OkStatus();
  // nudge _info
  NudgeInfos nudge_info{};
  // Only has value on performing lane change and safety confirmed.
  absl::flat_hash_set<std::string> follower_set{};
  absl::flat_hash_set<std::string> leader_set{};
  double follower_max_decel = 0.0;
  absl::flat_hash_set<std::string> unsafe_object_ids{};

  bool is_lc_pause = false;
  // TODO: Add returning a list of motion forms and Edge indices.
  std::vector<ApolloTrajectoryPointProto> traj_points{};
  MotionEdgeIndex best_last_edge_index{};

  struct SearchCost {
    std::vector<double> feature_cost{};  // Accumulated feature cost
    double cost_to_come = 0.0;
    double TotalCost() const { return cost_to_come; }
  };

  MotionEdgeVector<SearchCost> search_costs{};
  double min_cost = 0.0;
  std::map<std::string, ConstraintProto::LeadingObjectProto> leading_trajs{};

  std::unique_ptr<MotionGraph> motion_graph = nullptr;

  std::unique_ptr<RefSpeedTable> ref_speed_table = nullptr;

  std::unique_ptr<CostProvider> cost_provider = nullptr;

  // Debug only (FLAGS_planner_initializer_debug_level >= 1)
  // Optimal trajectory (among all the leading objs)
  std::vector<MotionEdgeIndex> terminated_edge_idxes{};
  std::vector<std::vector<ApolloTrajectoryPointProto>> top_k_trajs{};
  std::vector<double> top_k_total_costs{};
  std::vector<MotionEdgeIndex> top_k_edges{};

  // Trajectories with multiple leading objects.
  struct MultiTrajCandidate {
    std::vector<ApolloTrajectoryPointProto> trajectory{};
    std::vector<std::string> leading_traj_ids{};
    double total_cost = 0.0;
    MotionEdgeIndex last_edge_index{};
    std::vector<double> feature_costs{};
    double final_cost = 0.0;
    IgnoreTrajMap ignored_trajs{};
  };
  std::vector<MultiTrajCandidate> multi_traj_candidates{};

  // Data Dumping only
  struct IsFilteredReasons {
    bool is_out_of_bound = false;
    bool is_violating_stop_constraint = false;
    bool is_dynamic_collision = false;
    bool is_violating_leading_objects = false;
  };
  struct TrajectoryEvaluationDumping {
    double weighted_total_cost =
        0.0;  // weighted by onboard initializer's weights
    std::vector<double> dumped_weights{};  // onboard initializer's weights
    std::vector<double> feature_costs{};   // unweighted
    std::vector<ApolloTrajectoryPointProto> traj{};
    IsFilteredReasons is_filtered_reasons{};
  };
  TrajectoryEvaluationDumping expert_evaluation{};
  std::vector<TrajectoryEvaluationDumping> candidates_evaluation{};
  SpeedResponseStyle speed_response_style = SpeedResponseStyle::SPEED_RESPONSE_CONSERVATIVE;
  PlannerStatusProto::PlannerStatusCode lc_status_code = PlannerStatusProto::OK;
  // Lane change style decider output
  LaneChangeStyleDeciderResultProto lc_style_decider_result{};
  // Task evaluation result
  TaskSafetyEvaluationProto task_safety_evaluation_result{};
  // Frames result for the scene of cones riding line
  int scene_cones_riding_line_frames_result = 0;
  absl::flat_hash_set<std::string> gaming_lc_obs_set{};
};

struct ReferenceLineSearcherOutput {
  struct NodeInfo {
    double min_cost = std::numeric_limits<double>::infinity();
    GeometryEdgeIndex outgoing_edge_idx =
        GeometryEdgeVector<GeometryEdge>::kInvalidIndex;
    GeometryNodeIndex prev_node_idx =
        GeometryNodeVector<GeometryNode>::kInvalidIndex;
  };

  struct EdgeInfo {
    std::vector<double> feature_costs{};
    double sum_cost = 0.0;
  };

  std::vector<GeometryNodeIndex> nodes_list{};
  std::vector<GeometryEdgeIndex> edges_list{};
  std::vector<Vec2d> ref_line_points{};
  double total_cost = 0.0;
  std::vector<double> feature_costs{};

  std::unique_ptr<RefLineCostProvider> ptr_cost_provider = nullptr;
  GeometryEdgeVector<EdgeInfo> cost_edges{};
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_INITIALIZER_INITIALIZER_OUTPUT_H_
