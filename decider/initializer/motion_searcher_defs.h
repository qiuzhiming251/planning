

#ifndef ONBOARD_PLANNER_INITIALIZER_MOTION_SEARCHER_DEFS_H_
#define ONBOARD_PLANNER_INITIALIZER_MOTION_SEARCHER_DEFS_H_

#include <algorithm>
#include <array>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "boost/heap/pairing_heap.hpp"
#include "decider/initializer/motion_graph.h"
#include "plan_common/math/util.h"
#include "plan_common/plan_common_defs.h"
#include "modules/cnoa_pnc/planning/proto/initializer_config.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
namespace st::planning {
// DP search params
// TODO: move to planner config.
// If vehicle speed less than this threshold, we can set a stationary motion
// for it.
constexpr double kCanSetToZeroSpeed = 1.0;
// If vehicle speed less than this threshold and search failed, we can set a
// stationary motion for it.
constexpr double kSearchFailedCanSetToZeroSpeed = 1.5;
constexpr double kCanSetToZeroTrajLength = 4.0;
constexpr std::array<double, 9> kAccelerationSamplePoints = {
    -4.0, -3.0, -2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5};
constexpr double kDpDiscreteSpeedSampleStep = 3.0;  // m/s
constexpr double kDpDiscreteTimeSampleStep = 2.5;   // second.
const int kDpDiscreteTimeHorizon =
    CeilToInt(kInitializerTrajectoryTimeHorizon /
              kDpDiscreteTimeSampleStep);  // number of steps.
constexpr double kMinSpeedForFinalCost = 3.0;
constexpr int kConstVelSampleLayerSizeThreshold = 5;

constexpr double kSDiscreteMax = 130.0 / 3.6 * 8.0;
constexpr double kLDiscreteMax = 10.0;
constexpr double kSDiscreteResolution = 1.0;
constexpr double kLDiscreteResolution = 0.5;
constexpr double kTDiscreteResolution = 0.5;
constexpr double kInvSDiscreteResolution = 1.0 / kSDiscreteResolution;
constexpr double kInvLDiscreteResolution = 1.0 / kLDiscreteResolution;
constexpr double kInvTDiscreteResolution = 1.0 / kTDiscreteResolution;
constexpr uint64_t kSIndexLeftShiftBit = 1;
// constexpr uint64_t kLIndexLeftShiftBit = 1000 * 1;
// constexpr uint64_t kTIndexLeftShiftBit = 100 * 1000 * 1;
// constexpr uint64_t kNIndexLeftShiftBit = 100 * 100 * 1000 * 1;
constexpr uint64_t kLIndexLeftShiftBit =
    kSIndexLeftShiftBit *
    std::pow(10, static_cast<int>(std::floor(
                     std::log10(kSDiscreteMax* kInvSDiscreteResolution))) +
                     1);
constexpr uint64_t kTIndexLeftShiftBit =
    kLIndexLeftShiftBit *
    std::pow(10, static_cast<int>(std::floor(
                     std::log10(kLDiscreteMax* kInvLDiscreteResolution))) +
                     1);
constexpr uint64_t kNIndexLeftShiftBit =
    kTIndexLeftShiftBit *
    std::pow(10,
             static_cast<int>(std::floor(std::log10(
                 kInitializerTrajectoryTimeHorizon* kInvTDiscreteResolution))) +
                 1);

constexpr bool kEnableAstarSearchDebug = false;

// Discrete motion samples on geometry node, used to get opt_motion_edge.
struct DpMotionSample {
  int v_discrete = 0;  // discrete speed step
  int t_discrete = 0;  // discrete time step
  DpMotionSample(double v, double t)
      : v_discrete(std::max(0, RoundToInt(v / kDpDiscreteSpeedSampleStep))),
        t_discrete(std::clamp(RoundToInt(t / kDpDiscreteTimeSampleStep), 0,
                              kDpDiscreteTimeHorizon)) {}

  friend bool operator==(const DpMotionSample& lhs, const DpMotionSample& rhs) {
    return lhs.v_discrete == rhs.v_discrete && lhs.t_discrete == rhs.t_discrete;
  }

  template <typename H>
  friend H AbslHashValue(H h, const DpMotionSample& ms) {
    return H::combine(std::move(h), ms.v_discrete, ms.t_discrete);
  }
};

// Candidate trajectory info.
struct TrajInfo {
  MotionEdgeIndex idx{};
  double total_cost = 0.0;
  std::vector<double> feature_costs{};
  std::vector<ApolloTrajectoryPointProto> traj_points{};
};

struct BestEdgeInfo {
  MotionEdgeIndex idx{};
  double total_cost = 0.0;
  bool is_created_stationary_motion = false;
};

enum class CollisionConfiguration {
  NONE = 0,
  FRONT = 1,  // At least one corner of the considered obstacle is beyond the
              // front bumper of AV right before collision.
  LEFT = 2,   // All corners of the obstacle is behind the front bumper of AV
              // and at least one corner is beyond the rear bumper of AV and on
              // the left of AV right before collision.
  RIGHT = 3,  // All corners of the obstacle is behind the front bumper of AV
              // and at least one corner is beyond the rear bumper of AV and
              // on the right of AV right before collision.
  BACK = 4,   // All corners of the obstacle is behind the rear bumper of AV
              // right before the collision.
};
struct CollisionConfigurationInfo {
  int time_idx = 0;
  CollisionConfiguration collision_config = CollisionConfiguration::NONE;
};

using IgnoreTrajMap =
    absl::flat_hash_map<std::string, CollisionConfigurationInfo>;

// Astar Search Node Definition
// We just change the search procedure fromt DP method to Astar method,
// The Base Geometry Graph & Motion Form are kept
// Note that we use Dp search cost feture map to get the h_cost.

/*!
   @brief to sort nodes in a heap structure
*/
struct CompareAstarNodes {
  // Sorting 4D nodes by increasing C value - the total estimated cost
  bool operator()(const std::pair<uint64_t, double>& lhs,
                  const std::pair<uint64_t, double>& rhs) const {
    return lhs.second > rhs.second;
  }
};

typedef boost::heap::pairing_heap<std::pair<uint64_t, double>,
                                  boost::heap::compare<CompareAstarNodes>>
    AstarPriorityQueue;

struct AStarSearchNode {
  double accumulated_t = 0.0;
  double s_to_init = 0.0;
  uint64_t index = 0;
  uint64_t pred_index = 0;
  bool is_start_node = false;
  bool is_open = false;
  bool is_close = false;
  double g_cost = 0.0;
  double h_cost = 0.0;
  double total_cost = 0.0;
  std::vector<double> feature_costs{};
  std::vector<double> heuristic_costs{};
  GeometryNodeIndex geometry_node_idx{};
  GeometryEdgeIndex geometry_edge_idx{};
  double v_limit = 0.0;
  double v_ref = 0.0;
  // from pred to cur
  MotionForm* motion_form = nullptr;
  IgnoreTrajMap ignored_trajs{};

  // compute the index of the current astar node
  void SetIndex(const GeometryGraph& geom_graph,
                const InitializerConfig& initializer_params) {
    // const auto& astar_search_config =
    // initializer_params.astar_search_config(); uint64_t pos_size =
    // geom_graph.nodes().size(); uint64_t discrete_v =
    // motion_form->GetEndMotionState().v /
    //                       astar_search_config.velocity_resolution();
    // uint64_t discrete_t = accumulated_t /
    // astar_search_config.time_resolution(); uint64_t v_size =
    // astar_search_config.max_velocity() /
    //                   astar_search_config.velocity_resolution();
    // uint64_t t_size =
    //     astar_search_config.max_time() /
    //     astar_search_config.time_resolution();
    // index = geometry_node_idx.value() + discrete_v * pos_size +
    //         discrete_t * v_size * pos_size;
    if (motion_form == nullptr) {
      index = 0;
      return;
    }

    const auto& end_state = motion_form->GetEndMotionState();
    index =
        static_cast<uint64_t>(s_to_init * kInvSDiscreteResolution + 0.5) *
            kSIndexLeftShiftBit +
        static_cast<uint64_t>((end_state.l + 0.5 * kLDiscreteMax) *
                                  kInvLDiscreteResolution +
                              0.5) *
            kLIndexLeftShiftBit +
        static_cast<uint64_t>(accumulated_t * kInvTDiscreteResolution + 0.5) *
            kTIndexLeftShiftBit +
        geometry_edge_idx.value() * kNIndexLeftShiftBit;
  }
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_INITIALIZER_MOTION_SEARCHER_DEFS_H_
