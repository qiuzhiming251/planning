

#ifndef ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_CACHE_H_
#define ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_CACHE_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <math.h>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "plan_common/async/async_util.h"
#include "decider/initializer/motion_searcher_defs.h"
#include "decider/initializer/geometry/geometry_graph.h"
#include "decider/initializer/motion_form.h"
#include "decider/initializer/motion_graph.h"
//#include "plan_common/util/source_location.h"

namespace st::planning {
// motion graph cache is to collect the motion edges calculated as well as their
// related costs (except for the leading object costs);

// (v, acc, t, GeometryForm) -> costs (or summation oftthe calculated costs.)

// For one particular node, the motion form ended at this node should be
// specified by: acc, init_v, and the GeometryForm (which is specified by
// start_node, end_node, as well as the type of the connection: lateral quintic
// polynomials or quintic spirals.)

struct MotionEdgeCache {
  std::unique_ptr<MotionForm> ptr_motion_form = nullptr;
  std::vector<double> costs{};
  IgnoreTrajMap ignored_trajs{};
};

// Discrete motion edge sample
struct MotionEdgeKey {
  // One specific motion edge sample can be uniquely defined as a geometry edge
  // (defined as the start node, end node as well as the geometry edge type)
  // plus it's initial velocity and acceleration. Then a MotionEdgeKey can be
  // created using DpMotionInfo directly during motion search.

  int acc = 0;
  int init_v = 0;
  int t = 0;
  GeometryEdgeIndex geom_edge_index{};

  double v0() const { return init_v / 100.0; }
  double a0() const { return acc / 100.0; }
  double t0() const { return t / 100.0; }
  std::string DebugString() const {
    return absl::StrCat(acc, "\t", init_v, "\t", t, "\t",
                        geom_edge_index.value());
  }

  MotionEdgeKey(double acc, double init_v, double t,
                GeometryEdgeIndex geom_edge_index)
      : acc(round(acc * 100)),
        init_v(round(init_v * 100)),
        t(round(t * 100)),
        geom_edge_index(geom_edge_index) {}

  MotionEdgeKey()
      : acc(0),
        init_v(0),
        t(0),
        geom_edge_index(GeometryEdgeVector<GeometryEdge>::kInvalidIndex) {}

  friend bool operator==(const MotionEdgeKey& lhs, const MotionEdgeKey& rhs) {
    return lhs.acc == rhs.acc && lhs.t == rhs.t && lhs.init_v == rhs.init_v &&
           lhs.geom_edge_index.value() == rhs.geom_edge_index.value();
  }

  template <typename H>
  friend H AbslHashValue(H h, const MotionEdgeKey& mek) {
    return H::combine(std::move(h), mek.acc, mek.init_v, mek.t,
                      mek.geom_edge_index.value());
  }
};

struct NewCacheInfo {
  MotionEdgeKey key{};
  MotionEdgeCache cache{};
};

// Main structure to hold information during dynamic programming.
struct DpMotionInfo {
  MotionEdgeKey key{};
  double sum_cost = 0.0;
  std::vector<double> costs{};
  double start_t = 0.0;
  MotionEdgeIndex prev_motion_edge_index{};
  GeometryNodeIndex end_geometry_node_index{};
  MotionForm* motion_form = nullptr;
  GeometryEdgeIndex geometry_edge_index{};
  IgnoreTrajMap ignored_trajs{};
  std::string DebugString() const {
    return absl::StrCat(
        "sum_cost: ", sum_cost, "start_t: ", start_t,
        "prev_motion_edge_index: ", prev_motion_edge_index.value(),
        "end_geometry_node_index: ", end_geometry_node_index.value(),
        "\n Geometry edge index: ", geometry_edge_index.value());
  }
};

class MotionGraphCache {
 public:
  MotionGraphCache() {}

  void BatchGetOrFail(const std::vector<MotionEdgeKey>& samples,
                      std::vector<DpMotionInfo>* ptr_result,
                      std::vector<int>* failed_idx) const;

  void Insert(const MotionEdgeKey& key, std::vector<double> costs,
              IgnoreTrajMap ignored_trajs,
              std::unique_ptr<MotionForm> ptr_motion_form);

  void BatchInsert(std::vector<NewCacheInfo> new_motion_forms);

  inline bool has(const MotionEdgeKey& key) const {
    return cache_.contains(key);
  }

  absl::StatusOr<MotionForm*> GetMotionForm(const MotionEdgeKey& key) const;

  absl::StatusOr<std::vector<double>> GetCosts(const MotionEdgeKey& key) const;

  inline int size() const { return cache_.size(); }

  ~MotionGraphCache() {
    DestroyContainerAsyncMarkSource(std::move(cache_), "");
  }

 private:
  absl::flat_hash_map<MotionEdgeKey, MotionEdgeCache> cache_{};
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_INITIALIZER_MOTION_GRAPH_CACHE_
