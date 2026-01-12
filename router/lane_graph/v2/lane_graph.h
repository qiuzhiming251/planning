

#ifndef ONBOARD_PLANNER_ROUTER_LANE_GRAPH_V2_LANE_GRAPH_H_
#define ONBOARD_PLANNER_ROUTER_LANE_GRAPH_V2_LANE_GRAPH_H_

#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "plan_common/container/flat_map.h"
//#include "lite/check.h"
//#include "lite/logging.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/util/map_util.h"
#include "plan_common/log.h"

namespace std {
inline std::string to_string(const std::pair<int, std::string>& vertex_id) {
  return absl::StrCat(vertex_id.first, "-", vertex_id.second);
}
}  // namespace std
namespace st::planning::v2 {

static constexpr double kInfiniteCost = std::numeric_limits<double>::max();

template <typename VertexId>
struct TravelPath {
  std::vector<VertexId> vertices;
  double total_cost = kInfiniteCost;
};

template <typename VertexId>
struct VertexGraph {
 public:
  // Maps to_id to edge cost, order matters for reproducibility.
  using VertexEdgeMap = FlatMap<VertexId, double>;
  using VertexSet = absl::flat_hash_set<VertexId>;

 public:
  void AddVertex(const VertexId& vert_id) {
    if (vertices_.contains(vert_id)) {
      LOG_WARN << absl::StrCat(vert_id, " already exists!");
      return;
    }
    vertices_.insert(vert_id);
    edges_.emplace(vert_id, VertexEdgeMap());
  }

  /**
   * @brief It's `UpdateOrInsert` an edge, other than `Insert`. The implicit
   * check should be avoid.
   *
   * @param from_id
   * @param to_id
   * @param cost
   */
  void AddEdge(const VertexId& from_id, const VertexId& to_id, double cost) {
    if (!vertices_.contains(from_id) || !vertices_.contains(to_id)) {
      LOG_WARN << absl::StrCat("please insert vertices first,", from_id, "->",
                               to_id);
      return;
    }
    if (cost == kInfiniteCost) return;

    auto& edge_map = edges_[from_id];
    if (const auto res = edge_map.emplace(to_id, cost); !res.second) {
      res.first->second = std::min(res.first->second, cost);
    }
  }

  void RemoveEdge(const VertexId& from_id, const VertexId& to_id) {
    if (!IsEdgeValid(from_id, to_id)) {
      LOG_WARN << absl::StrCat("Invalid edge ", from_id, "->", to_id);
      return;
    }
    edges_[from_id].erase(to_id);
  }

  void RemoveEdgesFrom(const VertexId& from_id) {
    if (!vertices_.contains(from_id)) {
      LOG_WARN << absl::StrCat(from_id, " does not exist!");
      return;
    }
    edges_[from_id].clear();
  }

  void ModifyEdge(const VertexId& from_id, const VertexId& to_id, double cost) {
    if (!IsEdgeValid(from_id, to_id)) {
      LOG_WARN << absl::StrCat("Invalid edge(", from_id, "->", to_id, ")!");
      return;
    }
    edges_[from_id][to_id] = cost;
  }

  bool has_vertex(const VertexId& vert_id) const {
    return vertices_.contains(vert_id);
  }
  double edge_cost(const VertexId& from_id, const VertexId& to_id) const {
    if (const auto* edge = FindOrNull(edges_, from_id)) {
      if (const auto* v = FindOrNull(*edge, to_id)) {
        return *v;
      }
    }
    return kInfiniteCost;
  }
  const VertexSet& vertices() const { return vertices_; }
  const VertexEdgeMap& edges_from(const VertexId& from_id) const {
    DCHECK(vertices_.contains(from_id))
        << absl::StrCat("Invalid vertex id ", from_id);
    return FindOrDieNoPrint(edges_, from_id);
  }

  std::string DebugString() const {
    std::stringstream ss;
    ss << vertices_.size() << " vertices in total\n";
    for (const auto& vertex : vertices_) {
      ss << "  " << vertex << ":\n";
      for (const auto& [to_id, cost] : FindOrDieNoPrint(edges_, vertex)) {
        ss << "    edge to " << to_id << " with cost " << cost << std::endl;
      }
    }
    return ss.str();
  }

 private:
  bool IsEdgeValid(const VertexId& from_id, const VertexId& to_id) const {
    return vertices_.contains(from_id) && vertices_.contains(to_id) &&
           ContainsKey(FindOrDieNoPrint(edges_, from_id), to_id);
  }

  VertexSet vertices_;
  absl::flat_hash_map<VertexId, VertexEdgeMap> edges_;
};

// inline std::pair<int, std::string> ToVertId(int layer,
//                                             mapping::ElementId lane_id) {
//   return {layer, lane_id};
// }

inline std::pair<int, std::string> ToVertId(int layer,
                                            const std::string& lane_id) {
  return {layer, lane_id};
}

inline std::pair<int, mapping::ElementId> FromVertId(
    std::pair<int, uint64_t> vertex_id) {
  return {vertex_id.first, mapping::ElementId(vertex_id.second)};
}

struct LaneGraph {
  using VertexId = std::pair<int, std::string>;
  using EdgeType = std::pair<VertexId, VertexId>;
  using LaneGraphLayers = std::vector<std::pair<int, double>>;

  static constexpr double kInfiniteCost =
      std::numeric_limits<double>::infinity();
  static constexpr double kDeadEndToTargetCost = 1e10;
  static const VertexId kInvalidVertexId;
  static const VertexId kTargetVertex;

  // Maps a layer to the corresponding section index and fraction.
  LaneGraphLayers layers;
  VertexGraph<VertexId> graph;
  absl::flat_hash_set<VertexId> fork_lc_verts;
  absl::flat_hash_set<EdgeType> fork_lk_edges;
};

const LaneGraph::VertexId& InvalidVertexId(LaneGraph::VertexId);
// Only for unittest
// constexpr std::int64_t InvalidVertexId(std::int64_t) { return -1L; }

}  // namespace st::planning::v2

#endif  // ONBOARD_PLANNER_ROUTER_LANE_GRAPH_V2_LANE_GRAPH_H_
