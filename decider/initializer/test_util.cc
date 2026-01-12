

#include "decider/initializer/test_util.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

//#include "global/buffered_logger.h"
//#include "lite/check.h"
#include "decider/initializer/motion_form.h"
#include "decider/initializer/motion_graph.h"
#include "decider/initializer/motion_state.h"
#include "plan_common/math/util.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"

namespace st {
namespace planning {

XYGeometryGraph BuildLineGraph(
    absl::Span<const Vec2d> points,
    std::vector<std::unique_ptr<GeometryForm>>* ptr_geometry_forms) {
  CHECK(!points.empty());
  GeometryNodeVector<GeometryNode> nodes;
  std::vector<std::vector<GeometryNodeIndex>> nodes_layers_idx;
  nodes.reserve(points.size());
  nodes_layers_idx.reserve(points.size());
  for (int i = 0; i < points.size(); ++i) {
    const auto pt = points[i];
    std::vector<GeometryNodeIndex> nodes_layer_idx;
    nodes.push_back(GeometryNode{
        .index = GeometryNodeIndex(i), .xy = pt, .station_index = i});
    nodes_layer_idx.push_back(GeometryNodeIndex(i));
    nodes_layers_idx.emplace_back(nodes_layer_idx);
  }

  GeometryEdgeVector<GeometryEdge> edges;
  edges.reserve(points.size() - 1);

  GeometryNodeVector<std::vector<GeometryEdgeIndex>> outgoing_edges;
  outgoing_edges.reserve(points.size() - 1);

  for (int i = 1; i < points.size(); ++i) {
    std::unique_ptr<GeometryForm> ptr_geometry_form =
        std::make_unique<StraightLineGeometry>(points[i - 1], points[i]);
    ptr_geometry_forms->emplace_back(std::move(ptr_geometry_form));
    edges.push_back(GeometryEdge{.index = GeometryEdgeIndex(i - 1),
                                 .start = GeometryNodeIndex(i - 1),
                                 .end = GeometryNodeIndex(i),
                                 .geometry = ptr_geometry_forms->back().get()});
    outgoing_edges.push_back({GeometryEdgeIndex(i - 1)});
  }

  // The last node has no outgoing edges.
  outgoing_edges.push_back(std::vector<GeometryEdgeIndex>());

  GeometryGraphProto::EndInfo end_info;

  return XYGeometryGraph(std::move(nodes), std::move(nodes_layers_idx),
                         std::move(edges), std::move(outgoing_edges),
                         std::move(end_info));
}

std::pair<std::unique_ptr<XYGeometryGraph>, std::unique_ptr<XYTMotionGraph>>
BuildConstAccelLineGraph(absl::Span<const Vec2d> points, double init_speed,
                         double init_time, double accel) {
  CHECK(!points.empty());

  // Build a line shape geometry graph.
  std::vector<std::unique_ptr<GeometryForm>> ptr_geometry_forms;
  auto geom_graph = std::make_unique<XYGeometryGraph>(
      BuildLineGraph(points, &ptr_geometry_forms));
  const Vec2d tangent = points.size() == 1
                            ? Vec2d(1.0, 0.0)
                            : Vec2d(points[1] - points[0]).Unit();

  MotionState prev_state = {.xy = points[0],
                            .h = tangent.FastAngle(),
                            .k = 0.0,
                            .t = init_time,
                            .v = init_speed,
                            .a = accel};

  // Build a motion graph with constant acceleration edges.
  auto motion_graph = std::make_unique<XYTMotionGraph>(geom_graph.get());
  std::vector<std::unique_ptr<MotionForm>> motion_forms;
  motion_graph->AddMotionNode(prev_state,
                              GeometryNodeVector<GeometryNode>::kInvalidIndex);
  for (int i = 1; i < points.size(); ++i) {
    MotionState cur_state;
    cur_state.xy = points[i];
    const Vec2d dir = cur_state.xy - prev_state.xy;
    cur_state.h = dir.FastAngle();
    cur_state.a = accel;
    cur_state.k = 0.0;

    const double len = dir.norm();
    constexpr double kEpsilon = 1e-8;
    const double duration =
        std::fabs(accel) < kEpsilon
            ? (len / prev_state.v)
            : ((-prev_state.v + std::sqrt(Sqr(prev_state.v) + 2.0 * len)) /
               accel);
    cur_state.t = prev_state.t + duration;
    cur_state.v = prev_state.v + accel * duration;
    const auto index =
        motion_graph->AddMotionNode(cur_state, GeometryNodeIndex(i));
    CHECK_EQ(index.value(), i);

    prev_state = cur_state;
  }
  motion_forms.reserve(points.size());
  for (int i = 1; i < points.size(); ++i) {
    const auto& prev_node = motion_graph->GetMotionNode(MotionNodeIndex(i - 1));
    std::unique_ptr<MotionForm> new_motion_form =
        std::make_unique<ConstAccelMotion>(
            prev_node.state.v, accel,
            geom_graph->GetEdge(GeometryEdgeIndex(i - 1)).geometry);
    motion_graph->AddMotionEdge(
        /*start_node_index=*/prev_node.index,
        /*end_node_index=*/MotionNodeIndex(i),
        /*motion_form=*/new_motion_form.get(),
        /*end_geom_index=*/GeometryNodeIndex(i),
        /*prev_edge=*/i == 1 ? MotionEdgeVector<MotionEdge>::kInvalidIndex
                             : MotionEdgeIndex(i - 1));
    motion_forms.emplace_back(std::move(new_motion_form));
  }
  return {std::move(geom_graph), std::move(motion_graph)};
}

}  // namespace planning
}  // namespace st
