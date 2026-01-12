

#ifndef ONBOARD_PLANNER_INITIALIZER_TEST_UTIL_H_
#define ONBOARD_PLANNER_INITIALIZER_TEST_UTIL_H_

#include <memory>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "decider/initializer/geometry/geometry_form.h"
#include "decider/initializer/geometry/geometry_graph.h"
#include "decider/initializer/motion_graph.h"
#include "plan_common/math/vec.h"

namespace st {
namespace planning {

// Match CollisionInfo result.
MATCHER_P2(ObjCollisionEq, index, time, "") {
  *result_listener << "Actual value: index: " << arg.object_index
                   << ", time: " << arg.time;
  return arg.object_index == index && arg.time == time;
}

// Build a geometry graph from a straight line. A node is created for each point
// in `points`. Edges are created between consecutive nodes.
XYGeometryGraph BuildLineGraph(
    absl::Span<const Vec2d> points,
    std::vector<std::unique_ptr<GeometryForm>>* ptr_geometry_forms);

// Build a motion graph with constant acceleration.
std::pair<std::unique_ptr<XYGeometryGraph>, std::unique_ptr<XYTMotionGraph>>
BuildConstAccelLineGraph(absl::Span<const Vec2d> points, double init_speed,
                         double init_time, double accel);

};  // namespace planning

}  // namespace st

#endif  // ONBOARD_PLANNER_INITIALIZER_TEST_UTIL_H_
