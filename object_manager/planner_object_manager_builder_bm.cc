

#include <vector>

#include "benchmark/benchmark.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/vec.h"
#include "planner_object_manager_builder.h"
#include "test_util/object_prediction_builder.h"
#include "test_util/perception_object_builder.h"
#include "test_util/predicted_trajectory_builder.h"

namespace st {
namespace planning {
namespace {

Polygon2d GeneratePolygon(const Vec2d& center, int num_points, double radius) {
  std::vector<Vec2d> points;
  const double angle_step = 2.0 * M_PI / num_points;
  points.reserve(num_points);
  for (int i = 0; i < num_points; ++i) {
    points.push_back(center +
                     radius * Vec2d::FastUnitFromAngle(angle_step * i));
  }
  return Polygon2d(std::move(points), /*is_convex=*/true);
}

ObjectProto BuildObject(const std::string& id, const Polygon2d& contour,
                        double timestamp) {
  return PerceptionObjectBuilder()
      .set_id(id)
      .set_type(OT_VEHICLE)
      .set_pos(Vec2d(1.0, 0.0))
      .set_timestamp(timestamp)
      .set_velocity(2.0)
      .set_yaw(0.0)
      .set_length_width(4.0, 2.0)
      .set_box_center(Vec2d::Zero())
      .set_contour(contour)
      .Build();
}

ObjectsProto BuildPerceptionObjects(int num_objects, int num_contour_points,
                                    double timestamp) {
  ObjectsProto objects;
  objects.mutable_header()->set_timestamp(timestamp);
  objects.set_scope(ObjectsProto::SCOPE_REAL);
  const Polygon2d contour = GeneratePolygon(/*center=*/Vec2d::Zero(),
                                            num_contour_points, /*radius=*/2.0);
  for (int i = 0; i < num_objects; ++i) {
    *objects.add_objects() = BuildObject(absl::StrCat(i), contour, timestamp);
  }
  return objects;
}

ObjectsPredictionProto BuildPredictionObjects(int num_objects,
                                              int num_contour_points,
                                              double timestamp) {
  ObjectsPredictionProto prediction_proto;
  prediction_proto.mutable_header()->set_timestamp(timestamp);

  const Polygon2d contour = GeneratePolygon(/*center=*/Vec2d::Zero(),
                                            num_contour_points, /*radius=*/2.0);
  constexpr int kNumTrajs = 3;
  constexpr double kProb = (1.0 - 1e-6) / kNumTrajs;
  for (int i = 0; i < num_objects; ++i) {
    ObjectPredictionBuilder builder;
    builder.set_object(BuildObject(absl::StrCat(i), contour, timestamp));
    for (int i = 0; i < kNumTrajs; ++i) {
      builder.add_predicted_trajectory()
          ->set_probability(kProb)
          .set_straight_line(Vec2d::Zero(), Vec2d(0.0, 50.0),
                             /*init_v=*/0.2, /*last_v=*/0.2);
    }
    builder.Build().ToProto(prediction_proto.add_objects());
  }
  return prediction_proto;
}

void BM_BuildPlannerObjects(benchmark::State& state) {  // NOLINT
  const int perception_objects = state.range(0);
  const int prediction_objects = state.range(1);
  const int num_contour_points = state.range(2);
  const auto objects_proto =
      BuildPerceptionObjects(perception_objects, num_contour_points,
                             /*timestamp=*/100000.1);
  const auto prediction_proto = BuildPredictionObjects(
      prediction_objects, num_contour_points, /*timestamp=*/100000.0);
  for (auto _ : state) {
    benchmark::DoNotOptimize(BuildPlannerObjects(
        &objects_proto, &prediction_proto, /*align_time=*/std::nullopt,
        /*thread_pool=*/nullptr));
  }
}
BENCHMARK(BM_BuildPlannerObjects)->Args({10, 20, 16});

void BM_BuildPlannerObjectsAlignTime(benchmark::State& state) {  // NOLINT
  const int perception_objects = state.range(0);
  const int prediction_objects = state.range(1);
  const int num_contour_points = state.range(2);
  const auto objects_proto =
      BuildPerceptionObjects(perception_objects, num_contour_points,
                             /*timestamp=*/100000.1);
  const auto prediction_proto = BuildPredictionObjects(
      prediction_objects, num_contour_points, /*timestamp=*/100000.0);
  for (auto _ : state) {
    benchmark::DoNotOptimize(BuildPlannerObjects(
        &objects_proto, &prediction_proto, /*align_time=*/100000.1,
        /*thread_pool=*/nullptr));
  }
}
BENCHMARK(BM_BuildPlannerObjectsAlignTime)->Args({10, 20, 16});

}  // namespace
}  // namespace planning
}  // namespace st

BENCHMARK_MAIN();
