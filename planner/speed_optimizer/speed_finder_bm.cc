

#include <vector>

#include "benchmark/benchmark.h"
#include "decider/decision_manager/end_of_path_boundary.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/positioning.pb.h"
#include "modules/cnoa_pnc/planning/proto/prediction.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "object_manager/planner_object_manager_builder.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/drive_passage.h"
#include "router/drive_passage_builder.h"
#include "router/route_sections_util.h"
#include "decider/scheduler/path_boundary_builder.h"
#include "plan_common/ref_line/smooth_reference_line_result.h"
#include "planner/speed_optimizer/speed_finder.h"
#include "planner/speed_optimizer/speed_finder_input.h"
#include "test_util/load_psmm_util.h"
#include "test_util/object_prediction_builder.h"
#include "test_util/perception_object_builder.h"
#include "test_util/route_builder.h"
#include "test_util/util.h"
#include "plan_common/util/lane_path_util.h"

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
      .set_pos(Vec2d(30.0, 0.0))
      .set_timestamp(timestamp)
      .set_velocity(5.0)
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
          .set_straight_line(Vec2d::Zero(), Vec2d(30.0, 0.0),
                             /*init_v=*/0.2, /*last_v=*/0.2);
    }
    builder.Build().ToProto(prediction_proto.add_objects());
  }
  return prediction_proto;
}

void BM_SpeedFinder(benchmark::State& state) {  // NOLINT
  const PlannerParamsProto planner_params = DefaultPlannerParams();
  const VehicleGeometryParamsProto vehicle_geometry_params =
      DefaultVehicleGeometry();
  const VehicleDriveParamsProto& vehicle_drive_params =
      DefaultVehicleDriveParams();
  const MotionConstraintParamsProto& motion_constraint_params =
      planner_params.motion_constraint_params();
  const SpeedFinderParamsProto& speed_finder_params =
      planner_params.speed_finder_params();

  constexpr double kVx = 10.0;
  constexpr double kVy = 0.0;
  const PoseProto sdc_pose =
      CreatePose(/*timestamp=*/0.0, /*pos=*/Vec2d(20.0, 0.0),
                 /*heading=*/0.0, /*speed_vec=*/Vec2d(kVx, kVy));
  // Plan start point.
  const ApolloTrajectoryPointProto plan_start_point =
      ConvertToTrajPointProto(sdc_pose);

  SetMap("dojo");
  auto smm = std::make_shared<SemanticMapManager>();
  smm->LoadWholeMap().Build();

  PlannerSemanticMapModification psmm_modifier;
  const auto& planner_semantic_map_manager = CreateDojoTestPSMM();

  // Build drive passage.
  const auto route_path = RoutingToNameSpot(*smm, sdc_pose, "7b_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  const auto target_lane_path =
      BackwardExtendTargetAlignedRouteLanePath(
          planner_semantic_map_manager,
          !route_path.transitions().front().lc_left,
          route_path.lane_paths()[1].front(), route_path.lane_paths().front())
          .Connect(smm.get(), route_path.lane_paths()[1]);
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(planner_semantic_map_manager,
                                            route_sections, target_lane_path,
                                            kDrivePassageKeepBehindLength);
  CHECK(backward_extended_lane_path.ok());
  const auto drive_passage = BuildDrivePassage(
      planner_semantic_map_manager, target_lane_path,
      *backward_extended_lane_path,
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(planner_semantic_map_manager,
                                      kPlanningTimeHorizon),
      route_sections.destination(), /*all_lanes_virtual=*/false,
      /*override_speed_limit_mps=*/std::nullopt);
  CHECK(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";

  // Create spacetime object manager.
  constexpr int kNumContourPoints = 16;
  const int num_perception_objects = state.range(0);
  const auto perception_objects =
      BuildPerceptionObjects(num_perception_objects, kNumContourPoints,
                             /*timestamp=*/0.0);
  const auto prediction_objects =
      BuildPredictionObjects(num_perception_objects, kNumContourPoints,
                             /*timestamp=*/0.0);
  const auto planner_objects =
      BuildPlannerObjects(&perception_objects, &prediction_objects,
                          /*align_time=*/0.0,
                          /*thread_pool=*/nullptr);
  SpacetimeTrajectoryManager st_traj_mgr(planner_objects);

  // Build path sl boundary.
  LaneChangeStateProto lane_change_state;
  lane_change_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;
  const auto path_sl_boundary = BuildPathBoundaryFromPose(
      planner_semantic_map_manager, *drive_passage, plan_start_point,
      vehicle_geometry_params, st_traj_mgr, lane_change_state,
      smooth_result_map,
      /*borrow_lane_boundary=*/false,
      /*should_smooth_next_left_turn=*/false);
  CHECK(path_sl_boundary.ok());

  // Build constraint manager.
  ConstraintManager constraint_mgr;
  auto end_of_path_boundary_constraint =
      BuildEndOfPathBoundaryConstraint(*drive_passage, *path_sl_boundary);
  CHECK(end_of_path_boundary_constraint.ok());
  constraint_mgr.AddStopLine(
      std::move(end_of_path_boundary_constraint).value());

  const std::map<std::string, ConstraintProto::LeadingObjectProto>
      leading_trajs;
  const absl::flat_hash_set<std::string> follower_set;

  const absl::flat_hash_set<std::string> stalled_objects;

  constexpr int kPathPointNum = kSpeedFinderMaxTrajectorySteps;
  std::vector<PathPoint> path_points;
  path_points.reserve(kPathPointNum);
  for (int k = 0; k < kPathPointNum; ++k) {
    auto& path_point = path_points.emplace_back();
    path_point.set_x(plan_start_point.path_point().x() +
                     k * kTrajectoryTimeStep * kVx);
    path_point.set_y(plan_start_point.path_point().y() +
                     k * kTrajectoryTimeStep * kVy);
    path_point.set_z(0.0);
    path_point.set_theta(plan_start_point.path_point().theta());
    path_point.set_kappa(0.0);
    path_point.set_lambda(0.0);
    path_point.set_s(kVx * k * 0.1);
  }

  const DiscretizedPath path(path_points);

  const SpeedFinderInput input{.base_name = "SpeedFinderBenchmark",
                               .psmm = &planner_semantic_map_manager,
                               .traj_mgr = &st_traj_mgr,
                               .constraint_mgr = &constraint_mgr,
                               .leading_trajs = &leading_trajs,
                               .follower_set = &follower_set,
                               .drive_passage = &(*drive_passage),
                               .path_sl_boundary = &(*path_sl_boundary),
                               .stalled_objects = &stalled_objects,
                               .path = &path,
                               .st_path_points = &path_points,
                               .safe_invariance_speed_info = nullptr,
                               .plan_start_v = plan_start_point.v(),
                               .plan_start_a = plan_start_point.a(),
                               .plan_start_j = plan_start_point.j(),
                               .plan_time = absl::UnixEpoch(),
                               .planner_av_context = nullptr,
                               .objects_proto = &perception_objects,
                               .planner_model_pool = nullptr,
                               .run_act_net_speed_decision = false};

  for (auto _ : state) {
    benchmark::DoNotOptimize(
        FindSpeed(input, vehicle_geometry_params, vehicle_drive_params,
                  motion_constraint_params, speed_finder_params,
                  /*thread_pool=*/nullptr));
  }
}
BENCHMARK(BM_SpeedFinder)->Arg(1)->Arg(5)->Arg(10);

}  // namespace
}  // namespace planning
}  // namespace st

BENCHMARK_MAIN();
