

#include <utility>

#include "decider/scene_manager/scene_understanding.h"

//#include "global/trace.h"
//#include "lite/check.h"
//#include "perception/lidar_pipeline/sensor_fov/sensor_fov.h"
//#include "planner/planner_manager/planner_flags.h"
#include "decider/scene_manager/occluded_objects_reasoning.h"
#include "decider/scene_manager/scene_understanding_output.h"
#include "decider/scene_manager/traffic_flow_reasoning.h"
#include "plan_common/util/status_macros.h"

namespace st::planning {

absl::StatusOr<SceneReasoningOutput> RunSceneReasoning(
    const SceneReasoningInput& input, ThreadPool* thread_pool,
    ObjectHistoryManager& obj_his_manager) {
  CHECK_NOTNULL(input.psmm);
  CHECK_NOTNULL(input.prediction);
  CHECK_NOTNULL(input.tl_info_map);
  CHECK_NOTNULL(input.lane_paths);
  // CHECK_NOTNULL(input.route_sections);
  CHECK_NOTNULL(input.plan_start_point);

  const auto& psmm = *input.psmm;
  const auto& prediction = *input.prediction;
  const auto& tl_info_map = *input.tl_info_map;
  const auto& lane_paths = *input.lane_paths;
  // const auto& route_sections = *input.route_sections;

  SceneOutputProto scene_output_proto;

  // Run Traffic flow reasoning.
  // Output: traffic waiting queue , stall objects.
  const TrafficFlowReasoningInput traffic_flow_input{
      .psmm = &psmm,
      .prediction = &prediction,
      .lane_paths = &lane_paths,
      .tl_info_map = &tl_info_map,
      // .sensor_fovs = input.sensor_fovs,
      .plan_start_point = input.plan_start_point};

  ASSIGN_OR_RETURN(auto traffic_flow_output,
                   RunTrafficFlowReasoning(traffic_flow_input, thread_pool,
                                           obj_his_manager));

  for (auto& traffic_waiting_queue :
       traffic_flow_output.traffic_waiting_queues) {
    *scene_output_proto.add_traffic_waiting_queue() =
        std::move(traffic_waiting_queue);
  }
  for (auto& object_annotation : traffic_flow_output.object_annotations) {
    *scene_output_proto.add_objects_annotation() = std::move(object_annotation);
  }

  // Run Occluded objects reasoning when sensor fov not empty.
  // if (FLAGS_planner_enable_occluded_objects_inference) {
  //   // const auto sensor_fov =
  //   //     sensor_fov::BuildLidarViewSensorFov(*input.sensor_fovs);
  //   const OccludedObjectsReasoningInput occluded_objects_input{
  //       .psmm = &psmm,
  //       // .sensor_fov = &sensor_fov,
  //       .route_sections = &route_sections};
  //   ASSIGN_OR_RETURN(auto inferred_objects,
  //                    RunOccludedObjectsReasoning(occluded_objects_input));
  //   scene_output_proto.mutable_inferred_objects()->Reserve(
  //       inferred_objects.size());
  //   for (auto& inferred_object : inferred_objects) {
  //     *scene_output_proto.add_inferred_objects() =
  //     std::move(inferred_object);
  //   }
  // }

  return SceneReasoningOutput{
      .scene_output_proto = std::move(scene_output_proto),
      .distance_to_roadblock = traffic_flow_output.distance_to_roadblock};
}

}  // namespace st::planning
