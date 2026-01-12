

#ifndef ONBOARD_PLANNER_SCENE_SCENE_UNDERSTANDING_OUTPUT_H_
#define ONBOARD_PLANNER_SCENE_SCENE_UNDERSTANDING_OUTPUT_H_

#include <vector>

#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"

namespace st::planning {
// TODO: Move to traffic_follow_reasoning.h
struct TrafficFlowReasoningOutput {
  std::vector<TrafficWaitingQueueProto> traffic_waiting_queues;
  std::vector<ObjectAnnotationProto> object_annotations;
  std::optional<double> distance_to_roadblock = std::nullopt;
};

}  // namespace st::planning
#endif
