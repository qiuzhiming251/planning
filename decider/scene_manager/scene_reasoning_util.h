

#ifndef ONBOARD_PLANNER_SCENE_SCENE_REASONING_UTIL_H_
#define ONBOARD_PLANNER_SCENE_SCENE_REASONING_UTIL_H_

#include "modules/cnoa_pnc/planning/proto/planner.pb.h"
#include "modules/cnoa_pnc/planning/proto/scene_understanding.pb.h"
#include "object_manager/planner_object_manager.h"

namespace st::planning {
// Parse object annotation reasoning result to planner debug proto. Using
// PlannerObjectManager to calculate object position, this debug info
// will used to show stall objects in worldview.
void ParseObjectAnnotationToDebugProto(
    const ::google::protobuf::RepeatedPtrField<ObjectAnnotationProto>&
        objects_annotation,
    const PlannerObjectManager& object_manager, PlannerDebugProto* debug);

// Parse traffic waiting queue reasoning result to planner debug prot. Using
// PlannerObjectManager to calculate object position, this debug info will used
// to show traffic waiting objects in worldview.
void ParseTrafficWaitingQueueToDebugProto(
    const ::google::protobuf::RepeatedPtrField<TrafficWaitingQueueProto>&
        traffic_waiting_queues,
    const PlannerObjectManager& object_manager, PlannerDebugProto* debug);
}  // namespace st::planning

#endif
