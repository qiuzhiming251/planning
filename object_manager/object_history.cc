#include "object_history.h"

#include "plan_common/log_data.h"
namespace st::planning {

void UpdateObjectsHistory(
    ObjectHistoryManager& obj_history_mgr,
    const std::optional<ObjectsProto>& objects_proto,
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
    const SpacetimePlannerObjectTrajectoriesProto& st_planner_obj_trjs,
    const std::map<std::string, bool>& obj_leading,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const std::optional<NudgeObjectInfo>& nudge_info) {
  const int64_t cur_timestamp = objects_proto->header().timestamp();
  obj_history_mgr.CleanExceeded(cur_timestamp);
  absl::flat_hash_map<std::string, ObjectFrame> objects_frame;

  // DLOG(INFO) << "add default frame from objects_proto";
  for (const auto& object : objects_proto->objects()) {
    objects_frame.try_emplace(object.id(),
                              ObjectFrame{.id = object.id(),
                                          .timestamp = cur_timestamp,
                                          .object_proto = object});
    // default frame here only
    // add additional features below in case size of objects are not equal
    if (nudge_info.has_value() && (object.id() == nudge_info->id)) {
      objects_frame[object.id()].is_nudge = true;
      const std::string obs_debug = "id: " + object.id() + " is_nudge";
      Log2DDS::LogDataV2("object_history", obs_debug);
    }
  }

  for (const auto& lead : obj_leading) {
    objects_frame.at(lead.first).is_leading = lead.second;
    const std::string obs_debug =
        absl::StrCat("id: ", lead.first, "unleading: ", lead.second);
    Log2DDS::LogDataV0("object_history", obs_debug);
  }
  for (const auto& stalled : stalled_objects) {
    objects_frame.at(stalled).is_stalled = true;
    const std::string obs_debug = absl::StrCat("id: ", stalled, "is_stalled");
    Log2DDS::LogDataV0("object_history", obs_debug);
  }

  // DLOG(INFO) << "boundary_with_decision";
  for (const auto& boundary_with_decision : st_boundaries_with_decision) {
    if (!boundary_with_decision.object_id().has_value()) {
      continue;
    }
    std::string obj_id = boundary_with_decision.object_id().value();
    objects_frame.at(obj_id).lon_decision =
        boundary_with_decision.decision_type();
  }
  // DLOG(INFO) << "st_planner_obj_trjs";
  for (size_t i = 0; i < st_planner_obj_trjs.trajectory_size(); ++i) {
    std::string obj_id = st_planner_obj_trjs.trajectory(i).id();
    objects_frame.at(obj_id).lat_decision =
        st_planner_obj_trjs.trajectory(i).reason();
  }

  obj_history_mgr.AddObjectsFrameToHistory(std::move(objects_frame));

  // use test
  // for (const auto& object : objects_proto->objects()) {
  //   auto obj_id = object.id();
  //   if (obj_history_mgr.HasObject(obj_id)) {
  //     // auto frames = obj_history_mgr.GetObjHistory(obj_id)->GetFrames();
  //     auto history = obj_history_mgr.GetObjHistory(obj_id);
  //     std::vector<std::string> debug_infos;
  //     // for (auto it = frames.begin(); it != frames.end(); ++it) {
  //     debug_infos.emplace_back(absl::StrCat(
  //         "id:", obj_id, "oldest:", history->GetOldestFrame()->timestamp,
  //         "latest:", history->GetLatestFrame()->timestamp, "get_latest:",
  //         history
  //             ->GetFrameByTimestamp(
  //                 obj_history_mgr.GetObjLatestFrame(obj_id)->timestamp)
  //             ->timestamp,
  //         "get_0:", history->GetFrameByTimestamp(0)->timestamp,
  //         "get_cur-0.5:", history
  //             ->GetFrameByTimestamp(history->GetLatestFrame()->timestamp -
  //             4e5)
  //             ->timestamp));
  //     // }
  //     Log2DDS::LogDataV4("obs_his", debug_infos);
  //   }
  // }
}
}  // namespace st::planning