

#include <algorithm>
#include <utility>

#include "absl/status/status.h"
#include "plan_common/async/parallel_for.h"
#include "planner_object_manager_builder.h"
//#include "global/trace.h"
//#include "lite/logging.h"
#include "plan_common/log.h"
#include "plan_common/log_data.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction.h"
#include "predictor/prediction_util.h"

namespace st {
namespace planning {

absl::StatusOr<PlannerObjectManager> PlannerObjectManagerBuilder::Build(
    FilteredTrajectories* filtered_trajs, ThreadPool* thread_pool) {
  // ("PlannerObjectManagerBuilder::Build");

  // Filter trajectories.
  ObjectVector<PlannerObject> frame_dropped_objects;
  for (auto& object : planner_objects_) {
    auto& trajs = *object.mutable_prediction()->mutable_trajectories();
    int i = 0;

    if (object.is_frame_dropped() && trajs.size() > 0) {
      const auto reason = FilterReason::TRAJECTORY_IS_FRAME_DROPPED;
      if (filtered_trajs != nullptr) {
        auto* filtered = filtered_trajs->add_filtered();
        filtered->set_reason(reason);
        filtered->set_id(object.id());
        filtered->set_index(trajs.front().index());
      }
      Log2DDS::LogDataV2(
          "object_filters",
          absl::StrCat("[msg_proxy] filtered obj_id:", object.id(),
                       " reason:", FilterReason_Type_Name(reason)));

      frame_dropped_objects.emplace_back(object);
      trajs.erase(trajs.begin() + i, trajs.end());
      // LOG(INFO) << " frame_dropped_objects back traj num: " <<
      // frame_dropped_objects.back().num_trajs();
      continue;
    }
    for (int j = 0, n = trajs.size(); j < n; ++j) {
      auto& traj = trajs[j];
      bool is_traj_filtered = false;
      for (const auto* filter : filters_) {
        const auto reason = filter->Filter(object, traj);
        if (reason != FilterReason::NONE) {
          if (filtered_trajs != nullptr) {
            auto* filtered = filtered_trajs->add_filtered();
            filtered->set_reason(reason);
            filtered->set_id(object.id());
            filtered->set_index(traj.index());
          }
          is_traj_filtered = true;
          Log2DDS::LogDataV2(
              "object_filters",
              absl::StrCat("[msg_proxy] filtered obj_id:", object.id(),
                           " reason:", FilterReason_Type_Name(reason)));
          break;
        }
      }
      if (!is_traj_filtered) {
        // This is equivalent to erasing the filtered element, but can reduce
        // the number of element moves.
        if (i != j) {
          trajs[i] = std::move(traj);
        }
        ++i;
      }
    }
    trajs.erase(trajs.begin() + i, trajs.end());
  }

  planner_objects_.erase(
      std::remove_if(planner_objects_.begin(), planner_objects_.end(),
                     [](const auto& obj) { return obj.num_trajs() == 0; }),
      planner_objects_.end());

  return PlannerObjectManager(std::move(planner_objects_),
                              std::move(frame_dropped_objects));
}

ObjectVector<PlannerObject> BuildPlannerObjects(
    const ObjectsProto* perception, const ObjectsPredictionProto* prediction,
    std::optional<double> align_time, ThreadPool* thread_pool) {
  // _ARG2("BuildPlannerObjects", "num_objects",
  //                    perception == nullptr ? 0 : perception->objects_size(),
  //                    "num_predictions",
  //                    prediction == nullptr ? 0 : prediction->objects_size());

  struct ObjectInfo {
    std::string_view id;
    const ObjectProto* obj_ptr = nullptr;
    const ObjectPredictionProto* pred_ptr = nullptr;
  };
  absl::flat_hash_map<std::string_view, int> id_to_index;
  std::vector<ObjectInfo> obj_info;
  obj_info.reserve((perception == nullptr ? 0 : perception->objects_size()) +
                   (prediction == nullptr ? 0 : prediction->objects_size()));
  if (perception != nullptr) {
    for (const auto& obj : perception->objects()) {
      // LOG_ERROR << "planner object builder perception object id: " <<
      // obj.id();
      id_to_index[obj.id()] = obj_info.size();
      obj_info.push_back(
          {.id = obj.id(), .obj_ptr = &obj, .pred_ptr = nullptr});
    }
  }
  if (prediction != nullptr) {
    for (const auto& pred : prediction->objects()) {
      const auto& id = pred.perception_object().id();
      // LOG_ERROR << "planner object builder predition object id: " << id;
      if (const auto* index = FindOrNull(id_to_index, id); index != nullptr) {
        obj_info[*index].pred_ptr = &pred;
      } else {
        obj_info.push_back({.id = id, .obj_ptr = nullptr, .pred_ptr = &pred});
      }
    }
  }
  ObjectVector<PlannerObject> planner_objects;
  planner_objects.resize(obj_info.size());
  // do not use the thread pool
  ParallelFor(0, obj_info.size(), thread_pool, [&](int i) {
    // When there is no prediction, create one.
    // if (obj_info[i].pred_ptr == nullptr) {
    //   if (auto result = prediction::InstantPredictionForNewObject(
    //           *obj_info[i].obj_ptr,
    //           /*prediction_time=*/3.0);
    //       result.ok()) {
    //     const double time_shift =
    //         align_time.has_value()
    //             ? *align_time - obj_info[i].obj_ptr->timestamp()
    //             : 0.0;
    //     planner_objects[ObjectIndex(i)] =
    //         PlannerObject(prediction::ObjectPrediction(*result, time_shift,
    //                                                    *obj_info[i].obj_ptr));
    //   } else {
    //     LOG_ERROR << result.status();
    //   }
    //   return;
    // }
    CHECK(obj_info[i].pred_ptr != nullptr)
        << "obj_info[" << i << "] id = " << obj_info[i].id
        << " does not have prediction."
        << "\t pred_ptr = " << obj_info[i].pred_ptr;
    // In the following code, pred_ptr is not NULL.
    const ObjectProto* latest_obj = nullptr;
    if (obj_info[i].obj_ptr != nullptr &&
        obj_info[i].obj_ptr->timestamp() >
            obj_info[i].pred_ptr->perception_object().timestamp()) {
      latest_obj = obj_info[i].obj_ptr;
    } else if (obj_info[i].pred_ptr->has_perception_object()) {
      latest_obj = &obj_info[i].pred_ptr->perception_object();
    }
    if (latest_obj != nullptr) {
      const double time_shift =
          align_time.has_value() ? *align_time - latest_obj->timestamp() : 0.0;
      if (i == 0) {
        align_time.has_value()
            ? Log2DDS::LogDataV2(
                  "time_shift",
                  absl::StrFormat(" time_shift:%.3f, align_time:%.3f, "
                                  "latest_obj->timestamp:%.3f",
                                  time_shift, *align_time,
                                  latest_obj->timestamp()))
            : Log2DDS::LogDataV2(
                  "time_shift",
                  absl::StrFormat(" time_shift:%.3f", time_shift));
      }
      planner_objects[ObjectIndex(i)] =
          PlannerObject(prediction::ObjectPrediction(*obj_info[i].pred_ptr,
                                                     time_shift, *latest_obj));
    }
  });

  // Trim objects that has no prediction trajectory. It is unlikely to trigger.
  // Trim cone & barrier objects, not in use.
  // planner_objects.erase(
  //     std::remove_if(planner_objects.begin(), planner_objects.end(),
  //                    [](const PlannerObject& obj) {
  //                      return obj.num_trajs() == 0 || obj.type() == OT_CONE
  //                      ||
  //                             obj.type() == OT_BARRIER;
  //                    }),
  //     planner_objects.end());
  planner_objects.erase(
      std::remove_if(
          planner_objects.begin(), planner_objects.end(),
          [](const PlannerObject& obj) { return obj.num_trajs() == 0; }),
      planner_objects.end());
  return planner_objects;
}

}  // namespace planning
}  // namespace st
