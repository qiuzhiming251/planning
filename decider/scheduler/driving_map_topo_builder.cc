

#include <algorithm>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "plan_common/container/strong_int.h"
#include "decider/scheduler/driving_map_topo_builder.h"
//#include "global/trace.h"

#include "plan_common/maps/semantic_map_defs.h"

namespace st::planning {

namespace {

bool ContainsLane(const ad_byd::planning::Section& sec,
                  mapping::ElementId lane_id) {
  return std::find(sec.lanes().begin(), sec.lanes().end(), lane_id) !=
         sec.lanes().end();
}

}  // namespace

absl::StatusOr<DrivingMapTopo> BuildDrivingMapByRouteOnOfflineMap(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& sections_from_start) {
  std::vector<ad_byd::planning::SectionConstPtr> sections_info;
  sections_info.reserve(sections_from_start.size());
  int tot_lanes = 0;

  for (int i = 0; i < sections_from_start.size(); ++i) {
    const auto& sec_seg = sections_from_start.route_section_segment(i);

    const auto& sec_info = psmm.FindSectionByIdOrNull(sec_seg.id);

    if (sec_info == nullptr) {
      return absl::NotFoundError(
          absl::StrCat("Can not find section ", sec_seg.id));
    }

    tot_lanes += sec_info->lanes().size();
    sections_info.push_back(sec_info);
  }

  std::vector<DrivingMapTopo::Lane> lanes;
  lanes.reserve(tot_lanes);

  for (size_t i = 0; i < sections_info.size(); ++i) {
    const auto& sec_seg = sections_from_start.route_section_segment(i);

    const auto& next_sec =
        i + 1 == sections_info.size() ? nullptr : sections_info[i + 1];

    const auto& prev_sec = i == 0 ? nullptr : sections_info[i - 1];

    for (mapping::ElementId lane_id : sections_info[i]->lanes()) {
      const auto& lane_info = psmm.FindCurveLaneByIdOrNull(lane_id);

      if (lane_info == nullptr) {
        return absl::NotFoundError(absl::StrCat("Can not find lane ", lane_id));
      }

      std::vector<mapping::ElementId> outgoing_lane_ids;
      if (next_sec != nullptr) {
        for (const auto& out_id : lane_info->next_lane_ids()) {
          if (ContainsLane(*next_sec, out_id)) {
            outgoing_lane_ids.push_back(out_id);
          }
        }
      }

      std::vector<mapping::ElementId> incoming_lane_ids;
      if (prev_sec != nullptr) {
        for (const auto& in_id : lane_info->pre_lane_ids()) {
          if (ContainsLane(*prev_sec, in_id)) {
            incoming_lane_ids.push_back(in_id);
          }
        }
      }
      // TODO: Sample lane boundary.
      lanes.push_back(DrivingMapTopo::Lane{
          .id = lane_id,
          .start_fraction = sec_seg.start_fraction,
          .end_fraction = sec_seg.end_fraction,
          .outgoing_lane_ids = std::move(outgoing_lane_ids),
          .incoming_lane_ids = std::move(incoming_lane_ids)});
    }
  }

  std::vector<uint64_t> lanes_tmp;
  lanes_tmp.assign(sections_info.front()->lanes().begin(),
                   sections_info.front()->lanes().end());
  return DrivingMapTopo(std::move(lanes), lanes_tmp);
}

// TODO: Add unit test once online semantic map converter is done.
// absl::StatusOr<DrivingMapTopo> BuildDrivingMapByOnlineMap(
//     const PlannerSemanticMapManager& psmm,
//     const mapping::OnlineSemanticMapProto& online_map, const Vec2d& ego_pos)
//     {
//   std::vector<DrivingMapTopo::Lane> lanes;
//   lanes.reserve(online_map.lanes_size());
//   std::queue<mapping::ElementId> search_queue;
//   absl::flat_hash_set<mapping::ElementId> visited_id;

//   // Handle starting lanes
//   std::queue<mapping::ElementId> start_lane_search_queue;
//   for (const auto id : online_map.lane_ids_at_ego_pos()) {
//     start_lane_search_queue.push(mapping::ElementId(id));
//   }

//   std::vector<mapping::ElementId> starting_lane_ids;
//   starting_lane_ids.reserve(online_map.lane_ids_at_ego_pos_size());
//   while (!start_lane_search_queue.empty()) {
//     const auto lane_id = start_lane_search_queue.front();
//     start_lane_search_queue.pop();

//     if (visited_id.contains(lane_id)) {
//       continue;
//     }
//     visited_id.insert(lane_id);

//     const auto* lane_info = psmm.FindCurveLaneByIdOrNull(lane_id);
//     if (lane_info == nullptr) {
//       return absl::NotFoundError(absl::StrCat("Can not find lane ",
//       lane_id));
//     }

//     constexpr double kEpsilon = 1.0;  // m.
//     const auto ego_sl = lane_info->SmoothXYToSL(ego_pos);
//     if (ego_sl.s > lane_info->length() + kEpsilon) {
//       for (const auto& id : lane_info->outgoing_lanes()) {
//         if (visited_id.contains(id)) {
//           continue;
//         }
//         start_lane_search_queue.push(id);
//       }
//       continue;
//     } else if (ego_sl.s < -kEpsilon) {
//       for (const auto& id : lane_info->incoming_lanes()) {
//         if (visited_id.contains(id)) {
//           continue;
//         }
//         start_lane_search_queue.push(id);
//       }
//       continue;
//     }

//     const double start_fraction =
//         std::clamp(ego_sl.s / lane_info->length(), 0.0, 1.0);

//     search_queue.push(lane_id);
//     lanes.push_back(DrivingMapTopo::Lane{
//         .id = lane_id,
//         .start_fraction = start_fraction,
//         .end_fraction = 1.0,
//         .outgoing_lane_ids = lane_info->outgoing_lanes(),
//         .incoming_lane_ids = lane_info->incoming_lanes(),
//     });

//     starting_lane_ids.push_back(mapping::ElementId(lane_id));
//   }

//   // BFS
//   while (!search_queue.empty()) {
//     const auto current_id = search_queue.front();
//     search_queue.pop();
//     const auto* lane_info = psmm.FindCurveLaneByIdOrNull(current_id);
//     if (lane_info == nullptr) {
//       return absl::NotFoundError(
//           absl::StrCat("Can not find lane ", current_id));
//     }
//     for (const auto& out_lane_id : lane_info->outgoing_lanes()) {
//       if (visited_id.contains(out_lane_id)) {
//         continue;
//       }
//       visited_id.insert(out_lane_id);
//       search_queue.push(out_lane_id);
//       const auto* out_lane_info = psmm.FindCurveLaneByIdOrNull(out_lane_id);
//       if (out_lane_info == nullptr) {
//         return absl::NotFoundError(
//             absl::StrCat("Can not find lane ", out_lane_id));
//       }
//       lanes.push_back(DrivingMapTopo::Lane{
//           .id = out_lane_id,
//           .start_fraction = 0.0,
//           .end_fraction = 1.0,
//           .outgoing_lane_ids = out_lane_info->outgoing_lanes(),
//           .incoming_lane_ids = out_lane_info->incoming_lanes(),
//       });
//     }
//   }

//   return DrivingMapTopo(std::move(lanes), std::move(starting_lane_ids));
// }

}  // namespace st::planning
