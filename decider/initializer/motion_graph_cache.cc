

#include <algorithm>

#include "absl/status/status.h"
#include "decider/initializer/motion_graph_cache.h"

//#include "global/buffered_logger.h"
//#include "lite/check.h"
#include "plan_common/util/map_util.h"

namespace st::planning {
void MotionGraphCache::BatchGetOrFail(
    const std::vector<MotionEdgeKey>& keys,
    std::vector<DpMotionInfo>* ptr_dp_motion_infos,
    std::vector<int>* ptr_failed_idx) const {
  auto& dp_motion_infos = *ptr_dp_motion_infos;
  auto& failed_idx = *ptr_failed_idx;
  for (int i = 0; i < keys.size(); i++) {
    const auto& key = keys[i];
    const auto ptr_edge_info = FindOrNull(cache_, key);
    if (ptr_edge_info != nullptr) {
      dp_motion_infos[i].costs.resize(ptr_edge_info->costs.size());
      dp_motion_infos[i].costs = ptr_edge_info->costs;
      dp_motion_infos[i].motion_form = ptr_edge_info->ptr_motion_form.get();
      dp_motion_infos[i].ignored_trajs = ptr_edge_info->ignored_trajs;

    } else {
      failed_idx.push_back(i);
    }
  }
}

void MotionGraphCache::Insert(const MotionEdgeKey& key,
                              std::vector<double> costs,
                              IgnoreTrajMap ignored_trajs,
                              std::unique_ptr<MotionForm> ptr_motion_form) {
  if (!cache_.contains(key)) {
    MotionEdgeCache edge_cache = {.ptr_motion_form = std::move(ptr_motion_form),
                                  .costs = std::move(costs),
                                  .ignored_trajs = std::move(ignored_trajs)};
    auto [it, success] =
        cache_.emplace(std::make_pair(key, std::move(edge_cache)));
    CHECK(success);
  }
}

void MotionGraphCache::BatchInsert(std::vector<NewCacheInfo> new_motion_forms) {
  for (auto it = new_motion_forms.begin(); it != new_motion_forms.end(); ++it) {
    if (!cache_.contains(it->key)) {
      auto [iter, success] =
          cache_.emplace(std::make_pair(it->key, std::move(it->cache)));
      CHECK(success);
    }
  }
}

absl::StatusOr<MotionForm*> MotionGraphCache::GetMotionForm(
    const MotionEdgeKey& key) const {
  const auto ptr_edge_info = FindOrNull(cache_, key);
  if (ptr_edge_info != nullptr) {
    return ptr_edge_info->ptr_motion_form.get();
  } else {
    return absl::NotFoundError("Queried motion form not in cache.");
  }
}

absl::StatusOr<std::vector<double>> MotionGraphCache::GetCosts(
    const MotionEdgeKey& key) const {
  const auto ptr_edge_info = FindOrNull(cache_, key);
  if (ptr_edge_info != nullptr) {
    return ptr_edge_info->costs;
  } else {
    return absl::NotFoundError("Queried motion costs not in cache.");
  }
}

}  // namespace st::planning
