

#ifndef AD_BYD_PLANNING_COMMON_OBSTACLE_LRU_CACHE_H
#define AD_BYD_PLANNING_COMMON_OBSTACLE_LRU_CACHE_H

#include <string>

#include "obstacle.h"
#include "plan_common/util/lru_cache_async.h"

namespace ad_byd {
namespace planning {

using ObstacleLRUCache = LRUCacheAsync<Obstacle, std::string>;
using ObstacleLRUCachePtr = std::shared_ptr<ObstacleLRUCache>;
using ObstacleMapPtr =
    std::shared_ptr<std::unordered_map<std::string, ObstaclePtr>>;

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_COMMON_OBSTACLE_LRU_CACHE_H
