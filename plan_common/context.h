

#ifndef AD_BYD_PLANNING_COMMON_CONTEXT_H
#define AD_BYD_PLANNING_COMMON_CONTEXT_H
#include <list>
#include <memory>

#include "frame.h"
#include "input_frame.h"
#include "obstacle_lru_cache.h"
#include "plan_common/ref_line/ref_routing_lane_center.h"
#include "plan_common/maps/map.h"
#include "plan_common/maps/map_scene.h"
#include "plan_common/maps/vector_map/vector_map.h"
#include "module/state_container.h"

namespace ad_byd {
namespace planning {

struct ExtractorProcessorContext {
  PredictionInputFrame::CPtr input_frame = nullptr;
  MapConstPtr map_ptr = nullptr;
  VectorMapConstPtr vector_map_ptr = nullptr;
  MapScenePtr map_scene_ptr = nullptr;
  ObstacleLRUCachePtr dynamic_obs_cache_ptr = nullptr;
  ObstacleMapPtr static_obstacle_map_ptr = nullptr;
  StateContainerPtr state_container_ptr = nullptr;
  CandidateSequencesPtr candidate_sequences_ptr = nullptr;
  std::list<Obstacle*>* obstacles_ptr = nullptr;
  PlanningDataConstPtr prev_planning_data_ptr = nullptr;
  PolylineInfoPtr polyline_info_ptr = nullptr;
  RefRoutingLaneCenterPtr ref_routing_lane_center_ptr = nullptr;
};
struct EvaluatorProcessorContext {
  MapConstPtr map_ptr = nullptr;
  VectorMapConstPtr vector_map_ptr = nullptr;
  PlanningDataVecPtr planning_datas_ptr = nullptr;
  CandidateSequencesConstPtr candidate_sequences_ptr = nullptr;
  std::list<Obstacle*>* obstacles_ptr = nullptr;
  ObstacleMapPtr static_obstacle_map_ptr = nullptr;
  PlanningDataConstPtr prev_planning_data_ptr = nullptr;
  StateContainerConstPtr state_container_ptr = nullptr;
  PolylineInfoConstPtr polyline_info_ptr = nullptr;
  ObstacleLRUCachePtr dynamic_obs_cache_ptr = nullptr;
  NavigableLaneSequencePtr target_lane_sequence_ptr = nullptr;
  RefRoutingLaneCenterPtr ref_routing_lane_center_ptr = nullptr;
};

struct PredictionProcessorContext {
  MapConstPtr map_ptr = nullptr;
  std::list<Obstacle*>* obstacles_ptr = nullptr;
  ObstacleLRUCachePtr dynamic_obs_cache_ptr = nullptr;
  ObstacleMapPtr static_obstacle_map_ptr = nullptr;
  VectorMapConstPtr vector_map_ptr = nullptr;
  RefRoutingLaneCenterPtr ref_routing_lane_center_ptr = nullptr;
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_COMMON_ERROR_H
