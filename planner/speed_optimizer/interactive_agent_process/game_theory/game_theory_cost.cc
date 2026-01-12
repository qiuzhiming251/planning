

#include "planner/speed_optimizer/interactive_agent_process/game_theory/game_theory_cost.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/meta/type_traits.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "plan_common/log.h"
#include "gflags/gflags.h"
#include "plan_common/math/fast_math.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/halfplane.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/intelligent_driver_model.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "object_manager/planner_object.h"
#include "plan_common/log_data.h"
#include "plan_common/maps/overlap_info.h"
#include "plan_common/maps/st_boundary.h"
#include "plan_common/maps/st_point.h"
#include "plan_common/plan_common_defs.h"
#include "plan_common/second_order_trajectory_point.h"
#include "plan_common/speed/st_speed/speed_point.h"
#include "plan_common/speed/st_speed/speed_profile.h"
#include "plan_common/timer.h"
#include "plan_common/trajectory_util.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction_defs.h"
#include "predictor/prediction_object_state.h"
#include "planner/speed_optimizer/decider/interaction_util.h"
#include "planner/speed_optimizer/decider/post_st_boundary_modifier.h"
#include "planner/speed_optimizer/decider/pre_brake_decider.h"
#include "planner/speed_optimizer/decider/st_boundary_modifier_util.h"
#include "planner/speed_optimizer/empty_road_speed.h"
#include "planner/speed_optimizer/gridded_svt_graph.h"
#include "planner/speed_optimizer/st_graph_data.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/vehicle_geometry_util.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

namespace st::planning {

#define DEBUG_EGO_RSS (0)
#define DEBUG_OBJ_RSS (0)

GameTheoryCost::GameTheoryCost() { SetRssStaticParams(); }

GameTheoryCost::~GameTheoryCost() {}

void GameTheoryCost::SetRssStaticParams() {
  this->static_params.rss_params.ego_response_time = 0.1;
  this->static_params.rss_params.ego_longitudinal_acc_max = 2.0;
  this->static_params.rss_params.ego_longitudinal_brake_min = 1.0;
  this->static_params.rss_params.ego_longitudinal_brake_max = 4.0;
  this->static_params.rss_params.ego_lateral_acc_max = 1.0;
  this->static_params.rss_params.ego_lateral_brake_min = 1.0;
  this->static_params.rss_params.ego_lateral_brake_max = 1.0;
  this->static_params.rss_params.ego_lateral_miu = 0.0;

  this->static_params.rss_params.obj_response_time = 0.1;
  this->static_params.rss_params.obj_longitudinal_acc_max = 2.0;
  this->static_params.rss_params.obj_longitudinal_brake_min = 1.0;
  this->static_params.rss_params.obj_longitudinal_brake_max = 4.0;
  this->static_params.rss_params.obj_lateral_acc_max = 1.0;
  this->static_params.rss_params.obj_lateral_brake_min = 1.0;
  this->static_params.rss_params.obj_lateral_brake_max = 1.0;
  this->static_params.rss_params.obj_lateral_miu = 0.0;

  this->static_params.rss_params.over_speed_linear_coeff = 0.005;
  this->static_params.rss_params.over_speed_power_coeff = 0.15;
  this->static_params.rss_params.lack_speed_linear_coeff = 0.006;
  this->static_params.rss_params.lack_speed_power_coeff = 0.16;
  this->static_params.traffic_rule_cost_overtake_linear_coeff = 0.2;
  this->static_params.traffic_rule_cost_overtake_power_coeff = 0.16;
  this->static_params.traffic_rule_cost_follow_linear_coeff = 0.1;
  this->static_params.traffic_rule_cost_follow_power_coeff = 0.05;
  this->static_params.max_step_cost_value = 10.0;
}

void GameTheoryCost::CheckTwoPolygonCollision(
    polygon_collision_info_t *collision_info, bool *is_collision,
    const Polygon2d &base_polygon, const Box2d &target_box,
    const double lat_buffer, const double lon_buffer) {
  collision_info->is_front_collision = false;
  collision_info->is_left_collision = false;
  collision_info->is_rear_collision = false;
  collision_info->is_right_collision = false;
  collision_info->is_valid = false;
  *is_collision = false;

  if (base_polygon.points().size() < 3) return;
  const auto corners =
      target_box.GetCornersWithBufferCounterClockwise(lat_buffer, lon_buffer);
  double min_x = corners[0].x();
  double max_x = corners[0].x();
  double min_y = corners[0].y();
  double max_y = corners[0].y();
  for (const auto &corner : corners) {
    min_x = std::min(min_x, corner.x());
    max_x = std::max(max_x, corner.x());
    min_y = std::min(min_y, corner.y());
    max_y = std::max(max_y, corner.y());
  }
  if (min_x > base_polygon.max_x() || max_x < base_polygon.min_x() ||
      min_y > base_polygon.max_y() || max_y < base_polygon.min_y()) {
    return;
  }

  const auto line_segments = base_polygon.line_segments();
  int i = 0;
  for (int i = 0; i < line_segments.size(); i++) {
    if (!target_box.HasOverlapWithBuffer(line_segments[i], lat_buffer,
                                         lon_buffer)) {
      continue;
    }
    switch (i) {
      case 0:
        collision_info->is_front_collision = true;
        *is_collision = true;
        break;
      case 1:
        collision_info->is_left_collision = true;
        *is_collision = true;
        break;
      case 2:
        collision_info->is_rear_collision = true;
        *is_collision = true;
        break;
      case 3:
        collision_info->is_right_collision = true;
        *is_collision = true;
        break;
      default:
        break;
    }
  }

  return;
}

void GameTheoryCost::GetTwoPolygonCollisionInfo(
    bool *is_collision, polygon_collision_info_t *ego_polygon_collision_info,
    polygon_collision_info_t *obj_polygon_collision_info,
    const Polygon2d &ego_polygon, const Polygon2d &obj_polygon,
    const double ego_length, const double ego_width, const double obj_length,
    const double obj_width, const double ego_theta, const double obj_theta,
    const double lat_buffer, const double lon_buffer) {
  if (nullptr == is_collision || nullptr == ego_polygon_collision_info ||
      nullptr == obj_polygon_collision_info)
    return;

  /* update ego_collision_info */
  Box2d obj_box =
      Box2d(obj_polygon.CircleCenter(), obj_theta, obj_length, obj_width);

  CheckTwoPolygonCollision(ego_polygon_collision_info, is_collision,
                           ego_polygon, obj_box, lat_buffer, lon_buffer);

  /* update obj_collision_info */
  Box2d ego_box =
      Box2d(ego_polygon.CircleCenter(), ego_theta, ego_length, ego_width);

  CheckTwoPolygonCollision(obj_polygon_collision_info, is_collision,
                           obj_polygon, ego_box, lat_buffer, lon_buffer);

  return;
}

void GameTheoryCost::judge_ego_obj_responsibility(
    bool *is_ego_responsibility, bool *is_obj_responsibility,
    const bool is_collision, const game_theory_scenario_t game_theory_scenario,
    const polygon_collision_info_t &ego_polygon_collision_info,
    const polygon_collision_info_t &obj_polygon_collision_info,
    const SecondOrderTrajectoryPoint &ego_center_pose,
    const SecondOrderTrajectoryPoint &obj_center_pose,
    const double half_ego_length, const bool is_debug_cost) {
  if (nullptr == is_ego_responsibility || nullptr == is_obj_responsibility) {
    return;
  }
  *is_ego_responsibility = false;
  *is_obj_responsibility = false;
  if (!is_collision) return;

  switch (game_theory_scenario) {
    case GAME_THEORY_SCENARIO_UNKNOWN:
    case GAME_THEORY_SCENARIO_CROSS_STR_STR:
    case GAME_THEORY_SCENARIO_CROSS_STR_TL:
    case GAME_THEORY_SCENARIO_CROSS_TL_STR:
    case GAME_THEORY_SCENARIO_CROSS_STR_TR:
    case GAME_THEORY_SCENARIO_CROSS_TR_STR:
    case GAME_THEORY_SCENARIO_CROSS_TL_TL:
    case GAME_THEORY_SCENARIO_CROSS_TR_TR:
    case GAME_THEORY_SCENARIO_CROSS_UTURN_STR:
    case GAME_THEORY_SCENARIO_CROSS_STR_UTURN:
    case GAME_THEORY_SCENARIO_MERGE_PARALLEL:
    case GAME_THEORY_SCENARIO_MERGE_STR_TL:
    case GAME_THEORY_SCENARIO_MERGE_TL_STR:
    case GAME_THEORY_SCENARIO_MERGE_STR_TR:
    case GAME_THEORY_SCENARIO_MERGE_TR_STR:
    case GAME_THEORY_SCENARIO_MERGE_TL_TL:
    case GAME_THEORY_SCENARIO_MERGE_TR_TR:
    case GAME_THEORY_SCENARIO_MERGE_UTURN_STR:
    case GAME_THEORY_SCENARIO_MERGE_STR_UTURN:
    case GAME_THEORY_SCENARIO_MERGE_LANE_CHANGE:
    case GAME_THEORY_SCENARIO_ONCOMMING:
    case GAME_THEORY_SCENARIO_FOLLOW:

      if (is_debug_cost) {
        std::cout << " [collision_info - ego]: is_front_collision "
                  << ego_polygon_collision_info.is_front_collision
                  << " is_rear_collision "
                  << ego_polygon_collision_info.is_rear_collision
                  << " is_left_collision "
                  << ego_polygon_collision_info.is_left_collision
                  << " is_right_collision "
                  << ego_polygon_collision_info.is_right_collision << std::endl;
        std::cout << " [collision_info - obj]: is_front_collision "
                  << obj_polygon_collision_info.is_front_collision
                  << " is_rear_collision "
                  << obj_polygon_collision_info.is_rear_collision
                  << " is_left_collision "
                  << obj_polygon_collision_info.is_left_collision
                  << " is_right_collision "
                  << obj_polygon_collision_info.is_right_collision << std::endl;
      }
      if (ego_polygon_collision_info.is_front_collision) {
        *is_ego_responsibility = true;
      } else {
        SecondOrderTrajectoryPoint local_pose;
        CvtPoseGlobalToLocal(&local_pose, &obj_center_pose, &ego_center_pose);
        if (is_debug_cost) {
          std::cout << " [collision_info - ego]: local_pose.x "
                    << local_pose.pos().x() << " half_ego_length "
                    << half_ego_length << std::endl;
        }
        if (local_pose.pos().x() < -0.5 * half_ego_length) {
          *is_obj_responsibility = true;
        } else {
          *is_ego_responsibility = true;
        }
      }
      break;
    default:
      std::cout << " scene error " << std::endl;
  }

  return;
}

lon_dir_t GameTheoryCost::CalAgentsRelaLonDir(const FrenetBox &frenet_box_a,
                                              const FrenetBox &frenet_box_b,
                                              const double ego_theta,
                                              const double obj_theta) {
  lon_dir_t lon_dir = LD_REAR;
  constexpr double theta_threshold =
      ad_byd::planning::Constants::DEG2RAD * 90.0;

  if (lx_fgreater(lx_fabs(ego_theta - obj_theta), theta_threshold)) {
    return LD_FRONT;
  }

  if (frenet_box_a.s_max + 0.3 > frenet_box_b.s_max) {
    lon_dir = LD_REAR;
  } else {
    lon_dir = LD_FRONT;
  }

  return lon_dir;
}

lat_dir_t GameTheoryCost::CalAgentsRelaLatDir(const FrenetBox &frenet_box_a,
                                              const FrenetBox &frenet_box_b) {
  lat_dir_t lat_dir = LAT_LEFT;

  if (frenet_box_a.l_min > frenet_box_b.l_max) {
    lat_dir = LAT_RIGHT;
  } else {
    lat_dir = LAT_LEFT;
  }

  return lat_dir;
}

void GameTheoryCost::CalRssLateralSafeDist(
    double *safe_distance, const double ego_vel, const double agent_vel,
    const lat_dir_t lat_direct, const double ego_lateral_brake_max,
    const double ego_lateral_brake_min, const double ego_lateral_acc_max,
    const double agent_lateral_brake_max, const double agent_lateral_brake_min,
    const double agent_lateral_acc_max, const double response_time,
    const double lateral_miu) {
  if (nullptr == safe_distance) return;

  double result = 0.0;
  double ego_lat_brake_max = lx_fequal(ego_lateral_brake_max, 0.0)
                                 ? LX_EPSILON
                                 : ego_lateral_brake_max;
  double ego_lat_brake_min = lx_fequal(ego_lateral_brake_min, 0.0)
                                 ? LX_EPSILON
                                 : ego_lateral_brake_min;
  double agent_lat_brake_max = lx_fequal(agent_lateral_brake_max, 0.0)
                                   ? LX_EPSILON
                                   : agent_lateral_brake_max;
  double agent_lat_brake_min = lx_fequal(agent_lateral_brake_min, 0.0)
                                   ? LX_EPSILON
                                   : agent_lateral_brake_min;

  double ego_lat_vel_abs = lx_fabs(ego_vel);
  double agent_lat_vel_abs = lx_fabs(agent_vel);
  double distance_correction = lateral_miu;
  double ego_lat_vel_at_resp_time =
      ego_lat_vel_abs + response_time * ego_lateral_acc_max;
  double agent_lat_vel_at_resp_time =
      agent_lat_vel_abs + response_time * agent_lateral_acc_max;
  double ego_active_brake_distance =
      (ego_lat_vel_abs * ego_lat_vel_abs) / (2.0 * ego_lat_brake_max);
  double ego_passive_brake_distance =
      (ego_lat_vel_abs + ego_lat_vel_at_resp_time) * response_time / 2.0 +
      (ego_lat_vel_at_resp_time * ego_lat_vel_at_resp_time) /
          (2.0 * ego_lat_brake_min);
  double agent_active_brake_distance =
      (agent_lat_vel_abs * agent_lat_vel_abs) / (2.0 * agent_lat_brake_max);
  double agent_passive_brake_distance =
      (agent_lat_vel_abs + agent_lat_vel_at_resp_time) * response_time / 2.0 +
      (agent_lat_vel_at_resp_time * agent_lat_vel_at_resp_time) /
          (2.0 * agent_lat_brake_min);

  /* based on ego and agent relation, calculate safe distance */
  if (LAT_RIGHT == lat_direct) {
    if (lx_fless(ego_vel, 0.0) && lx_fless(agent_vel, 0.0)) {
      // -------------------------------
      // ego    vvvvvvvv
      // -------------------------------
      // other  vvvvvvvv
      // -------------------------------
      result = ego_passive_brake_distance - agent_active_brake_distance;
    } else if (lx_fless(ego_vel, 0.0) && !lx_fless(agent_vel, 0.0)) {
      // -------------------------------
      // ego    vvvvvvvv
      // -------------------------------
      // other  ^^^^^^^^
      // -------------------------------
      result = ego_passive_brake_distance + agent_passive_brake_distance;
    } else if (!lx_fless(ego_vel, 0.0) && lx_fless(agent_vel, 0.0)) {
      // -------------------------------
      // ego    ^^^^^^^^
      // -------------------------------
      // other  vvvvvvvv
      // -------------------------------
      result = 0.0;
    } else if (!lx_fless(ego_vel, 0.0) && !lx_fless(agent_vel, 0.0)) {
      // -------------------------------
      // ego    ^^^^^^^^
      // -------------------------------
      // other  ^^^^^^^^
      // -------------------------------
      result = agent_passive_brake_distance - ego_active_brake_distance;
    } else {
      result = 0.0;
    }
  } else if (LAT_LEFT == lat_direct) {
    if (lx_fless(ego_vel, 0.0) && lx_fless(agent_vel, 0.0)) {
      // -------------------------------
      // other    vvvvvvvv
      // -------------------------------
      // ego      vvvvvvvv
      // -------------------------------
      result = agent_passive_brake_distance - ego_active_brake_distance;
    } else if (lx_fless(ego_vel, 0.0) && !lx_fless(agent_vel, 0.0)) {
      // -------------------------------
      // other    ^^^^^^^^
      // -------------------------------
      // ego      vvvvvvvv
      // -------------------------------
      result = 0.0;
    } else if (!lx_fless(ego_vel, 0.0) && lx_fless(agent_vel, 0.0)) {
      // -------------------------------
      // other    vvvvvvvv
      // -------------------------------
      // ego      ^^^^^^^^
      // -------------------------------
      result = ego_passive_brake_distance + agent_passive_brake_distance;
    } else if (!lx_fless(ego_vel, 0.0) && !lx_fless(agent_vel, 0.0)) {
      // -------------------------------
      // other    ^^^^^^^^
      // -------------------------------
      // ego      ^^^^^^^^
      // -------------------------------
      result = ego_passive_brake_distance - agent_active_brake_distance;
    } else {
      result = 0.0;
    }
  }

  result = std::max(result, 0.0);
  result += distance_correction;
  *safe_distance = result;

  return;
}

void GameTheoryCost::CalRssLonSafeVelBoundary(
    double *ego_vel_high, double *ego_vel_low,
    const double lon_relative_distance, const lon_dir_t lon_direct,
    const double agent_vel, const double response_time,
    const double longitudinal_acc_max, const double longitudinal_brake_min,
    const double longitudinal_brake_max) {
  if (nullptr == ego_vel_high || nullptr == ego_vel_low) return;

  double agent_abs_vel = lx_fabs(agent_vel);
  double agent_vel_at_response_time =
      agent_abs_vel + response_time * longitudinal_acc_max;
  double agent_driven_distance;
  double lon_abs_distance = std::max(0.0, lon_relative_distance);

  if (LD_FRONT == lon_direct) {
    if (!lx_fless(agent_vel, 0.0)) {
      // ego ---->(lon_distance) other --->
      // other hard brake
      agent_driven_distance =
          (agent_abs_vel * agent_abs_vel) / (2.0 * longitudinal_brake_max);
      // ego has vel high
      double a = 1.0 / (2.0 * longitudinal_brake_min);
      double b = response_time + (longitudinal_acc_max * response_time /
                                  longitudinal_brake_min);
      double c =
          0.5 *
              (longitudinal_acc_max +
               std::pow(longitudinal_acc_max, 2.0) / longitudinal_brake_min) *
              std::pow(response_time, 2.0) -
          agent_driven_distance - lon_abs_distance;
      *ego_vel_high =
          (-b + std::sqrt(std::pow(b, 2.0) - 4.0 * a * c)) / (2.0 * a);
      *ego_vel_low = 0.0;
    } else {
      // ego ----> <---- other
      agent_driven_distance =
          (agent_abs_vel + agent_vel_at_response_time) / 2.0 * response_time +
          agent_vel_at_response_time * agent_vel_at_response_time /
              (2 * longitudinal_brake_min);
      if (lx_fgreater(agent_driven_distance, lon_abs_distance)) {
        *ego_vel_high = 0.0;
        *ego_vel_low = 0.0;
      } else {
        double a = 1.0 / (2.0 * longitudinal_brake_min);
        double b = response_time + (longitudinal_acc_max * response_time /
                                    longitudinal_brake_min);
        double c =
            0.5 *
                (longitudinal_acc_max +
                 std::pow(longitudinal_acc_max, 2.0) / longitudinal_brake_min) *
                std::pow(response_time, 2.0) -
            (lon_abs_distance - agent_driven_distance);
        *ego_vel_high =
            (-b + std::sqrt(std::pow(b, 2.0) - 4.0 * a * c)) / (2.0 * a);
        *ego_vel_low = 0.0;
      }
    }
  } else {
    if (!lx_fless(agent_vel, 0.0)) {
      // other ---> ego--->
      agent_driven_distance =
          (agent_abs_vel + agent_vel_at_response_time) / 2.0 * response_time +
          agent_vel_at_response_time * agent_vel_at_response_time /
              (2.0 * longitudinal_brake_min);
      if (lx_fless(agent_driven_distance, lon_abs_distance)) {
        *ego_vel_high = std::numeric_limits<double>::max();
        *ego_vel_low = 0.0;
      } else {
        *ego_vel_high = std::numeric_limits<double>::max();
        *ego_vel_low = std::sqrt(2.0 * longitudinal_brake_max *
                                 (agent_driven_distance - lon_abs_distance));
      }
    } else {
      // <----other ego-->
      *ego_vel_high = std::numeric_limits<double>::max();
      *ego_vel_low = 0.0;
    }
  }

  return;
}

void GameTheoryCost::GameTheoryRssSafeCheckEgo(
    bool *is_safe, lon_violate_type_t *lon_violate_type, double *rss_vel_low,
    double *rss_vel_high, const gt_ego_state_t *ego_state,
    const gt_agent_state_t *obj_state, const double ego_width,
    const double obj_width, const bool is_debug_cost) {
  if (nullptr == is_safe || nullptr == lon_violate_type ||
      nullptr == rss_vel_low || nullptr == rss_vel_high ||
      nullptr == ego_state || nullptr == obj_state) {
    return;
  }
  lon_dir_t lon_dir =
      CalAgentsRelaLonDir(ego_state->frenet_box, obj_state->frenet_box,
                          ego_state->theta, obj_state->theta);
  lat_dir_t lat_dir =
      CalAgentsRelaLatDir(ego_state->frenet_box, obj_state->frenet_box);

  bool is_debug_rss = false;
#if DEBUG_EGO_RSS
  is_debug_rss = true;
#endif

  if (is_debug_cost && is_debug_rss) {
    printf(
        "[ ego_safety_cost rss ]: lon_dir %d lat_dir %d "
        "ego_vel_s %.2f \n",
        lon_dir, lat_dir, ego_state->sl_vel.s_dt * 3.6);
  }

  if (ego_state->sl_vel.s_dt < 0.0) {
    *is_safe = true;
    *lon_violate_type = LON_VIO_LEGAL;
    *rss_vel_low = 0.0;
    *rss_vel_high = 0.0;
    return;
  }

  double safe_lat_dist = 0.0;

  if (is_debug_cost && is_debug_rss) {
    printf(
        "[ ego_safety_cost rss ]: ego_vel_l %.2f obj_vel_l %.2f "
        "lat_dir %d\n",
        ego_state->sl_vel.l_dt * 3.6, obj_state->sl_vel.l_dt * 3.6, lat_dir);
  }

  CalRssLateralSafeDist(&safe_lat_dist, ego_state->sl_vel.l_dt,
                        obj_state->sl_vel.l_dt, lat_dir,
                        static_params.rss_params.ego_lateral_brake_max,
                        static_params.rss_params.ego_lateral_brake_min,
                        static_params.rss_params.ego_lateral_acc_max,
                        static_params.rss_params.ego_lateral_brake_max,
                        static_params.rss_params.ego_lateral_brake_min,
                        static_params.rss_params.ego_lateral_acc_max,
                        static_params.rss_params.ego_response_time,
                        static_params.rss_params.ego_lateral_miu);

  safe_lat_dist += 0.5 * (ego_width + obj_width);
  double ego_agent_lateral_abs_dist = lx_fabs(ego_state->frenet_box.center_l() -
                                              obj_state->frenet_box.center_l());

  if (is_debug_cost && is_debug_rss) {
    printf(
        "[ ego_safety_cost rss ]: safe_lat_dist %.2f "
        "ego_agent_lateral_abs_dist %.2f lat_dir %d\n",
        safe_lat_dist, ego_agent_lateral_abs_dist, lat_dir);
  }

  if (!lx_fless(ego_agent_lateral_abs_dist, safe_lat_dist)) {
    *is_safe = true;
    *lon_violate_type = LON_VIO_LEGAL;
    *rss_vel_low = 0.0;
    *rss_vel_high = 0.0;
    return;
  }

  double lon_relative_distance = 0.0;
  if (LD_REAR == lon_dir) {
    lon_relative_distance =
        ego_state->frenet_box.s_min - obj_state->frenet_box.s_max;
  } else if (LD_FRONT == lon_dir) {
    lon_relative_distance =
        obj_state->frenet_box.s_min - ego_state->frenet_box.s_max;
  }

  if (is_debug_cost && is_debug_rss) {
    printf(
        "[ ego_safety_cost rss ]: lon_relative_distance %.2f "
        "lon_dir %d \n",
        lon_relative_distance, lon_dir);
  }

  if (!lx_fgreater(lon_relative_distance, 0.0) && (LD_FRONT == lon_dir)) {
    *is_safe = false;
    *lon_violate_type = LON_VIO_TOOFAST;
    *rss_vel_high = 0.0;
    *rss_vel_low = 0.0;
  }

  double ego_vel_high, ego_vel_low;

  CalRssLonSafeVelBoundary(&ego_vel_high, &ego_vel_low, lon_relative_distance,
                           lon_dir, obj_state->sl_vel.s_dt,
                           static_params.rss_params.ego_response_time,
                           static_params.rss_params.ego_longitudinal_acc_max,
                           static_params.rss_params.ego_longitudinal_brake_min,
                           static_params.rss_params.ego_longitudinal_brake_max);

  if (is_debug_cost && is_debug_rss) {
    printf(
        "[ ego_safety_cost rss ]: obj_vel_s %.2f lon_dir %d "
        "lon_relative_distance %.2f ego_vel_low %.2f "
        "ego_vel_high %.2f\n",
        obj_state->sl_vel.s_dt * 3.6, lon_dir, lon_relative_distance,
        ego_vel_low * 3.6, ego_vel_high * 3.6);
  }

  if (lx_fgreater(ego_state->sl_vel.s_dt, ego_vel_high)) {
    *is_safe = false;
    *lon_violate_type = LON_VIO_TOOFAST;
    *rss_vel_low = ego_vel_low;
    *rss_vel_high = ego_vel_high;
  } else if (lx_fless(ego_state->sl_vel.s_dt, ego_vel_low)) {
    *is_safe = false;
    *lon_violate_type = LON_VIO_TOOSLOW;
    *rss_vel_low = ego_vel_low;
    *rss_vel_high = ego_vel_high;
  } else {
    *is_safe = true;
    *lon_violate_type = LON_VIO_LEGAL;
    *rss_vel_low = 0.0;
    *rss_vel_high = 0.0;
  }

  return;
}

void GameTheoryCost::GameTheoryRssSafeCheckObj(
    bool *is_safe, lon_violate_type_t *lon_violate_type, double *rss_vel_low,
    double *rss_vel_high, const gt_ego_state_t *ego_state,
    const gt_agent_state_t *obj_state, const double ego_width,
    const double obj_width, const bool is_debug_cost) {
  if (nullptr == is_safe || nullptr == lon_violate_type ||
      nullptr == rss_vel_low || nullptr == rss_vel_high ||
      nullptr == ego_state || nullptr == obj_state) {
    return;
  }
  lon_dir_t lon_dir =
      CalAgentsRelaLonDir(obj_state->frenet_box, ego_state->frenet_box,
                          obj_state->theta, ego_state->theta);
  lat_dir_t lat_dir =
      CalAgentsRelaLatDir(obj_state->frenet_box, ego_state->frenet_box);
  bool is_debug_rss = false;
#if DEBUG_OBJ_RSS
  is_debug_rss = true;
#endif
  if (is_debug_cost && is_debug_rss) {
    printf(
        "[ obj_safety_cost rss ]: lon_dir %d lat_dir %d "
        "obj_vel_s %.2f \n",
        lon_dir, lat_dir, obj_state->sl_vel.s_dt * 3.6);
  }
  double safe_lat_dist = 0.0;
  if (is_debug_cost && is_debug_rss) {
    printf(
        "[ obj_safety_cost rss ]: obj_vel_l %.2f ego_vel_l %.2f "
        "lat_dir %d\n",
        obj_state->sl_vel.l_dt * 3.6, ego_state->sl_vel.l_dt * 3.6, lat_dir);
  }
  CalRssLateralSafeDist(&safe_lat_dist, obj_state->sl_vel.l_dt,
                        ego_state->sl_vel.l_dt, lat_dir,
                        static_params.rss_params.obj_lateral_brake_max,
                        static_params.rss_params.obj_lateral_brake_min,
                        static_params.rss_params.obj_lateral_acc_max,
                        static_params.rss_params.obj_lateral_brake_max,
                        static_params.rss_params.obj_lateral_brake_min,
                        static_params.rss_params.obj_lateral_acc_max,
                        static_params.rss_params.obj_response_time,
                        static_params.rss_params.obj_lateral_miu);

  safe_lat_dist += 0.5 * (ego_width + obj_width);
  double agent_ego_lateral_abs_dist = lx_fabs(ego_state->frenet_box.center_l() -
                                              obj_state->frenet_box.center_l());

  if (is_debug_cost && is_debug_rss) {
    printf(
        "[ obj_safety_cost rss ]: safe_lat_dist %.2f "
        "obj_ego_lateral_abs_dist %.2f lat_dir %d\n",
        safe_lat_dist, agent_ego_lateral_abs_dist, lat_dir);
  }
  if (!lx_fless(agent_ego_lateral_abs_dist, safe_lat_dist)) {
    *is_safe = true;
    *lon_violate_type = LON_VIO_LEGAL;
    *rss_vel_low = 0.0;
    *rss_vel_high = 0.0;
    return;
  }
  double lon_relative_distance = 0.0;
  if (LD_REAR == lon_dir) {
    lon_relative_distance =
        obj_state->frenet_box.s_min - ego_state->frenet_box.s_max;
  } else if (LD_FRONT == lon_dir) {
    lon_relative_distance =
        ego_state->frenet_box.s_min - obj_state->frenet_box.s_max;
  }
  lon_relative_distance = std::fabs(lon_relative_distance);
  if (is_debug_cost && is_debug_rss) {
    printf(
        "[ obj_safety_cost rss ]: lon_relative_distance %.2f "
        "lon_dir %d \n",
        lon_relative_distance, lon_dir);
  }
  if (!lx_fgreater(lon_relative_distance, 0.0) && (LD_FRONT == lon_dir)) {
    *is_safe = false;
    *lon_violate_type = LON_VIO_TOOFAST;
    *rss_vel_high = 0.0;
    *rss_vel_low = 0.0;
  }
  double obj_vel_high, obj_vel_low;
  double tmp_ego_vel = ego_state->sl_vel.s_dt;
  if (lx_fless(obj_state->sl_vel.s_dt, 0.0)) {
    tmp_ego_vel = -tmp_ego_vel;
  }
  CalRssLonSafeVelBoundary(&obj_vel_high, &obj_vel_low, lon_relative_distance,
                           lon_dir, tmp_ego_vel,
                           static_params.rss_params.obj_response_time,
                           static_params.rss_params.obj_longitudinal_acc_max,
                           static_params.rss_params.obj_longitudinal_brake_min,
                           static_params.rss_params.obj_longitudinal_brake_max);
  if (is_debug_cost && is_debug_rss) {
    printf(
        "[ obj_safety_cost rss ]: ego_vel_s %.2f lon_dir %d "
        "lon_relative_distance %.2f obj_vel_low %.2f "
        "obj_vel_high %.2f\n",
        ego_state->sl_vel.s_dt * 3.6, lon_dir, lon_relative_distance,
        obj_vel_low * 3.6, obj_vel_high * 3.6);
  }
  double temp_vel = std::max(std::fabs(obj_state->sl_vel.s_dt),
                             std::fabs(obj_state->sl_vel.l_dt));
  if (lx_fgreater(temp_vel, obj_vel_high)) {
    *is_safe = false;
    *lon_violate_type = LON_VIO_TOOFAST;
    *rss_vel_low = obj_vel_low;
    *rss_vel_high = obj_vel_high;
  } else if (lx_fless(temp_vel, obj_vel_low)) {
    *is_safe = false;
    *lon_violate_type = LON_VIO_TOOSLOW;
    *rss_vel_low = obj_vel_low;
    *rss_vel_high = obj_vel_high;
  } else {
    *is_safe = true;
    *lon_violate_type = LON_VIO_LEGAL;
    *rss_vel_low = 0.0;
    *rss_vel_high = 0.0;
  }
  return;
}

TurnLeftCrossCost::TurnLeftCrossCost() {}

TurnLeftCrossCost::~TurnLeftCrossCost() {}

void TurnLeftCrossCost::EgoSafetyCost(gt_ego_state_t *ego_state,
                                      const gt_agent_state_t *obj_state,
                                      const double ego_width,
                                      const double obj_width,
                                      const bool is_debug_cost) {
  bool is_rss_safe;
  lon_violate_type_t lon_violate_type;
  double rss_vel_low, rss_vel_high;
  double step_ego_cost = 0.0;

  GameTheoryRssSafeCheckEgo(&is_rss_safe, &lon_violate_type, &rss_vel_low,
                            &rss_vel_high, ego_state, obj_state, ego_width,
                            obj_width, is_debug_cost);

  if (is_debug_cost) {
    printf(
        "[ ego_safety_cost     ]: is_rss_safe %d lon_violate_type %d "
        "rss_vel_low %.2f rss_vel_high %.2f \n",
        is_rss_safe, lon_violate_type, rss_vel_low * 3.6, rss_vel_high * 3.6);
  }

  if (!is_rss_safe) {
    if (LON_VIO_TOOFAST == lon_violate_type) {
      step_ego_cost =
          static_params.rss_params.over_speed_linear_coeff * ego_state->vel *
          std::pow(10, static_params.rss_params.over_speed_power_coeff *
                           lx_fabs(ego_state->vel - rss_vel_high));
      if (is_debug_cost) {
        printf("[ ego_safety_cost-  F ]: delta_vel %.2f step_ego_cost %.2f \n",
               lx_fabs(ego_state->vel - rss_vel_high) * 3.6, step_ego_cost);
      }
    } else {
      step_ego_cost =
          static_params.rss_params.lack_speed_linear_coeff * ego_state->vel *
          std::pow(10, static_params.rss_params.lack_speed_power_coeff *
                           lx_fabs(ego_state->vel - rss_vel_low));
      if (is_debug_cost) {
        printf("[ ego_safety_cost-  S ]: delta_vel %.2f step_ego_cost %.2f \n",
               lx_fabs(ego_state->vel - rss_vel_low) * 3.6, step_ego_cost);
      }
    }
    step_ego_cost =
        std::clamp(step_ego_cost, 0.0, static_params.max_step_cost_value);
  }

  if (is_debug_cost) {
    printf("[ ego_safety_cost-  R ]: ----R: step_ego_cost %.2f \n",
           step_ego_cost);
  }

  ego_state->safety_cost = step_ego_cost;
}

void TurnLeftCrossCost::EgoTrafficRuleCost(
    gt_ego_state_t *ego_state, const gt_agent_state_t *obj_state,
    const StBoundaryProto::DecisionType decision_type,
    const bool is_ego_responsibility, const bool is_collision,
    const bool is_debug_cost, const double ego_dist_to_enter_intersection,
    const double obj_dist_to_enter_intersection) {
  double ego_time_to_intersection, ego_time_to_leave_intersection;
  double obj_time_to_intersection, obj_time_to_leave_intersection;

  ego_time_to_intersection = this->ego_time_to_intersection;
  ego_time_to_leave_intersection = this->ego_time_to_leave_intersection;
  obj_time_to_intersection = this->obj_time_to_intersection;
  obj_time_to_leave_intersection = this->obj_time_to_leave_intersection;

  double overtake_delta_time =
      ego_time_to_leave_intersection - obj_time_to_intersection;

  double follow_delta_time =
      obj_time_to_intersection - ego_time_to_intersection;

  double traffic_rule_cost;

  double ratio = 10.0;
  if (ego_dist_to_enter_intersection > 0.0) {
    double tmp_val =
        obj_dist_to_enter_intersection / ego_dist_to_enter_intersection;
    ratio = std::exp(tmp_val * tmp_val * tmp_val / 30.0);
  }
  ratio = std::clamp(ratio, 1.0, 10.0);

  if (StBoundaryProto::OVERTAKE == decision_type) {
    overtake_delta_time *= 1.0 / ratio;
    traffic_rule_cost = static_params.traffic_rule_cost_overtake_linear_coeff *
                        ego_state->vel * std::pow(10, overtake_delta_time);
    traffic_rule_cost *= 1.0 / ratio;
    if (obj_dist_to_enter_intersection < 20.0) {
      traffic_rule_cost *= 2.0;
    }
    if (is_debug_cost) {
      printf(
          "[ ego_traffic_cost- O ]: overtake_delta_time %.2f "
          "ego_vel %.2f traffic_rule_cost %.2f ratio %.2f\n",
          overtake_delta_time, ego_state->vel, traffic_rule_cost, 1.0 / ratio);
    }
  } else {
    if (obj_dist_to_enter_intersection < 20.0) {
      ratio = 0.5;
    }
    traffic_rule_cost =
        static_params.traffic_rule_cost_follow_linear_coeff * ego_state->vel *
        std::pow(10, static_params.traffic_rule_cost_follow_power_coeff *
                         follow_delta_time) *
        ratio;
    if (is_debug_cost) {
      printf(
          "[ ego_traffic_cost- F ]: follow_delta_time %.2f "
          "ego_vel %.2f traffic_rule_cost %.2f ratio %.2f\n",
          follow_delta_time, ego_state->vel, traffic_rule_cost, ratio);
    }
  }

  traffic_rule_cost =
      std::clamp(traffic_rule_cost, 0.0, static_params.max_step_cost_value);

  if (is_collision && is_ego_responsibility) {
    if (lx_fequal(ego_state->vel, 0.0)) {
      if (!lx_fless(ego_state->wait_time, 2.0)) {
        traffic_rule_cost += 1000.0;
      }
    } else {
      traffic_rule_cost += 1000.0;
    }
    if (is_debug_cost) {
      printf(
          "[ ego_traffic_cost- C ]: is_collision %d "
          "is_ego_responsibility %d ego_wait_time %f\n",
          is_collision, is_ego_responsibility, ego_state->wait_time);
    }
  }

  if (is_debug_cost) {
    printf("[ ego_traffic_cost- R ]: ----R: traffic_rule_cost %.2f \n",
           traffic_rule_cost);
  }

  ego_state->law_cost = traffic_rule_cost;

  return;
}

void TurnLeftCrossCost::EgoAccelerateCost(
    gt_ego_state_t *ego_state, const gt_agent_state_t *obj_state,
    const StBoundaryProto::DecisionType decision_type,
    const double ego_init_acc, const double ego_sampling_acc,
    const bool is_debug_cost, const double obj_init_acc,
    const double ego_dist_to_enter_intersection,
    const double obj_dist_to_enter_intersection) {
  double acc_cost;

  double delta_acc = lx_fabs(ego_init_acc - ego_sampling_acc);
  double ratio = 1.0;
  if (obj_init_acc > 0.0) {
    ratio = 1.0;
  } else {
    ratio = std::exp(2.0 * obj_init_acc);
  }

  double dist_ratio = 10.0;
  if (ego_dist_to_enter_intersection > 0.0) {
    double tmp_val =
        obj_dist_to_enter_intersection / ego_dist_to_enter_intersection;
    dist_ratio = std::exp(tmp_val * tmp_val * tmp_val / 30.0);
  }
  dist_ratio = std::clamp(dist_ratio, 1.0, 10.0);

  if (StBoundaryProto::OVERTAKE == decision_type) {
    if (lx_fless(ego_state->vel, obj_state->vel) &&
        lx_fgreater(ego_sampling_acc, ego_init_acc)) {
      acc_cost = 0.0;
    } else {
      // acc_cost = 0.1 * ego_state->vel * std::pow(100, 0.2 * delta_acc);
      if (dist_ratio < 2.5) {
        delta_acc *= dist_ratio;
        acc_cost = 0.01 * ego_state->vel * std::pow(2 * delta_acc, 2);
        acc_cost *= dist_ratio;
      } else {
        delta_acc *= 1.0 / dist_ratio;
        acc_cost = 0.01 * ego_state->vel * std::pow(2 * delta_acc, 2);
        acc_cost *= 1.0 / dist_ratio;
      }
    }
    if (is_debug_cost) {
      printf(
          "[ ego_acc_cost-     O ]: ego_vel %.2f obj_vel %.2f "
          "ego_init_acc %.2f ego_sampling_acc %.2f acc_cost %.2f "
          "delta_acc %.2f dist_ratio %.2f\n",
          ego_state->vel, obj_state->vel, ego_init_acc, ego_sampling_acc,
          delta_acc, acc_cost, 1 / ratio);
    }
  } else {
    if (lx_fgreater(ego_state->vel, obj_state->vel) &&
        lx_fless(ego_sampling_acc, ego_init_acc)) {
      acc_cost = 0.0;
    } else {
      // acc_cost = 0.1 * ego_state->vel * std::pow(100, 0.2 * delta_acc);
      acc_cost = 0.01 * ego_state->vel * std::pow(2 * delta_acc, 2);
      acc_cost *= ratio;
    }
    if (is_debug_cost) {
      printf(
          "[ ego_acc_cost-     F ]: ego_vel %.2f obj_vel %.2f "
          "ego_init_acc %.2f ego_sampling_acc %.2f delta_acc %.2f "
          "acc_cost %.2f\n",
          ego_state->vel, obj_state->vel, ego_init_acc, ego_sampling_acc,
          delta_acc, acc_cost);
    }
  }

  if (lx_fequal(ego_state->vel, 0.0)) {
    acc_cost = std::pow(2 * delta_acc, 2);
  }

  acc_cost = std::clamp(acc_cost, 0.0, static_params.max_step_cost_value);

  if (is_debug_cost) {
    printf("[ ego_acc_cost-     R ]: ----R: acc_cost %.2f \n", acc_cost);
  }

  ego_state->acc_cost = acc_cost;

  return;
}

void TurnLeftCrossCost::ObjSafetyCost(
    gt_agent_state_t *obj_state, const gt_ego_state_t *ego_state,
    const double ego_width, const double obj_width, const bool is_debug_cost,
    const double obj_dist_to_enter_intersection) {
  bool is_rss_safe;
  lon_violate_type_t lon_violate_type;
  double rss_vel_low, rss_vel_high;
  double step_obj_cost = 0.0;

  GameTheoryRssSafeCheckObj(&is_rss_safe, &lon_violate_type, &rss_vel_low,
                            &rss_vel_high, ego_state, obj_state, ego_width,
                            obj_width, is_debug_cost);

  if (is_debug_cost) {
    printf(
        "[ obj_safety_cost     ]: is_rss_safe %d lon_violate_type %d "
        "rss_vel_low %.2f rss_vel_high %.2f\n",
        is_rss_safe, lon_violate_type, rss_vel_low * 3.6, rss_vel_high * 3.6);
  }

  if (!is_rss_safe) {
    if (LON_VIO_TOOFAST == lon_violate_type) {
      step_obj_cost =
          static_params.rss_params.over_speed_linear_coeff * obj_state->vel *
          std::pow(10, static_params.rss_params.over_speed_power_coeff *
                           lx_fabs(obj_state->vel - rss_vel_high));
      if (is_debug_cost) {
        printf("[ obj_safety_cost-  F ]: delta_vel %.2f step_obj_cost %.2f\n",
               lx_fabs(obj_state->vel - rss_vel_high) * 3.6, step_obj_cost);
      }
    } else {
      step_obj_cost =
          static_params.rss_params.lack_speed_linear_coeff * obj_state->vel *
          std::pow(10, static_params.rss_params.lack_speed_power_coeff *
                           lx_fabs(obj_state->vel - rss_vel_low));
      if (is_debug_cost) {
        printf("[ obj_safety_cost-  S ]: delta_vel %.2f step_obj_cost %.2f\n",
               lx_fabs(obj_state->vel - rss_vel_low) * 3.6, step_obj_cost);
      }
    }
    step_obj_cost =
        std::clamp(step_obj_cost, 0.0, static_params.max_step_cost_value);
  }

  double ratio = 13.0 / (1.0 + std::exp(obj_dist_to_enter_intersection / 12.0));
  ratio = std::clamp(ratio, 0.0, 1.0);
  step_obj_cost *= ratio;

  if (is_debug_cost) {
    printf("[ obj_safety_cost-  R ]: ----R: step_obj_cost %.2f\n",
           step_obj_cost);
  }

  obj_state->safety_cost = step_obj_cost;

  return;
}

void TurnLeftCrossCost::ObjTrafficRuleCost(
    gt_agent_state_t *obj_state, const gt_ego_state_t *ego_state,
    const StBoundaryProto::DecisionType decision_type,
    const bool is_obj_responsibility, const bool is_collision,
    const bool is_debug_cost, const double obj_dist_to_enter_intersection) {
  double ego_time_to_intersection, ego_time_to_leave_intersection;
  double obj_time_to_intersection, obj_time_to_leave_intersection;

  ego_time_to_intersection = this->ego_time_to_intersection;
  ego_time_to_leave_intersection = this->ego_time_to_leave_intersection;
  obj_time_to_intersection = this->obj_time_to_intersection;
  obj_time_to_leave_intersection = this->obj_time_to_leave_intersection;

  double overtake_delta_time =
      ego_time_to_intersection - obj_time_to_intersection;

  double follow_delta_time =
      obj_time_to_intersection - ego_time_to_intersection;

  double traffic_rule_cost;

  if (StBoundaryProto::OVERTAKE == decision_type) {
    traffic_rule_cost =
        0.5 *
        std::pow(10, static_params.traffic_rule_cost_overtake_power_coeff *
                         overtake_delta_time);
    if (is_debug_cost) {
      printf(
          "[ obj_traffic_cost- O ]: overtake_delta_time %.2f "
          "obj_vel %.2f traffic_rule_cost %.2f \n",
          overtake_delta_time, obj_state->vel, traffic_rule_cost);
    }
  } else {
    // traffic_rule_cost = 0.2 *
    //         std::pow(10, static_params.traffic_rule_cost_follow_power_coeff *
    //             follow_delta_time);
    traffic_rule_cost = 0.05 * std::pow(10, 0.4 * follow_delta_time);
    if (is_debug_cost) {
      printf(
          "[ obj_traffic_cost- F ]: follow_delta_time %.2f "
          "obj_vel %.2f traffic_rule_cost %.2f \n",
          follow_delta_time, obj_state->vel, traffic_rule_cost);
    }
  }

  double ratio = 13.0 / (1.0 + std::exp(obj_dist_to_enter_intersection / 12.0));
  ratio = std::clamp(ratio, 0.0, 1.0);
  traffic_rule_cost *= ratio;

  traffic_rule_cost =
      std::clamp(traffic_rule_cost, 0.0, static_params.max_step_cost_value);

  if (is_collision && is_obj_responsibility &&
      !lx_fequal(ego_state->vel, 0.0)) {
    traffic_rule_cost += 1000.0;
    if (is_debug_cost) {
      printf(
          "[ obj_traffic_cost- C ]: is_collision %d "
          "is_obj_responsibility %d\n",
          is_collision, is_obj_responsibility);
    }
  }

  if (is_debug_cost) {
    printf("[ obj_traffic_cost- R ]: ----R: traffic_rule_cost %.2f \n",
           traffic_rule_cost);
  }
  obj_state->law_cost = traffic_rule_cost;

  return;
}

void TurnLeftCrossCost::ObjAccelerateCost(
    gt_agent_state_t *obj_state, const double obj_init_acc,
    const double obj_sampling_acc, const bool is_debug_cost,
    const double ego_dist_to_enter_intersection,
    const double obj_dist_to_enter_intersection) {
  double delta_acc = lx_fabs(obj_init_acc - obj_sampling_acc);
  // double acc_cost = 0.1 * obj_state->vel *  std::pow(100, 0.2 * delta_acc);
  double acc_cost = 0.03 * obj_state->vel * std::pow(2 * delta_acc, 2);

  double dist_ratio = 3.0;
  if (ego_dist_to_enter_intersection > 0.0) {
    dist_ratio =
        obj_dist_to_enter_intersection / ego_dist_to_enter_intersection;
  }

  if (obj_init_acc > 0 && !lx_fgreater(dist_ratio, 3.0)) {
    if (lx_fequal(obj_state->vel, 0.0)) {
      acc_cost = std::pow(2 * delta_acc, 2);
    }
  }

  double ratio = 13.0 / (1.0 + std::exp(obj_dist_to_enter_intersection / 12.0));

  ratio = std::clamp(ratio, 0.0, 1.0);

  if (obj_sampling_acc < 0) {
    acc_cost = ratio * acc_cost;
  }

  acc_cost = std::clamp(acc_cost, 0.0, static_params.max_step_cost_value);

  if (is_debug_cost) {
    printf(
        "[ obj_acc_cost        ]: obj_init_acc %.2f obj_sampling_acc %.2f "
        "delta_acc %.2f acc_cost %.2f ratio %.2f\n",
        obj_init_acc, obj_sampling_acc, delta_acc, acc_cost, ratio);
    printf("[ obj_acc_cost-     R ]: ----R: acc_cost %.2f\n", acc_cost);
  }
  obj_state->acc_cost = acc_cost;

  return;
}

void TurnLeftCrossCost::CalcTimeToCollision(
    const gt_ego_state_t *ego_state, const gt_agent_state_t *obj_state,
    const double ego_dist_to_enter_intersection,
    const double ego_dist_to_leave_intersection,
    const double obj_dist_to_enter_intersection,
    const double obj_dist_to_leave_intersection, const bool is_debug_cost) {
  this->ego_time_to_intersection = 1000;
  this->ego_time_to_leave_intersection = 1000;
  this->obj_time_to_intersection = 1000;
  this->obj_time_to_leave_intersection = 1000;

  if (!lx_fequal(ego_state->vel, 0.0)) {
    this->ego_time_to_intersection =
        std::max(ego_dist_to_enter_intersection - ego_state->dist, 0.0) /
        ego_state->vel;
    this->ego_time_to_leave_intersection =
        std::max(ego_dist_to_leave_intersection - ego_state->dist, 0.0) /
        ego_state->vel;
  }

  if (!lx_fequal(obj_state->vel, 0.0)) {
    this->obj_time_to_intersection =
        std::max(obj_dist_to_enter_intersection - obj_state->dist, 0.0) /
        obj_state->vel;
    this->obj_time_to_leave_intersection =
        std::max(obj_dist_to_leave_intersection - obj_state->dist, 0.0) /
        obj_state->vel;
  }

  if (is_debug_cost) {
    printf(
        "[ time_to_collision   ]: ego_vel %.2f ego_dist_to_enter %.2f "
        " ego_dist_to_leave_enter %.2f ego_time_to_collision %.2f "
        "ego_time_to_leave_collision %.2f \n",
        ego_state->vel * 3.6, ego_dist_to_enter_intersection - ego_state->dist,
        ego_dist_to_leave_intersection - ego_state->dist,
        this->ego_time_to_intersection, this->ego_time_to_leave_intersection);
    printf(
        "[ time_to_collision   ]: obj_vel %.2f obj_dist_to_enter %.2f "
        " obj_dist_to_leave_enter %.2f obj_time_to_collision %.2f "
        "obj_time_to_leave_collision %.2f \n",
        obj_state->vel * 3.6, obj_dist_to_enter_intersection - obj_state->dist,
        obj_dist_to_leave_intersection - obj_state->dist,
        this->obj_time_to_intersection, this->obj_time_to_leave_intersection);
  }

  return;
}

void TurnLeftCrossCost::GameTheoryCostUpdate(
    gt_ego_state_t *ego_state, gt_agent_state_t *obj_state,
    const double ego_length, const double ego_width, const double obj_length,
    const double obj_width, const double ego_dist_to_enter_intersection,
    const double ego_dist_to_leave_intersection,
    const double obj_dist_to_enter_intersection,
    const double obj_dist_to_leave_intersection,
    const StBoundaryProto::DecisionType decision_type,
    const double ego_init_acc, const double obj_init_acc,
    const double ego_sampling_acc, const double obj_sampling_acc,
    const int step, const bool is_debug_cost,
    const game_theory_scenario_t game_theory_scenario) {
  if (is_debug_cost) {
    printf(" step: %d | ego_acc: %.2f | obj_acc: %.2f | decision: %d \n", step,
           ego_sampling_acc, obj_sampling_acc, decision_type);
  }

  this->CalcTimeToCollision(
      ego_state, obj_state, ego_dist_to_enter_intersection,
      ego_dist_to_leave_intersection, obj_dist_to_enter_intersection,
      obj_dist_to_leave_intersection, is_debug_cost);

  bool is_collision;
  polygon_collision_info_t ego_polygon_collision_info,
      obj_polygon_collision_info;
  this->GetTwoPolygonCollisionInfo(
      &is_collision, &ego_polygon_collision_info, &obj_polygon_collision_info,
      ego_state->global_polygon, obj_state->global_polygon, ego_length,
      ego_width, obj_length, obj_width, ego_state->theta, obj_state->theta, 0.2,
      0.2);

  bool is_ego_responsibility, is_obj_responsibility;

  this->judge_ego_obj_responsibility(
      &is_ego_responsibility, &is_obj_responsibility, is_collision,
      game_theory_scenario, ego_polygon_collision_info,
      obj_polygon_collision_info, ego_state->global_center_pose,
      obj_state->global_center_pose, ego_length * 0.5, is_debug_cost);

  this->EgoSafetyCost(ego_state, obj_state, ego_width, obj_width,
                      is_debug_cost);
  this->EgoTrafficRuleCost(ego_state, obj_state, decision_type,
                           is_ego_responsibility, is_collision, is_debug_cost,
                           ego_dist_to_enter_intersection,
                           obj_dist_to_enter_intersection);
  this->EgoAccelerateCost(ego_state, obj_state, decision_type, ego_init_acc,
                          ego_sampling_acc, is_debug_cost, obj_init_acc,
                          ego_dist_to_enter_intersection,
                          obj_dist_to_enter_intersection);
  this->ObjSafetyCost(obj_state, ego_state, ego_width, obj_width, is_debug_cost,
                      obj_dist_to_enter_intersection);
  this->ObjTrafficRuleCost(obj_state, ego_state, decision_type,
                           is_obj_responsibility, is_collision, is_debug_cost,
                           obj_dist_to_enter_intersection);
  this->ObjAccelerateCost(obj_state, obj_init_acc, obj_sampling_acc,
                          is_debug_cost, ego_dist_to_enter_intersection,
                          obj_dist_to_enter_intersection);
  if (is_debug_cost) {
    printf("\n");
  }
}

}  // namespace st::planning
