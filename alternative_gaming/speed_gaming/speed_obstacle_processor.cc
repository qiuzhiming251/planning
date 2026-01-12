/**
 * @file speed_obstacle_processor.cc
 * @author zhang.xing (zhang.xing43@byd.com)
 * @brief
 * @version 0.1
 * @date 2025-03-19
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "speed_obstacle_processor.h"
#include "plan_common/math/fast_math.h"
#include "plan_common/log_data.h"
namespace st::planning {
constexpr double kObstaclePathMaxLength = 100.0;
constexpr double kSearchBuffer = 0.2;  // m.
constexpr double kMaxLatDist = 1.5;    // m.
constexpr double kEps = 1e-4;
void SpeedObstacleProcessor::PreProcess(
    std::unordered_map<std::string, GamingConflictZoneInfo>
        &conflict_zone_infos,
    SpacetimeTrajectoryManager *const modified_traj_mgr) {
  conflict_zone_infos.clear();
  RefinePredTraj(*traj_mgr_, modified_traj_mgr);  // TODO: 待预测提升后去除

  // step1: 构造自车path kd tree和segment
  constexpr double kPathApproxTolerance = 0.05;  // m.
  const auto vehicle_rect =
      CreateOffsetRectFromVehicleGeometry(*vehicle_geo_params_);
  std::vector<Vec2d> points;
  points.reserve(ego_path_->size());
  for (const auto &point : *ego_path_) {
    points.emplace_back(point.x(), point.y());
  }
  const auto ego_path_kd_tree = std::make_unique<SegmentMatcherKdtree>(points);
  path_approx_ = std::make_unique<PathApprox>(BuildPathApprox(
      *ego_path_, vehicle_rect, kPathApproxTolerance, ego_path_kd_tree.get()));
  const auto av_sl = drive_passage_->QueryFrenetCoordinateAt(
      Vec2d(points.front().x(), points.front().y()));

  // step2: 遍历动态障碍车轨迹进行交叉检验和分类
  const auto &moving_spacetime_objects =
      modified_traj_mgr->moving_object_trajs();
  const double ego_radius =
      Hypot(std::max(vehicle_geo_params_->front_edge_to_center(),
                     vehicle_geo_params_->back_edge_to_center()),
            vehicle_geo_params_->right_edge_to_center());
  const double path_step_length = (*ego_path_)[1].s() - (*ego_path_)[0].s();
  for (const SpacetimeObjectTrajectory *origin_st_traj :
       moving_spacetime_objects) {
    if (ToStBoundaryObjectType(origin_st_traj->planner_object().type()) ==
        StBoundaryProto::IGNORABLE) {
      continue;
    }
    auto st_traj = SpeedGamingCommon::genThetaValidTraj(*origin_st_traj);
    //过滤自车后方障碍物
    const double kBackConsiderLength = 30.0;
    const Box2d &obj_box = st_traj.bounding_box();
    const auto obj_sl =
        drive_passage_->QueryFrenetCoordinateAt(st_traj.pose().pos());
    if (obj_sl.ok() &&
        ((obj_sl->s - av_sl->s < -kBackConsiderLength) ||
         (obj_sl->s < av_sl->s &&
          abs(obj_sl->l - av_sl->l) <
              0.5 * vehicle_geo_params_->width() + obj_box.half_width()))) {
      continue;
    }

    // 2.1 从障碍物预测轨迹抽取path
    const DiscretizedPath obj_path = SpeedGamingCommon::GeneratePathBasedTraj(
        &st_traj, kObstaclePathMaxLength, false);

    // 2.2 path交叉检测
    const double obj_radius =
        obj_box.diagonal() * 0.5 + st_traj.required_lateral_gap();
    const double search_radius = obj_radius + ego_radius + kSearchBuffer;
    const auto &object_states = st_traj.states();

    std::optional<std::pair<double, double>> first_overlap_ego_s_range;
    int first_overlap_obj_index = -1;
    std::optional<std::pair<double, double>> last_overlap_ego_s_range;
    int last_overlap_obj_index = -1;
    PathConflictCheck(&st_traj, search_radius, *path_approx_, path_step_length,
                      first_overlap_ego_s_range, first_overlap_obj_index,
                      last_overlap_ego_s_range, last_overlap_obj_index);

    // 2.3障碍物分类
    //存在path conflict
    if (first_overlap_ego_s_range.has_value()) {
      const auto &first_overlap_obj_state =
          object_states[first_overlap_obj_index];
      const auto &last_overlap_obj_state =
          object_states[last_overlap_obj_index];
      const auto interaction_type = InteractionClassification(
          last_overlap_obj_state, last_overlap_ego_s_range->second);
      const auto first_state_frenet =
          ego_path_->XYToSL(first_overlap_obj_state.contour,
                            first_overlap_obj_state.box.center());
      const bool kFilterNearCrossObj = true;
      //对于切入点在当前自车车头后，交互类型为cross的障碍物不处理
      if (kFilterNearCrossObj && interaction_type == InteractionType::kCross &&
          first_state_frenet.s_max <
              vehicle_geo_params_->front_edge_to_center()) {
        continue;
      }

      GamingConflictZoneInfo conflict_zone_info;
      conflict_zone_info.obj_traj_id = st_traj.traj_id();
      // ego_view,ego只能确定s,不能确定t,agent根据初始预测轨迹确定t,TODO:ego_view和agent_view是否有信息重复？
      conflict_zone_info.conflict_zone_in_ego_view.is_path_conflict = true;
      conflict_zone_info.conflict_zone_in_ego_view.ego_cutin_s =
          std::min(first_overlap_ego_s_range.value().first,
                   last_overlap_ego_s_range.value().first);
      conflict_zone_info.conflict_zone_in_ego_view.ego_cutout_s =
          std::max(first_overlap_ego_s_range.value().second,
                   last_overlap_ego_s_range.value().second);
      conflict_zone_info.conflict_zone_in_ego_view.agent_cutin_time =
          first_overlap_obj_state.traj_point->t();
      conflict_zone_info.conflict_zone_in_ego_view.agent_cutin_s =
          first_overlap_obj_state.traj_point->s();
      conflict_zone_info.conflict_zone_in_ego_view.agent_cutout_time =
          last_overlap_obj_state.traj_point->t();
      conflict_zone_info.conflict_zone_in_ego_view.agent_cutout_s =
          last_overlap_obj_state.traj_point->s();
      // agent,ego只能确定s,不能确定t,agent根据初始预测轨迹确定t
      conflict_zone_info.conflict_zone_in_agent_view.is_path_conflict = true;
      conflict_zone_info.conflict_zone_in_agent_view.ego_cutin_s =
          std::min(first_overlap_ego_s_range.value().first,
                   last_overlap_ego_s_range.value().first);
      conflict_zone_info.conflict_zone_in_agent_view.ego_cutout_s =
          std::max(first_overlap_ego_s_range.value().second,
                   last_overlap_ego_s_range.value().second);
      conflict_zone_info.conflict_zone_in_agent_view.agent_cutin_time =
          first_overlap_obj_state.traj_point->t();
      conflict_zone_info.conflict_zone_in_agent_view.agent_cutin_s =
          first_overlap_obj_state.traj_point->s();
      conflict_zone_info.conflict_zone_in_agent_view.agent_cutout_time =
          last_overlap_obj_state.traj_point->t();
      conflict_zone_info.conflict_zone_in_agent_view.agent_cutout_s =
          last_overlap_obj_state.traj_point->s();

      if (first_overlap_obj_index == 0) {
        // 0时刻就conflict
        conflict_zone_info.is_pre_gaming = true;
      }
      conflict_zone_info.interaction_type = interaction_type;
      conflict_zone_infos.insert(
          {conflict_zone_info.obj_traj_id, conflict_zone_info});
    } else {  //进行风险场conflict判断,只针对turnmerge
      const auto interaction_type =
          InteractionClassification(object_states.back(), ego_path_->length());
      if (interaction_type == InteractionType::kTurnMerge) {
        // ego_view check
        std::vector<Polygon2d> obj_polygons;
        obj_polygons.reserve(object_states.size());
        std::vector<Box2d> obj_boxs;
        obj_boxs.reserve(object_states.size());
        for (auto &state : object_states) {
          obj_polygons.push_back(state.contour);
          obj_boxs.push_back(state.box);
        }
        const double ego_l0 = 1.0;                           // TODO待标定
        const double kRiskfiledWidthSCoeffForEgo = 0.1;      // TODO待标定
        const double kRiskfiledWidthThetaCoeffForEgo = 0.1;  // TODO待标定
        std::optional<size_t> first_risk_conflict_obj_idx;
        std::optional<size_t> last_risk_conflict_obj_idx;
        PathPoint first_risk_conflict_ego_point;
        PathPoint last_risk_conflict_ego_point;
        std::optional<double> risk_field_lat_ratio_ego_view;
        std::vector<std::pair<double, double>> ego_left_risk_edge;
        std::vector<std::pair<double, double>> ego_right_risk_edge;
        RiskFieldConflictCheck(
            ego_path_, &obj_polygons, &obj_boxs,
            0.5 * vehicle_geo_params_->width(), ego_l0,
            kRiskfiledWidthSCoeffForEgo, kRiskfiledWidthThetaCoeffForEgo,
            first_risk_conflict_obj_idx, last_risk_conflict_obj_idx,
            first_risk_conflict_ego_point, last_risk_conflict_ego_point,
            risk_field_lat_ratio_ego_view, ego_left_risk_edge,
            ego_right_risk_edge);

        // agent_view check
        std::vector<Polygon2d> ego_polygons;
        ego_polygons.reserve(ego_path_->size());
        std::vector<Box2d> ego_boxs;
        ego_boxs.reserve(ego_path_->size());
        for (auto &path_pt : (*ego_path_)) {
          const Vec2d pt_center(path_pt.x(), path_pt.y());
          const Vec2d tangent = Vec2d::FastUnitFromAngle(path_pt.theta());
          const Vec2d center =
              pt_center +
              tangent * (0.5 * vehicle_geo_params_->length() -
                         vehicle_geo_params_->back_edge_to_center());
          const Box2d pt_box(center, path_pt.theta(),
                             vehicle_geo_params_->length(),
                             vehicle_geo_params_->width());
          const Polygon2d pt_polygon(pt_box);
          ego_polygons.push_back(pt_polygon);
          ego_boxs.push_back(pt_box);
        }
        const double agent_l0 = 1.0;                           // TODO待标定
        constexpr double kRiskfiledWidthSCoeffForAgent = 0.1;  // TODO待标定
        constexpr double kRiskfiledWidthThetaCoeffForAgent = 0.1;  // TODO待标定
        std::optional<size_t> first_risk_conflict_ego_idx;
        std::optional<size_t> last_risk_conflict_ego_idx;
        PathPoint first_risk_conflict_agent_point;
        PathPoint last_risk_conflict_agent_point;
        std::optional<double> risk_field_lat_ratio_agent_view;
        std::vector<std::pair<double, double>> agent_left_risk_edge;
        std::vector<std::pair<double, double>> agent_right_risk_edge;
        RiskFieldConflictCheck(
            &obj_path, &ego_polygons, &ego_boxs, 0.5 * obj_box.width(),
            agent_l0, kRiskfiledWidthSCoeffForAgent,
            kRiskfiledWidthThetaCoeffForAgent, first_risk_conflict_ego_idx,
            last_risk_conflict_ego_idx, first_risk_conflict_agent_point,
            last_risk_conflict_agent_point, risk_field_lat_ratio_agent_view,
            agent_left_risk_edge, agent_right_risk_edge);

        //互相都侵入对方的风险场，填充st信息
        if (first_risk_conflict_obj_idx.has_value() &&
            first_risk_conflict_ego_idx.has_value()) {
          GamingConflictZoneInfo conflict_zone_info;
          conflict_zone_info.obj_traj_id = st_traj.traj_id();
#ifdef ALTERNATIVE_GAMING_UNIT_TEST
          conflict_zone_info.ego_left_risk_edge = std::move(ego_left_risk_edge);
          conflict_zone_info.ego_right_risk_edge =
              std::move(ego_right_risk_edge);
          conflict_zone_info.agent_left_risk_edge =
              std::move(agent_left_risk_edge);
          conflict_zone_info.agent_right_risk_edge =
              std::move(agent_right_risk_edge);
#endif
          // ego_view,ego只能确定s,不能确定t,agent根据初始预测轨迹确定t,TODO:ego_view和agent_view是否有信息重复？
          const auto &first_risk_conflict_obj_state =
              object_states[first_risk_conflict_obj_idx.value()];
          const auto &last_risk_conflict_obj_state =
              object_states[last_risk_conflict_obj_idx.value()];
          conflict_zone_info.conflict_zone_in_ego_view.is_risk_field_conflict =
              true;
          conflict_zone_info.conflict_zone_in_ego_view.risk_field_lat_ratio =
              risk_field_lat_ratio_ego_view.value();
          conflict_zone_info.conflict_zone_in_ego_view.ego_cutin_s =
              std::min(first_risk_conflict_ego_point.s(),
                       last_risk_conflict_ego_point.s());
          conflict_zone_info.conflict_zone_in_ego_view.ego_cutout_s =
              std::max(first_risk_conflict_ego_point.s(),
                       last_risk_conflict_ego_point.s());
          conflict_zone_info.conflict_zone_in_ego_view.agent_cutin_time =
              first_risk_conflict_obj_state.traj_point->t();
          conflict_zone_info.conflict_zone_in_ego_view.agent_cutin_s =
              first_risk_conflict_obj_state.traj_point->s();
          conflict_zone_info.conflict_zone_in_ego_view.agent_cutout_time =
              last_risk_conflict_obj_state.traj_point->t();
          conflict_zone_info.conflict_zone_in_ego_view.agent_cutout_s =
              last_risk_conflict_obj_state.traj_point->s();
          // agent,ego只能确定s,不能确定t,agent根据初始预测轨迹确定t
          conflict_zone_info.conflict_zone_in_agent_view
              .is_risk_field_conflict = true;
          conflict_zone_info.conflict_zone_in_agent_view.risk_field_lat_ratio =
              risk_field_lat_ratio_agent_view.value();
          conflict_zone_info.conflict_zone_in_agent_view.ego_cutin_s =
              (*ego_path_)[first_risk_conflict_ego_idx.value()].s();
          conflict_zone_info.conflict_zone_in_agent_view.ego_cutout_s =
              (*ego_path_)[last_risk_conflict_ego_idx.value()].s();

          auto GetLowerIdx = [&obj_path](double s) -> size_t {
            auto it = std::lower_bound(
                obj_path.begin(), obj_path.end(), s,
                [](const PathPoint &p, double s) { return p.s() < s; });
            return std::distance(obj_path.begin(), it);
          };
          size_t first_risk_conflict_agent_idx =
              GetLowerIdx(first_risk_conflict_agent_point.s());
          size_t last_risk_conflict_agent_idx =
              GetLowerIdx(last_risk_conflict_agent_point.s());
          if (first_risk_conflict_agent_idx > last_risk_conflict_agent_idx) {
            std::swap(first_risk_conflict_agent_idx,
                      last_risk_conflict_agent_idx);
          }
          if (first_risk_conflict_agent_idx > 0) {
            first_risk_conflict_agent_idx--;
          }
          conflict_zone_info.conflict_zone_in_agent_view.agent_cutin_time =
              object_states[first_risk_conflict_agent_idx].traj_point->t();
          conflict_zone_info.conflict_zone_in_agent_view.agent_cutin_s =
              object_states[first_risk_conflict_agent_idx].traj_point->s();
          conflict_zone_info.conflict_zone_in_agent_view.agent_cutout_time =
              object_states[last_risk_conflict_agent_idx].traj_point->t();
          conflict_zone_info.conflict_zone_in_agent_view.agent_cutout_s =
              object_states[last_risk_conflict_agent_idx].traj_point->s();
          conflict_zone_info.interaction_type = interaction_type;
          conflict_zone_infos.insert(
              {conflict_zone_info.obj_traj_id, conflict_zone_info});
        }
      }
    }
  }

  // step3: 遍历静态障碍物进行交叉检验
  const auto &stationary_spacetime_objects =
      modified_traj_mgr->stationary_object_trajs();
  for (const SpacetimeObjectTrajectory *st_traj :
       stationary_spacetime_objects) {
    if (ToStBoundaryObjectType(st_traj->planner_object().type()) ==
        StBoundaryProto::IGNORABLE) {
      continue;
    }
    const Box2d &obj_box = st_traj->bounding_box();
    const double obj_radius =
        obj_box.diagonal() * 0.5 + st_traj->required_lateral_gap();
    const double search_radius = obj_radius + ego_radius + kSearchBuffer;

    const auto agent_overlaps = ComputeAgentOverlapsWithBuffer(
        *path_approx_, path_step_length, /*first_index=*/0,
        /*last_index=*/ego_path_->size() - 1, st_traj->states()[0].contour,
        /*max_lat_dist=*/kMaxLatDist, st_traj->required_lateral_gap(),
        st_traj->required_lateral_gap(), search_radius);
    auto overlap_ego_s_range = ConvertToOverlapRange(agent_overlaps);
    if (overlap_ego_s_range.has_value()) {
      GamingConflictZoneInfo conflict_zone_info;
      conflict_zone_info.obj_traj_id = st_traj->traj_id();
      conflict_zone_info.conflict_zone_in_ego_view.is_path_conflict = true;
      conflict_zone_info.conflict_zone_in_ego_view.ego_cutin_s =
          overlap_ego_s_range.value().first;
      conflict_zone_info.conflict_zone_in_ego_view.ego_cutout_s =
          overlap_ego_s_range.value().first;
      conflict_zone_info.conflict_zone_in_ego_view.agent_cutin_time = 0.0;
      conflict_zone_info.conflict_zone_in_ego_view.agent_cutin_s = 0.0;
      conflict_zone_info.conflict_zone_in_ego_view.agent_cutout_time = 10.0;
      conflict_zone_info.conflict_zone_in_ego_view.agent_cutout_s = 0.0;
      conflict_zone_info.conflict_zone_in_agent_view.is_path_conflict = true;
      conflict_zone_info.conflict_zone_in_agent_view.ego_cutin_s =
          overlap_ego_s_range.value().first;
      conflict_zone_info.conflict_zone_in_agent_view.ego_cutout_s =
          overlap_ego_s_range.value().first;
      conflict_zone_info.conflict_zone_in_agent_view.agent_cutin_time = 0.0;
      conflict_zone_info.conflict_zone_in_agent_view.agent_cutin_s = 0.0;
      conflict_zone_info.conflict_zone_in_agent_view.agent_cutout_time = 10.0;
      conflict_zone_info.conflict_zone_in_agent_view.agent_cutout_s = 0.0;
      conflict_zone_info.is_pre_gaming = true;
      conflict_zone_info.interaction_type = InteractionType::kStaticOccupy;
      conflict_zone_infos.insert(
          {conflict_zone_info.obj_traj_id, conflict_zone_info});
    }
  }
}
void SpeedObstacleProcessor::RiskFieldGenerate(
    std::unordered_map<std::string, GamingConflictZoneInfo>
        &conflict_zone_infos,
    const std::vector<StBoundaryWithDecision> &st_boundaries_with_decision) {
  //进行风险场conflict判断,只针对turnmerge
  std::unordered_set<std::string> path_conflict_trajs;
  for (auto &st_boundarie_decision : st_boundaries_with_decision) {
    const auto st_boundary = st_boundarie_decision.raw_st_boundary();
    if (!st_boundary->is_stationary() && st_boundary->traj_id().has_value()) {
      size_t pos = st_boundary->traj_id()->find("|");
      std::string traj_id = pos != std::string::npos
                                ? st_boundary->traj_id()->substr(0, pos)
                                : st_boundary->traj_id().value();
      path_conflict_trajs.insert(traj_id);
    }
  }
  const auto &moving_spacetime_objects = traj_mgr_->moving_object_trajs();
  for (const SpacetimeObjectTrajectory *st_traj : moving_spacetime_objects) {
    if (path_conflict_trajs.count(st_traj->traj_id())) {
      continue;
    }
    if (ToStBoundaryObjectType(st_traj->planner_object().type()) ==
        StBoundaryProto::IGNORABLE) {
      continue;
    }
    const auto &object_states = st_traj->states();
    const Box2d &obj_box = st_traj->bounding_box();
    const DiscretizedPath obj_path = SpeedGamingCommon::GeneratePathBasedTraj(
        st_traj, kObstaclePathMaxLength, false);
    const auto interaction_type = InteractionClassification(object_states);
    if (interaction_type == InteractionType::kTurnMerge) {
      // ego_view check
      std::vector<Polygon2d> obj_polygons;
      obj_polygons.reserve(object_states.size());
      std::vector<Box2d> obj_boxs;
      obj_boxs.reserve(object_states.size());
      for (auto &state : object_states) {
        obj_polygons.push_back(state.contour);
        obj_boxs.push_back(state.box);
      }
      const double ego_l0 = 1.0;                           // TODO待标定
      const double kRiskfiledWidthSCoeffForEgo = 0.3;      // TODO待标定
      const double kRiskfiledWidthThetaCoeffForEgo = 0.3;  // TODO待标定
      std::optional<size_t> first_risk_conflict_obj_idx;
      std::optional<size_t> last_risk_conflict_obj_idx;
      PathPoint first_risk_conflict_ego_point;
      PathPoint last_risk_conflict_ego_point;
      std::optional<double> risk_field_lat_ratio_ego_view;
      std::vector<std::pair<double, double>> ego_left_risk_edge;
      std::vector<std::pair<double, double>> ego_right_risk_edge;
      RiskFieldConflictCheck(
          ego_path_, &obj_polygons, &obj_boxs,
          0.5 * vehicle_geo_params_->width(), ego_l0,
          kRiskfiledWidthSCoeffForEgo, kRiskfiledWidthThetaCoeffForEgo,
          first_risk_conflict_obj_idx, last_risk_conflict_obj_idx,
          first_risk_conflict_ego_point, last_risk_conflict_ego_point,
          risk_field_lat_ratio_ego_view, ego_left_risk_edge,
          ego_right_risk_edge);

      // agent_view check
      std::vector<Polygon2d> ego_polygons;
      ego_polygons.reserve(ego_path_->size());
      std::vector<Box2d> ego_boxs;
      ego_boxs.reserve(ego_path_->size());
      for (auto &path_pt : (*ego_path_)) {
        const Vec2d pt_center(path_pt.x(), path_pt.y());
        const Vec2d tangent = Vec2d::FastUnitFromAngle(path_pt.theta());
        const Vec2d center =
            pt_center + tangent * (0.5 * vehicle_geo_params_->length() -
                                   vehicle_geo_params_->back_edge_to_center());
        const Box2d pt_box(center, path_pt.theta(),
                           vehicle_geo_params_->length(),
                           vehicle_geo_params_->width());
        const Polygon2d pt_polygon(pt_box);
        ego_polygons.push_back(pt_polygon);
        ego_boxs.push_back(pt_box);
      }
      const double agent_l0 = 1.0;                           // TODO待标定
      constexpr double kRiskfiledWidthSCoeffForAgent = 0.3;  // TODO待标定
      constexpr double kRiskfiledWidthThetaCoeffForAgent = 0.3;  // TODO待标定
      std::optional<size_t> first_risk_conflict_ego_idx;
      std::optional<size_t> last_risk_conflict_ego_idx;
      PathPoint first_risk_conflict_agent_point;
      PathPoint last_risk_conflict_agent_point;
      std::optional<double> risk_field_lat_ratio_agent_view;
      std::vector<std::pair<double, double>> agent_left_risk_edge;
      std::vector<std::pair<double, double>> agent_right_risk_edge;
      RiskFieldConflictCheck(
          &obj_path, &ego_polygons, &ego_boxs, 0.5 * obj_box.width(), agent_l0,
          kRiskfiledWidthSCoeffForAgent, kRiskfiledWidthThetaCoeffForAgent,
          first_risk_conflict_ego_idx, last_risk_conflict_ego_idx,
          first_risk_conflict_agent_point, last_risk_conflict_agent_point,
          risk_field_lat_ratio_agent_view, agent_left_risk_edge,
          agent_right_risk_edge);

      //互相都侵入对方的风险场，填充st信息
      if (first_risk_conflict_obj_idx.has_value() &&
          first_risk_conflict_ego_idx.has_value()) {
        GamingConflictZoneInfo conflict_zone_info;
        conflict_zone_info.obj_traj_id = st_traj->traj_id();
        const auto &first_risk_conflict_obj_state =
            object_states[first_risk_conflict_obj_idx.value()];
        const auto &last_risk_conflict_obj_state =
            object_states[last_risk_conflict_obj_idx.value()];
        conflict_zone_info.conflict_zone_in_ego_view.is_risk_field_conflict =
            true;
        conflict_zone_info.conflict_zone_in_ego_view.risk_field_lat_ratio =
            risk_field_lat_ratio_ego_view.value();
        conflict_zone_info.conflict_zone_in_ego_view.ego_cutin_s =
            std::min(first_risk_conflict_ego_point.s(),
                     last_risk_conflict_ego_point.s());
        conflict_zone_info.conflict_zone_in_ego_view.ego_cutout_s =
            std::max(first_risk_conflict_ego_point.s(),
                     last_risk_conflict_ego_point.s());
        conflict_zone_info.conflict_zone_in_ego_view.agent_cutin_time =
            first_risk_conflict_obj_state.traj_point->t();
        conflict_zone_info.conflict_zone_in_ego_view.agent_cutin_s =
            first_risk_conflict_obj_state.traj_point->s();
        conflict_zone_info.conflict_zone_in_ego_view.agent_cutout_time =
            last_risk_conflict_obj_state.traj_point->t();
        conflict_zone_info.conflict_zone_in_ego_view.agent_cutout_s =
            last_risk_conflict_obj_state.traj_point->s();
        // agent,ego只能确定s,不能确定t,agent根据初始预测轨迹确定t
        conflict_zone_info.conflict_zone_in_agent_view.is_risk_field_conflict =
            true;
        conflict_zone_info.conflict_zone_in_agent_view.risk_field_lat_ratio =
            risk_field_lat_ratio_agent_view.value();
        conflict_zone_info.conflict_zone_in_agent_view.ego_cutin_s =
            (*ego_path_)[first_risk_conflict_ego_idx.value()].s();
        conflict_zone_info.conflict_zone_in_agent_view.ego_cutout_s =
            (*ego_path_)[last_risk_conflict_ego_idx.value()].s();

        auto GetLowerIdx = [&obj_path](double s) -> size_t {
          auto it = std::lower_bound(
              obj_path.begin(), obj_path.end(), s,
              [](const PathPoint &p, double s) { return p.s() < s; });
          return std::distance(obj_path.begin(), it);
        };
        size_t first_risk_conflict_agent_idx =
            GetLowerIdx(first_risk_conflict_agent_point.s());
        size_t last_risk_conflict_agent_idx =
            GetLowerIdx(last_risk_conflict_agent_point.s());
        if (first_risk_conflict_agent_idx > last_risk_conflict_agent_idx) {
          std::swap(first_risk_conflict_agent_idx,
                    last_risk_conflict_agent_idx);
        }
        if (first_risk_conflict_agent_idx > 0) {
          first_risk_conflict_agent_idx--;
        }
        conflict_zone_info.conflict_zone_in_agent_view.agent_cutin_time =
            object_states[first_risk_conflict_agent_idx].traj_point->t();
        conflict_zone_info.conflict_zone_in_agent_view.agent_cutin_s =
            object_states[first_risk_conflict_agent_idx].traj_point->s();
        conflict_zone_info.conflict_zone_in_agent_view.agent_cutout_time =
            object_states[last_risk_conflict_agent_idx].traj_point->t();
        conflict_zone_info.conflict_zone_in_agent_view.agent_cutout_s =
            object_states[last_risk_conflict_agent_idx].traj_point->s();
        conflict_zone_info.interaction_type = interaction_type;
        conflict_zone_infos.insert(
            {conflict_zone_info.obj_traj_id, conflict_zone_info});
      }
    }
  }
}
void SpeedObstacleProcessor::GenerateConflictZoneForOneAgent(
    GamingConflictZoneInfo &conflict_zone_info,
    const SpacetimeObjectTrajectory *obj_traj, bool is_static) {
  if (ToStBoundaryObjectType(obj_traj->planner_object().type()) !=
      StBoundaryProto::IGNORABLE) {
    const auto &object_states = obj_traj->states();

    std::optional<std::pair<double, double>> ego_s_range;
    std::optional<std::pair<int, int>> agent_index_range;
    SimplifyPathConflictCheck(ego_path_, obj_traj, is_static, ego_s_range,
                              agent_index_range);

    //存在path conflict
    if (agent_index_range.has_value()) {
      const auto &first_overlap_obj_state =
          object_states[agent_index_range->first];
      const auto &last_overlap_obj_state =
          object_states[agent_index_range->second];
      const auto interaction_type =
          is_static ? InteractionType::kStaticOccupy
                    : InteractionClassification(last_overlap_obj_state,
                                                agent_index_range->second);

      conflict_zone_info.obj_traj_id = obj_traj->traj_id();
      // ego_view,ego只能确定s,不能确定t,agent根据初始预测轨迹确定t
      conflict_zone_info.conflict_zone_in_ego_view.is_path_conflict = true;
      conflict_zone_info.conflict_zone_in_ego_view.ego_cutin_s =
          ego_s_range->first;
      conflict_zone_info.conflict_zone_in_ego_view.ego_cutout_s =
          ego_s_range->second;
      conflict_zone_info.conflict_zone_in_ego_view.agent_cutin_time =
          first_overlap_obj_state.traj_point->t();
      conflict_zone_info.conflict_zone_in_ego_view.agent_cutin_s =
          first_overlap_obj_state.traj_point->s();
      conflict_zone_info.conflict_zone_in_ego_view.agent_cutout_time =
          is_static ? 10.0 : last_overlap_obj_state.traj_point->t();
      conflict_zone_info.conflict_zone_in_ego_view.agent_cutout_s =
          last_overlap_obj_state.traj_point->s();
      // agent,ego只能确定s,不能确定t,agent根据初始预测轨迹确定t
      conflict_zone_info.conflict_zone_in_agent_view.is_path_conflict = true;
      conflict_zone_info.conflict_zone_in_agent_view.ego_cutin_s =
          ego_s_range->first;
      conflict_zone_info.conflict_zone_in_agent_view.ego_cutout_s =
          ego_s_range->second;
      conflict_zone_info.conflict_zone_in_agent_view.agent_cutin_time =
          first_overlap_obj_state.traj_point->t();
      conflict_zone_info.conflict_zone_in_agent_view.agent_cutin_s =
          first_overlap_obj_state.traj_point->s();
      conflict_zone_info.conflict_zone_in_agent_view.agent_cutout_time =
          is_static ? 10.0 : last_overlap_obj_state.traj_point->t();
      conflict_zone_info.conflict_zone_in_agent_view.agent_cutout_s =
          last_overlap_obj_state.traj_point->s();

      if (agent_index_range->first == 0) {
        // 0时刻就conflict
        conflict_zone_info.is_pre_gaming = true;
      }
      conflict_zone_info.interaction_type = interaction_type;
    }
  }
}
void SpeedObstacleProcessor::GenerateYieldConflictZoneForLaneChangeSafety(
    std::unordered_map<std::string, GamingConflictZoneInfo>
        &conflict_zone_infos) {
  conflict_zone_infos.clear();

  // step1: 构造自车points
  std::vector<Vec2d> points;
  points.reserve(ego_path_->size());
  for (const auto &point : *ego_path_) {
    points.emplace_back(point.x(), point.y());
  }

  const Vec2d ego_center(ego_path_->front().x(), ego_path_->front().y());
  const Vec2d tangent = Vec2d::FastUnitFromAngle(ego_path_->front().theta());
  const Vec2d center =
      ego_center + tangent * (0.5 * vehicle_geo_params_->length() -
                              vehicle_geo_params_->back_edge_to_center());
  const Box2d ego_box(center, ego_path_->front().theta(),
                      vehicle_geo_params_->length(),
                      vehicle_geo_params_->width());
  const Polygon2d ego_polygon(ego_box);
  ego_box_sl_ = drive_passage_->QueryFrenetBoxAtContour(ego_polygon).value();

  // step2: 遍历动态前车轨迹进行交叉检验和分类
  const auto &moving_spacetime_objects = traj_mgr_->moving_object_trajs();
  for (const SpacetimeObjectTrajectory *st_traj : moving_spacetime_objects) {
    const Polygon2d &obj_box = st_traj->contour();
    const auto obj_box_sl = drive_passage_->QueryFrenetBoxAtContour(obj_box);
    if (obj_box_sl.ok() && obj_box_sl->s_min > ego_box_sl_.s_max) {
      GamingConflictZoneInfo conflict_zone_info;
      GenerateConflictZoneForOneAgent(conflict_zone_info, st_traj, false);
      if (conflict_zone_info.conflict_zone_in_ego_view.is_path_conflict) {
        conflict_zone_infos.insert(
            {conflict_zone_info.obj_traj_id, conflict_zone_info});
      }
    }
  }

  // step3: 遍历静态障碍物进行交叉检验
  const auto &stationary_spacetime_objects =
      traj_mgr_->stationary_object_trajs();
  for (const SpacetimeObjectTrajectory *st_traj :
       stationary_spacetime_objects) {
    const Polygon2d &obj_box = st_traj->planner_object().contour();
    const auto obj_box_sl = drive_passage_->QueryFrenetBoxAtContour(obj_box);
    if (obj_box_sl.ok() && obj_box_sl->s_min > ego_box_sl_.s_max) {
      GamingConflictZoneInfo conflict_zone_info;
      GenerateConflictZoneForOneAgent(conflict_zone_info, st_traj, true);
      if (conflict_zone_info.conflict_zone_in_ego_view.is_path_conflict) {
        conflict_zone_infos.insert(
            {conflict_zone_info.obj_traj_id, conflict_zone_info});
      }
    }
  }
}

void SpeedObstacleProcessor::ConvertStBoundaryToConflictZone(
    std::unordered_map<std::string, GamingConflictZoneInfo>
        &conflict_zone_infos,
    const std::vector<StBoundaryWithDecision> &st_boundaries_with_decision,
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>
        &processed_st_objects,
    const absl::flat_hash_set<std::string> &gaming_lc_obs_set,
    const int &plan_id) {
  std::string group_name = Log2DDS::TaskPrefix(plan_id) + "gaming/input";
  conflict_zone_infos.clear();
  for (auto &st_boundarie_decision : st_boundaries_with_decision) {
    if (st_boundarie_decision.decision_type() != StBoundaryProto::IGNORE) {
      const auto st_boundary = st_boundarie_decision.raw_st_boundary();
      const auto &overlap_meta = st_boundary->overlap_meta();
      if (overlap_meta.has_value() &&
          overlap_meta->is_oncoming())  //忽略对向obj
      {
        std::string debug_info =
            "ignore oncoming: " + st_boundary->traj_id().value() + "\n";
        Log2DDS::LogDataV2(group_name, debug_info);
        continue;
      }
      if (!st_boundary->is_stationary() && st_boundary->traj_id().has_value()) {
        size_t pos = st_boundary->traj_id()->find("|");
        std::string traj_id = pos != std::string::npos
                                  ? st_boundary->traj_id()->substr(0, pos)
                                  : st_boundary->traj_id().value();
        // 优先从processed_st_objects中取轨迹，再从原始traj_mgr中取
        auto processed_traj = processed_st_objects.find(traj_id);
        const auto &valid_traj =
            processed_traj != processed_st_objects.end()
                ? SpeedGamingCommon::genThetaValidTraj(processed_traj->second)
                : SpeedGamingCommon::genThetaValidTraj(
                      *traj_mgr_->FindTrajectoryById(
                          std::string_view(traj_id)));

        // new_traj
        // auto gaming_processed_traj =
        // SpeedGamingCommon::genThetaValidTraj(*obj_traj);
        const auto &object_states = valid_traj.states();
        const int first_overlap_obj_index =
            st_boundary->overlap_infos().front().obj_idx;
        const int last_overlap_obj_index =
            st_boundary->overlap_infos().back().obj_idx;
        const auto &first_overlap_obj_state =
            object_states[first_overlap_obj_index];
        const auto &last_overlap_obj_state =
            object_states[last_overlap_obj_index];
        const auto interaction_type = AnalyzeInteractionType(
            last_overlap_obj_state,
            st_boundary->overlap_infos().back().av_end_idx);
        // std::string debug_info = "obs_id_is: " +
        // st_boundary->traj_id().value() + "\n";

        const auto first_state_frenet =
            ego_path_->XYToSL(first_overlap_obj_state.contour,
                              first_overlap_obj_state.box.center());
        const auto ego_pose = ego_path_->front();
        std::pair<double, double> obj_proj =
            std::make_pair(std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity());
        const bool kFilterNearCrossObj = true;
        if (ego_pose.has_x() && ego_pose.has_y() && ego_pose.has_theta()) {
          obj_proj = GetProjectionDistance(
              ego_pose, first_overlap_obj_state.contour, plan_id);
        }

        GamingConflictZoneInfo conflict_zone_info;

        conflict_zone_info.obj_traj_id = traj_id;
        // ego_view,ego只能确定s,不能确定t,agent根据初始预测轨迹确定t,TODO:ego_view和agent_view是否有信息重复？
        conflict_zone_info.conflict_zone_in_ego_view.is_path_conflict = true;
        conflict_zone_info.conflict_zone_in_ego_view.ego_cutin_s =
            st_boundary->min_s();
        conflict_zone_info.conflict_zone_in_ego_view.ego_cutout_s =
            st_boundary->max_s();
        conflict_zone_info.conflict_zone_in_ego_view.agent_cutin_time =
            st_boundary->min_t();
        conflict_zone_info.conflict_zone_in_ego_view.agent_cutin_s =
            first_overlap_obj_state.traj_point->s();
        conflict_zone_info.conflict_zone_in_ego_view.agent_cutout_time =
            st_boundary->max_t();
        conflict_zone_info.conflict_zone_in_ego_view.agent_cutout_s =
            last_overlap_obj_state.traj_point->s();
        // agent,ego只能确定s,不能确定t,agent根据初始预测轨迹确定t
        conflict_zone_info.conflict_zone_in_agent_view.is_path_conflict = true;
        conflict_zone_info.conflict_zone_in_agent_view.ego_cutin_s =
            st_boundary->min_s();
        conflict_zone_info.conflict_zone_in_agent_view.ego_cutout_s =
            st_boundary->max_s();
        conflict_zone_info.conflict_zone_in_agent_view.agent_cutin_time =
            st_boundary->min_t();
        conflict_zone_info.conflict_zone_in_agent_view.agent_cutin_s =
            first_overlap_obj_state.traj_point->s();
        conflict_zone_info.conflict_zone_in_agent_view.agent_cutout_time =
            st_boundary->max_t();
        conflict_zone_info.conflict_zone_in_agent_view.agent_cutout_s =
            last_overlap_obj_state.traj_point->s();

        if (first_overlap_obj_index == 0) {
          // 0时刻就conflict
          conflict_zone_info.is_pre_gaming = true;
        }
        auto isSameObs = [](const std::string &s1, const std::string &s2) {
          auto getNumPrefix = [](const std::string &s) {
            auto first_non_digit =
                std::find_if_not(s.begin(), s.end(), [](char c) {
                  return std::isdigit(static_cast<unsigned char>(c));
                });
            return std::string(s.begin(), first_non_digit);
          };
          return getNumPrefix(s1) == getNumPrefix(s2);
        };
        constexpr bool kPrecossLCObs = false;
        if (kPrecossLCObs) {
          for (const std::string &str : gaming_lc_obs_set) {
            if (isSameObs(traj_id, str)) {
              conflict_zone_info.is_lc_pass = true;
              std::string debug_info = "gaming_lane_change_obs_processer_id: " +
                                       st_boundary->traj_id().value() + "\n";
              Log2DDS::LogDataV2(group_name, debug_info);
              break;
            }
          }
        }
        conflict_zone_info.interaction_type = interaction_type;
        if (st_boundarie_decision.decision_type() == StBoundaryProto::FOLLOW ||
            st_boundarie_decision.modifier().modifier_type() ==
                StBoundaryModifierProto::CIPV) {
          conflict_zone_info.is_follow = true;
        }
        //对于切入点在当前自车车头后，交互类型为cross的障碍物不处理
        if (kFilterNearCrossObj &&
            interaction_type == InteractionType::kCross &&
            (obj_proj.first < vehicle_geo_params_->front_edge_to_center() ||
             first_state_frenet.s_max <
                 vehicle_geo_params_->front_edge_to_center()) &&
            conflict_zone_info.is_follow == false) {
          conflict_zone_info.is_ignore = true;
          Log2DDS::LogDataV3(
              group_name,
              absl::StrCat(st_boundary->traj_id().value_or("unknown"),
                           " max proj: ", obj_proj.first,
                           " min proj: ", obj_proj.second));
        } else if (kFilterNearCrossObj &&
                   interaction_type == InteractionType::kCross &&
                   (obj_proj.second <
                        vehicle_geo_params_->front_edge_to_center() ||
                    first_state_frenet.s_min <
                        vehicle_geo_params_->front_edge_to_center()) &&
                   conflict_zone_info.is_pre_gaming == false &&
                   conflict_zone_info.is_follow == false) {
          conflict_zone_info.is_ignore = true;
          Log2DDS::LogDataV3(
              group_name,
              absl::StrCat(st_boundary->traj_id().value_or("unknown"),
                           " max proj: ", obj_proj.first,
                           " min proj: ", obj_proj.second));
        }
        conflict_zone_infos.insert(
            {conflict_zone_info.obj_traj_id, conflict_zone_info});
      } else {
        GamingConflictZoneInfo conflict_zone_info;
        conflict_zone_info.obj_traj_id = st_boundary->id();
        conflict_zone_info.conflict_zone_in_ego_view.is_path_conflict = true;
        conflict_zone_info.conflict_zone_in_ego_view.ego_cutin_s =
            st_boundary->min_s();
        conflict_zone_info.conflict_zone_in_ego_view.ego_cutout_s =
            st_boundary->min_s();
        conflict_zone_info.conflict_zone_in_ego_view.agent_cutin_time = 0.0;
        conflict_zone_info.conflict_zone_in_ego_view.agent_cutin_s = 0.0;
        conflict_zone_info.conflict_zone_in_ego_view.agent_cutout_time = 10.0;
        conflict_zone_info.conflict_zone_in_ego_view.agent_cutout_s = 0.0;
        conflict_zone_info.conflict_zone_in_agent_view.is_path_conflict = true;
        conflict_zone_info.conflict_zone_in_agent_view.ego_cutin_s =
            st_boundary->min_s();
        conflict_zone_info.conflict_zone_in_agent_view.ego_cutout_s =
            st_boundary->min_s();
        conflict_zone_info.conflict_zone_in_agent_view.agent_cutin_time = 0.0;
        conflict_zone_info.conflict_zone_in_agent_view.agent_cutin_s = 0.0;
        conflict_zone_info.conflict_zone_in_agent_view.agent_cutout_time = 10.0;
        conflict_zone_info.conflict_zone_in_agent_view.agent_cutout_s = 0.0;
        conflict_zone_info.is_pre_gaming = true;
        conflict_zone_info.interaction_type = InteractionType::kStaticOccupy;
        if (st_boundarie_decision.decision_type() == StBoundaryProto::FOLLOW) {
          conflict_zone_info.is_follow = true;
        }
        conflict_zone_infos.insert(
            {conflict_zone_info.obj_traj_id, conflict_zone_info});
      }
    }
  }
}
void SpeedObstacleProcessor::PathConflictCheck(
    const SpacetimeObjectTrajectory *st_traj, const double &search_radius,
    const PathApprox &ego_path_approx, const double &path_step_length,
    std::optional<std::pair<double, double>> &first_overlap_ego_s_range,
    int &first_overlap_obj_index,
    std::optional<std::pair<double, double>> &last_overlap_ego_s_range,
    int &last_overlap_obj_index) {
  const auto &object_states = st_traj->states();
  size_t obj_state_len = object_states.size();
  //从前往后找第1个碰撞点
  for (size_t i = 0; i < obj_state_len; ++i) {
    const auto agent_overlaps = ComputeAgentOverlapsWithBuffer(
        ego_path_approx, path_step_length, /*first_index=*/0,
        /*last_index=*/ego_path_->size() - 1, object_states[i].contour,
        /*max_lat_dist=*/kMaxLatDist, st_traj->required_lateral_gap(),
        st_traj->required_lateral_gap(), search_radius);
    first_overlap_ego_s_range = ConvertToOverlapRange(agent_overlaps);
    if (first_overlap_ego_s_range.has_value()) {
      first_overlap_obj_index = i;
      break;
    }
  }
  if (first_overlap_ego_s_range.has_value()) {  //如果从头找不到，也没必要从尾找
    //从后往前找最后1个碰撞点，TODO:这个地方找的是最后1个碰撞点，计算cutout点可能还需要考虑将这个点外扩
    for (size_t i = 0; i < obj_state_len; ++i) {
      const auto agent_overlaps = ComputeAgentOverlapsWithBuffer(
          ego_path_approx, path_step_length, /*first_index=*/0,
          /*last_index=*/ego_path_->size() - 1,
          object_states[obj_state_len - 1 - i].contour,
          /*max_lat_dist=*/kMaxLatDist, st_traj->required_lateral_gap(),
          st_traj->required_lateral_gap(), search_radius);
      last_overlap_ego_s_range = ConvertToOverlapRange(agent_overlaps);
      if (last_overlap_ego_s_range.has_value()) {
        last_overlap_obj_index = obj_state_len - 1 - i;
        break;
      }
    }
  }
}
void SpeedObstacleProcessor::SimplifyPathConflictCheck(
    const DiscretizedPath *ego_path,
    const SpacetimeObjectTrajectory *agent_traj, bool is_static,
    std::optional<std::pair<double, double>> &ego_s_range,
    std::optional<std::pair<int, int>> &agent_index_range) {
  double l_threshold =
      0.5 * vehicle_geo_params_->width() + agent_traj->required_lateral_gap();
  if (is_static) {
    auto contour_frenet =
        ego_path->XYToSL(agent_traj->planner_object().contour(),
                         agent_traj->planner_object().bounding_box().center());
    if ((fabs(contour_frenet.l_min) < l_threshold ||
         fabs(contour_frenet.l_max) < l_threshold ||
         contour_frenet.l_min * contour_frenet.l_max < 0) &&
        contour_frenet.s_max > 0) {
      ego_s_range = std::make_pair(std::max(0.0, contour_frenet.s_min),
                                   std::max(0.0, contour_frenet.s_max));
      agent_index_range = std::make_pair(0, 0);
    }
  } else {
    const auto &object_states = agent_traj->states();
    std::vector<Box2d> obj_boxs;
    obj_boxs.reserve(object_states.size());
    for (auto &state : object_states) {
      obj_boxs.push_back(state.box);
    }
    // step2: get agent first and last indexs
    // Traverse from front to back until get two index
    size_t obj_state_len = object_states.size();
    //从前往后找碰撞点
    size_t agent_first_index = -1;
    size_t agent_last_index = -1;
    std::optional<double> ego_min_s;
    std::optional<double> ego_max_s;
    for (size_t i = 0; i < obj_state_len; ++i) {
      auto contour_frenet = ego_path->XYToSL(object_states[i].contour,
                                             object_states[i].box.center());
      if ((fabs(contour_frenet.l_min) < l_threshold ||
           fabs(contour_frenet.l_max) < l_threshold ||
           contour_frenet.l_min * contour_frenet.l_max < 0) &&
          contour_frenet.s_max > 0) {
        ego_min_s = std::max(0.0, contour_frenet.s_min);
        ego_max_s = std::max(0.0, contour_frenet.s_max);
        agent_first_index = i;
        break;
      }
    }
    if (agent_first_index != -1) {
      //从后往前找碰撞点
      for (size_t i = obj_state_len - 1; i >= 0; --i) {
        auto contour_frenet = ego_path->XYToSL(object_states[i].contour,
                                               object_states[i].box.center());
        if (fabs(contour_frenet.l_min) < l_threshold ||
            fabs(contour_frenet.l_max) < l_threshold ||
            contour_frenet.l_min * contour_frenet.l_max < 0) {
          ego_max_s = std::max(contour_frenet.s_max, ego_max_s.value());
          agent_last_index = i;
          break;
        }
      }
      ego_s_range = std::make_pair(ego_min_s.value(), ego_max_s.value());
      agent_index_range = std::make_pair(agent_first_index, agent_last_index);
    }
  }
}
void SpeedObstacleProcessor::RiskFieldConflictCheck(
    const DiscretizedPath *ego_path,
    const std::vector<Polygon2d> *agent_polygons,
    const std::vector<Box2d> *agent_boxs, const double &ego_half_width,
    const double &l0, const double &s_coeff, const double &theta_coeff,
    std::optional<size_t> &first_conflict_agent_idx,
    std::optional<size_t> &last_conflict_agent_idx,
    PathPoint &first_conflict_ego_point, PathPoint &last_conflict_ego_point,
    std::optional<double> &risk_field_lat_ratio,
    std::vector<std::pair<double, double>> &left_risk_edge,
    std::vector<std::pair<double, double>> &right_risk_edge) {
  left_risk_edge.clear();
  right_risk_edge.clear();
  const double ego_s_init = ego_path->front().s();
  const double ego_theta_init = ego_path->front().theta();
  size_t agent_state_len = agent_polygons->size();
  for (size_t i = 0; i < agent_state_len; ++i) {
    const auto state_frenet =
        ego_path->XYToSL((*agent_polygons)[i], (*agent_boxs)[i].center());
    const auto ref_path_point = ego_path->Evaluate(state_frenet.center_s());
    const double l_bound =
        l0 + s_coeff * (state_frenet.center_s() - ego_s_init) +
        theta_coeff *
            abs(NormalizeAngle(ref_path_point.theta() - ego_theta_init));
    left_risk_edge.push_back(
        {ref_path_point.x() +
             l_bound * std::cos(ref_path_point.theta() + 0.5 * M_PI),
         ref_path_point.y() +
             l_bound * std::sin(ref_path_point.theta() + 0.5 * M_PI)});
    right_risk_edge.push_back(
        {ref_path_point.x() +
             l_bound * std::cos(ref_path_point.theta() - 0.5 * M_PI),
         ref_path_point.y() +
             l_bound * std::sin(ref_path_point.theta() - 0.5 * M_PI)});

    bool is_risk_conflict = false;
    if (state_frenet.center_l() > 0 &&
        state_frenet.l_min < l_bound + ego_half_width)  //他车处于左侧，看min_l
    {
      if (risk_field_lat_ratio.has_value()) {
        risk_field_lat_ratio = std::min(
            risk_field_lat_ratio.value(),
            std::max(0.0, state_frenet.l_min - ego_half_width) / l_bound);
      } else {
        risk_field_lat_ratio =
            std::max(0.0, state_frenet.l_min - ego_half_width) / l_bound;
      }
      is_risk_conflict = true;
    } else if (state_frenet.center_l() < 0 &&
               state_frenet.l_max > -l_bound - ego_half_width) {
      //他车处于右侧，看max_l
      if (risk_field_lat_ratio.has_value()) {
        risk_field_lat_ratio = std::min(
            risk_field_lat_ratio.value(),
            std::max(0.0, -ego_half_width - state_frenet.l_max) / l_bound);
      } else {
        risk_field_lat_ratio =
            std::max(0.0, -ego_half_width - state_frenet.l_max) / l_bound;
      }
      is_risk_conflict = true;
    }
    if (is_risk_conflict) {
      if (!first_conflict_agent_idx.has_value()) {
        first_conflict_agent_idx = i;
        first_conflict_ego_point = ref_path_point;
      }
      last_conflict_agent_idx = i;
      last_conflict_ego_point = ref_path_point;
    }
  }
}
InteractionType SpeedObstacleProcessor::AnalyzeInteractionType(
    const prediction::PredictionObjectState &last_overlap_obj_state,
    int last_overlap_ego_index) {
  const auto last_overlap_ego_point = ego_path_->at(last_overlap_ego_index);
  const double rel_yaw =
      NormalizeAngle(last_overlap_obj_state.traj_point->theta() -
                     last_overlap_ego_point.theta());
  if (abs(rel_yaw) > M_PI / 4)  // TODO:优化判断条件
  {
    return InteractionType::kCross;
  }
  const double ego_delta_yaw =
      NormalizeAngle(ego_path_->back().theta() - ego_path_->front().theta());
  if (abs(ego_delta_yaw) < M_PI / 3)  // TODO:优化判断条件
  {
    return InteractionType::kStraightMerge;
  } else {
    return InteractionType::kTurnMerge;
  }
}
InteractionType SpeedObstacleProcessor::InteractionClassification(
    const prediction::PredictionObjectState &last_overlap_obj_state,
    double last_overlap_ego_s) {
  const auto last_overlap_ego_point = ego_path_->Evaluate(last_overlap_ego_s);
  const double rel_yaw =
      NormalizeAngle(last_overlap_obj_state.traj_point->theta() -
                     last_overlap_ego_point.theta());
  if (abs(rel_yaw) > M_PI / 4)  // TODO:优化判断条件
  {
    return InteractionType::kCross;
  }
  const double ego_delta_yaw =
      NormalizeAngle(ego_path_->back().theta() - ego_path_->front().theta());
  if (abs(ego_delta_yaw) < M_PI / 3)  // TODO:优化判断条件
  {
    return InteractionType::kStraightMerge;
  } else {
    return InteractionType::kTurnMerge;
  }
}
InteractionType SpeedObstacleProcessor::InteractionClassification(
    absl::Span<const prediction::PredictionObjectState> states) {
  if (states.size() == 0) {
    return InteractionType::kCross;
  }
  const int sample_num = 5;
  const size_t total_size = states.size();
  std::vector<prediction::PredictionObjectState> sample_states;
  if (total_size <= sample_num) {
    sample_states.assign(states.begin(), states.end());
  } else {
    std::vector<int> indices;
    indices.reserve(sample_num);
    const size_t last_idx = total_size - 1;                     // maximun index
    indices.push_back(0);                                       // 0%
    int idx_25 = static_cast<int>((25 * last_idx + 50) / 100);  // 25 %
    indices.push_back(idx_25);
    int idx_50 = static_cast<int>((last_idx + 1) / 2);  // 50%
    indices.push_back(idx_50);
    int idx_75 = static_cast<int>((75 * last_idx + 50) / 100);  // 75%
    indices.push_back(idx_75);
    indices.push_back(static_cast<int>(last_idx));  // 100%
    for (int i : indices) {
      sample_states.emplace_back(states.at(i));
    }
  }
  return InteractionClassificationByDrivePassage(
      absl::MakeConstSpan(sample_states));
}

InteractionType SpeedObstacleProcessor::InteractionClassificationByDrivePassage(
    absl::Span<const prediction::PredictionObjectState> states) {
  // std::string group_name = Log2DDS::TaskPrefix(plan_id) + "gaming/input";
  int index = 0;
  for (auto i : states) {
    double s =
        drive_passage_->frenet_frame()
            ->XYToSL(Vec2d(i.traj_point->pos().x(), i.traj_point->pos().y()))
            .s;
    Vec2d nearset_point_vec =
        drive_passage_->frenet_frame()->InterpolateTangentByS(s);
    double theta = fabs(NormalizeAngle(
        fast_math::Atan2(nearset_point_vec.y(), nearset_point_vec.x()) -
        i.traj_point->theta()));
    if (theta < M_PI / 4) {
      const double ego_delta_yaw = NormalizeAngle(ego_path_->back().theta() -
                                                  ego_path_->front().theta());
      if (abs(ego_delta_yaw) < M_PI / 3)  // TODO:优化判断条件
      {
        return InteractionType::kStraightMerge;
      } else {
        return InteractionType::kTurnMerge;
      }
    }
  }
  return InteractionType::kCross;
}
std::optional<std::pair<double, double>>
SpeedObstacleProcessor::ConvertToOverlapRange(
    absl::Span<const AgentOverlap> agent_overlaps) {
  if (agent_overlaps.empty()) return std::nullopt;
  std::optional<std::pair<double, double>> overlap_range;
  for (const auto &agent_overlap : agent_overlaps) {
    if (agent_overlap.lat_dist != 0.0) continue;
    if (!overlap_range.has_value()) {
      overlap_range =
          std::make_pair(agent_overlap.first_ra_s, agent_overlap.last_ra_s);
    } else {
      overlap_range->first =
          std::min(overlap_range->first, agent_overlap.first_ra_s);
      overlap_range->second =
          std::max(overlap_range->second, agent_overlap.last_ra_s);
    }
  }
  if (overlap_range.has_value() &&
      overlap_range->first >= overlap_range->second) {
    return std::nullopt;
  }
  return overlap_range;
}

void SpeedObstacleProcessor::RefinePredTraj(
    const SpacetimeTrajectoryManager &origin_traj_mgr,
    SpacetimeTrajectoryManager *const modified_traj_mgr) {
  *modified_traj_mgr = origin_traj_mgr;
  auto mutable_trajs_ptr = modified_traj_mgr->mutable_trajectories();
  for (size_t i = 0; i < mutable_trajs_ptr->size(); i++) {
    //修改预测原始theta值
    auto &st_traj = mutable_trajs_ptr->at(i);
    prediction::PredictedTrajectory modified_traj = st_traj.trajectory();
    auto points_ptr = modified_traj.mutable_points();
    if (points_ptr->size() < 2) {
      continue;
    }
    for (size_t i = 0; i + 1 < points_ptr->size(); i++) {
      const double dx =
          points_ptr->at(i + 1).pos().x() - points_ptr->at(i).pos().x();
      const double dy =
          points_ptr->at(i + 1).pos().y() - points_ptr->at(i).pos().y();
      if (dx * dx + dy * dy < 0.25)  //低速不修
      {
        continue;
      }
      double theta = NormalizeAngle(fast_math::Atan2(dy, dx));
      points_ptr->at(i).set_theta(theta);
    }
    points_ptr->back().set_theta(
        points_ptr->at(points_ptr->size() - 2).theta());
    st_traj = st_traj.CreateTrajectoryMutatedInstance(modified_traj);
  }
}
void SpeedObstacleProcessor::ConvertStBoundaryToSimResult(
    const StBoundary &st_boundary,
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>
        &processed_st_objects,
    const SpacetimeTrajectoryManager &traj_mgr,
    const SpeedGamingParams &speed_gaming_params,
    GamingSimResult &yield_sim_result) {
  std::string obj_traj_id = st_boundary.traj_id().value();
  //优先从processed_st_objects中取轨迹，再从原始traj_mgr中取
  auto processed_traj =
      processed_st_objects.find(st_boundary.traj_id().value());
  auto st_traj = traj_mgr.FindTrajectoryById(
      std::string_view(st_boundary.traj_id().value()));
  const auto valid_traj =
      processed_traj != processed_st_objects.end()
          ? SpeedGamingCommon::genThetaValidTraj(processed_traj->second)
          : SpeedGamingCommon::genThetaValidTraj(*st_traj);
  const auto &object_states = valid_traj.states();
  const int first_overlap_obj_index =
      st_boundary.overlap_infos().front().obj_idx;
  const int last_overlap_obj_index = st_boundary.overlap_infos().back().obj_idx;
  const auto &first_overlap_obj_state = object_states[first_overlap_obj_index];
  const auto &last_overlap_obj_state = object_states[last_overlap_obj_index];
  yield_sim_result.planner_object = &(st_traj->planner_object());
  yield_sim_result.interaction_type = AnalyzeInteractionType(
      last_overlap_obj_state, st_boundary.overlap_infos().back().av_end_idx);

  // ego_view,ego只能确定s,不能确定t,agent根据初始预测轨迹确定t,TODO:ego_view和agent_view是否有信息重复？
  yield_sim_result.sim_conflict_zone_in_ego_view.is_path_conflict = true;
  yield_sim_result.sim_conflict_zone_in_ego_view.ego_cutin_s =
      st_boundary.min_s();
  yield_sim_result.sim_conflict_zone_in_ego_view.ego_cutout_s =
      st_boundary.max_s();
  yield_sim_result.sim_conflict_zone_in_ego_view.agent_cutin_time =
      st_boundary.min_t();
  yield_sim_result.sim_conflict_zone_in_ego_view.agent_cutin_s =
      first_overlap_obj_state.traj_point->s();
  yield_sim_result.sim_conflict_zone_in_ego_view.agent_cutout_time =
      st_boundary.max_t();
  yield_sim_result.sim_conflict_zone_in_ego_view.agent_cutout_s =
      last_overlap_obj_state.traj_point->s();
  // agent,ego只能确定s,不能确定t,agent根据初始预测轨迹确定t
  yield_sim_result.sim_conflict_zone_in_agent_view.is_path_conflict = true;
  yield_sim_result.sim_conflict_zone_in_agent_view.ego_cutin_s =
      st_boundary.min_s();
  yield_sim_result.sim_conflict_zone_in_agent_view.ego_cutout_s =
      st_boundary.max_s();
  yield_sim_result.sim_conflict_zone_in_agent_view.agent_cutin_time =
      st_boundary.min_t();
  yield_sim_result.sim_conflict_zone_in_agent_view.agent_cutin_s =
      first_overlap_obj_state.traj_point->s();
  yield_sim_result.sim_conflict_zone_in_agent_view.agent_cutout_time =
      st_boundary.max_t();
  yield_sim_result.sim_conflict_zone_in_agent_view.agent_cutout_s =
      last_overlap_obj_state.traj_point->s();
  yield_sim_result.ego_lon_type = LonGamingDecisionType::kYield;
  auto obj_traj =
      processed_traj != processed_st_objects.end()
          ? &(processed_traj->second)
          : traj_mgr.FindTrajectoryById(std::string_view(obj_traj_id));
  auto processed_oobj_traj = SpeedGamingCommon::genThetaValidTraj(*obj_traj);
  yield_sim_result.obj_speed_data =
      SpeedGamingCommon::ConvertPredTrajToSpeedData(
          &processed_oobj_traj, speed_gaming_params.time_step,
          speed_gaming_params.planning_horizon);
  const auto obj_path = SpeedGamingCommon::GeneratePathBasedTraj(
      &processed_oobj_traj, kObstaclePathMaxLength);
  yield_sim_result.obj_traj = SpeedGamingCommon::GenerateTrajByPathAndSpeedData(
      obj_path, yield_sim_result.obj_speed_data);
}
void SpeedObstacleProcessor::PostPrecess(
    const SpacetimeTrajectoryManager *traj_mgr,
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>
        *processed_st_objects,
    const std::unordered_set<std::string> &ignore_objs,
    const std::unordered_map<std::string, GamingSimResult> &follow_sim_results,
    const std::unordered_map<std::string, GamingSimResult> &yield_sim_results,
    const std::unordered_map<std::string, GamingSimResult>
        &overtake_sim_results,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>
        *const gaming_processed_st_objects,
    std::unordered_map<std::string, StBoundaryProto::DecisionType>
        *const gaming_decision,
    std::unordered_map<std::string, RiskObjInfo>
        *const gaming_risk_field_zoneinfos) {
  for (auto &obj_id : ignore_objs) {
    StBoundaryProto::DecisionType decision_type = StBoundaryProto::IGNORE;
    gaming_decision->emplace(obj_id, decision_type);
  }
  for (auto &sim_result : follow_sim_results) {
    StBoundaryProto::DecisionType decision_type = StBoundaryProto::FOLLOW;
    gaming_decision->emplace(sim_result.first, decision_type);
  }
  for (auto &sim_result : overtake_sim_results) {
    StBoundaryProto::DecisionType decision_type = StBoundaryProto::OVERTAKE;
    gaming_decision->emplace(sim_result.first, decision_type);
  }
  for (auto &sim_result : yield_sim_results) {
    StBoundaryProto::DecisionType decision_type = StBoundaryProto::YIELD;
    gaming_decision->emplace(sim_result.first, decision_type);
    //传出修正预测轨迹或者风险场切入切出点
    if (sim_result.second.sim_conflict_zone_in_ego_view.is_path_conflict) {
      //优先从processed_st_objects中取轨迹，再从原始traj_mgr中取
      auto processed_traj = processed_st_objects->find(sim_result.first);
      auto st_traj = processed_traj != processed_st_objects->end()
                         ? processed_traj->second
                         : *traj_mgr->FindTrajectoryById(
                               std::string_view(sim_result.first));
      auto &modified_traj_points = sim_result.second.obj_traj;
      prediction::PredictedTrajectory modified_traj = st_traj.trajectory();
      auto points_ptr = modified_traj.mutable_points();
      points_ptr->clear();
      for (auto &pt : modified_traj_points) {
        if (sim_result.second.interaction_type == InteractionType::kCross &&
            pt.t() > sim_result.second.sim_conflict_zone_in_ego_view
                         .agent_cutout_time) {
          break;
        }
        prediction::PredictedTrajectoryPoint tpt(pt);
        points_ptr->emplace_back(tpt);
      }
      st_traj = st_traj.CreateTrajectoryMutatedInstance(modified_traj);
      gaming_processed_st_objects->emplace(sim_result.first, st_traj);
    } else if (sim_result.second.sim_conflict_zone_in_ego_view
                   .is_risk_field_conflict) {
      RiskObjInfo risk_obj_info;
      auto st_traj = traj_mgr->FindTrajectoryById(sim_result.first);
      risk_obj_info.risk_field_zoneinfo =
          std::move(sim_result.second.sim_conflict_zone_in_ego_view);
      risk_obj_info.planner_obj = &(st_traj->planner_object());
      gaming_risk_field_zoneinfos->emplace(sim_result.first, risk_obj_info);
    }
  }
}
SpacetimeObjectTrajectory
SpeedObstacleProcessor::GenerateSpacetimeObjectTrajectory(
    const std::unordered_map<std::string, SpacetimeObjectTrajectory>
        *processed_st_objects,
    const SpacetimeTrajectoryManager *traj_mgr,
    const GamingSimResult &sim_result, const std::string &id) {
  //传出修正预测轨迹
  //优先从processed_st_objects中取轨迹，再从原始traj_mgr中取
  auto processed_traj = processed_st_objects->find(id);
  auto st_traj = processed_traj != processed_st_objects->end()
                     ? processed_traj->second
                     : *traj_mgr->FindTrajectoryById(std::string_view(id));
  auto &modified_traj_points = sim_result.obj_traj;
  prediction::PredictedTrajectory modified_traj = st_traj.trajectory();
  auto points_ptr = modified_traj.mutable_points();
  points_ptr->clear();
  for (auto &pt : modified_traj_points) {
    if (sim_result.interaction_type == InteractionType::kCross &&
        pt.t() > sim_result.sim_conflict_zone_in_ego_view.agent_cutout_time) {
      break;
    }
    prediction::PredictedTrajectoryPoint tpt(pt);
    points_ptr->emplace_back(tpt);
  }
  st_traj = st_traj.CreateTrajectoryMutatedInstance(modified_traj);
  return st_traj;
}
std::pair<double, double> SpeedObstacleProcessor::GetProjectionDistance(
    const PathPoint &ego_pose, const Polygon2d &contour, const int &plan_id) {
  double max_proj = -std::numeric_limits<double>::infinity();  // 初始化为负无穷
  double min_proj = std::numeric_limits<double>::infinity();  // 初始化为正无穷
  const double cos_theta = std::cos(ego_pose.theta());
  const double sin_theta = std::sin(ego_pose.theta());
  // std::string group_name = Log2DDS::TaskPrefix(plan_id) + "gaming/input";

  const auto &points = contour.points();

  // Log2DDS::LogDataV2(group_name, "points size: " +
  // absl::StrCat(points.size()));
  for (const auto &pt : points) {
    const double dx = pt.x() - ego_pose.x();
    const double dy = pt.y() - ego_pose.y();
    const double x_ego = dx * cos_theta + dy * sin_theta;
    max_proj = std::max(x_ego, max_proj);
    min_proj = std::min(x_ego, min_proj);

    // Log2DDS::LogDataV2(group_name, "x_ego: " + absl::StrCat(x_ego));
  }
  return std::make_pair(max_proj, min_proj);
}
}  // namespace st::planning