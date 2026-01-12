#include "risk_field.h"

namespace st::planning {
constexpr double kIdmAlpha = 0.2;
constexpr double kIdmK0ForCross = 0.5;
constexpr double kIdmSafeDist = 3.0;
constexpr double kEpsilon = 1e-03;
constexpr double kLatDistDVCoeff = 0.05;
constexpr double kLongitudeDistDVCoeff = 0.2;
constexpr double kSyncMinLatDistance = 1.0;
constexpr double kOppoMinLatDistance = 0.5;
constexpr double kInitRisk = -1.0;  // Init risk value as no risk
constexpr double kLatRiskLengthBufferCoeff = 0.0;
constexpr double kTurnRadiusCompensation = 2.0;
constexpr double kUpperBoundBuffer = -17.0;

std::optional<Vec2d> RiskField::GetLineInsertion(const Vec2d& A, const Vec2d& B,
                                                 const Vec2d& C,
                                                 const Vec2d& D) {
  // 计算斜率
  double m1 = (B.y() - A.y()) / (B.x() - A.x());
  double m2 = (D.y() - C.y()) / (D.x() - C.x());
  // 如果斜率相同，则直线平行或共线
  if (m1 == m2) {
    return std::nullopt;  // 平行或共线，无交点
  }
  // 计算交点的 x 坐标
  double x = ((m1 * A.x() - m2 * C.x()) + (C.y() - A.y())) / (m1 - m2);
  // 计算交点的 y 坐标
  double y = m1 * (x - A.x()) + A.y();
  return Vec2d(x, y);
}

AgentPoseInfo RiskField::CalcAgentPoseInfo(const Box2d& av_box,
                                           const Box2d& agent_box) {
  AgentPoseInfo agent_pose_info;
  const double av_half_length = av_box.half_length();
  const double av_half_width = av_box.half_width();
  const double agent_half_length = agent_box.half_length();
  const double agent_half_width = agent_box.half_width();
  const double av_cos_heading = av_box.cos_heading();
  const double av_sin_heading = av_box.sin_heading();
  const double agent_cos_heading = agent_box.cos_heading();
  const double agent_sin_heading = agent_box.sin_heading();

  const Vec2d center_diff = agent_box.center() - av_box.center();
  const double heading_diff =
      NormalizeAngle(agent_box.heading() - av_box.heading());

  // The cos, sin of heading diff
  const double sin_heading_diff = std::sin(heading_diff);
  const double cos_heading_diff = std::cos(heading_diff);

  // Directional and left-hand side vector of AV
  const Vec2d av_x_axis_unit_vector(av_cos_heading, av_sin_heading);
  const Vec2d av_y_axis_unit_vector(-av_sin_heading, av_cos_heading);

  // The center of agent-box in the view of av
  const Vec2d agent_center_in_av_vcs(
      center_diff.InnerProd(av_x_axis_unit_vector),
      center_diff.InnerProd(av_y_axis_unit_vector));

  agent_pose_info.is_parallel = (std::abs(sin_heading_diff) < 0.1);
  agent_pose_info.is_vertical = (std::abs(cos_heading_diff) < 0.1);
  agent_pose_info.heading_diff = heading_diff;
  agent_pose_info.cos_heading_diff = cos_heading_diff;
  agent_pose_info.sin_heading_diff = sin_heading_diff;
  agent_pose_info.agent_length = agent_box.length();
  agent_pose_info.agent_width = agent_box.width();

  // The unit vector of agent vector along the heading direction in the view of
  // av
  const Vec2d agent_x_axis_unit_vec_av_vcs(cos_heading_diff, sin_heading_diff);

  // The unit vector of agent vector to agent's left-hand side in the view of av
  const Vec2d agent_y_axis_unit_vec_av_vcs(-sin_heading_diff, cos_heading_diff);

  // Four vertices of agent-box in vcs
  const auto agent_center_to_head_vector =
      agent_half_length * agent_x_axis_unit_vec_av_vcs;
  const auto agent_center_to_left_vector =
      agent_half_width * agent_y_axis_unit_vec_av_vcs;
  const Vec2d trans_left_head_point = agent_center_in_av_vcs +
                                      agent_center_to_head_vector +
                                      agent_center_to_left_vector;
  const Vec2d trans_left_tail_point = agent_center_in_av_vcs -
                                      agent_center_to_head_vector +
                                      agent_center_to_left_vector;
  const Vec2d trans_right_head_point = agent_center_in_av_vcs +
                                       agent_center_to_head_vector -
                                       agent_center_to_left_vector;
  const Vec2d trans_right_tail_point = agent_center_in_av_vcs -
                                       agent_center_to_head_vector -
                                       agent_center_to_left_vector;

  // four points vector
  agent_pose_info.agent_box_points = {
      trans_left_head_point, trans_left_tail_point, trans_right_tail_point,
      trans_right_head_point};

  // Calculate the min, max coordinates:
  double x_min = 1E10;
  double x_max = -1E10;
  double y_min = 1E10;
  double y_max = -1E10;
  for (const auto& p : agent_pose_info.agent_box_points) {
    const double& x = p.x();
    const double& y = p.y();
    x_min = std::min(x_min, p.x());
    x_max = std::max(x_max, p.x());
    y_min = std::min(y_min, p.y());
    y_max = std::max(y_max, p.y());
  }

  agent_pose_info.x_min = x_min;
  agent_pose_info.x_max = x_max;
  agent_pose_info.y_min = y_min;
  agent_pose_info.y_max = y_max;

  // Determine the conflict edges
  if ((agent_pose_info.is_parallel) && (cos_heading_diff <= -0.1))  //平行对向
  {
    agent_pose_info.parallel_left_edge_head_point = trans_right_head_point;
    agent_pose_info.parallel_right_edge_head_point = trans_left_head_point;
    agent_pose_info.parallel_left_edge_tail_point = trans_right_tail_point;
    agent_pose_info.parallel_right_edge_tail_point = trans_left_tail_point;
  } else if (agent_pose_info.is_parallel)  //平行同向
  {
    agent_pose_info.parallel_left_edge_head_point = trans_left_head_point;
    agent_pose_info.parallel_right_edge_head_point = trans_right_head_point;
    agent_pose_info.parallel_left_edge_tail_point = trans_left_tail_point;
    agent_pose_info.parallel_right_edge_tail_point = trans_right_tail_point;
  } else if (sin_heading_diff >= 0.1)  //垂直向左
  {
    agent_pose_info.lower_conflict_edge_head_point = trans_left_head_point;
    agent_pose_info.lower_conflict_edge_tail_point = trans_left_tail_point;
    agent_pose_info.upper_conflict_edge_head_point = trans_right_head_point;
    agent_pose_info.upper_conflict_edge_tail_point = trans_right_tail_point;
  } else  //垂直向右
  {
    agent_pose_info.lower_conflict_edge_head_point = trans_right_head_point;
    agent_pose_info.lower_conflict_edge_tail_point = trans_right_tail_point;
    agent_pose_info.upper_conflict_edge_head_point = trans_left_head_point;
    agent_pose_info.upper_conflict_edge_tail_point = trans_left_tail_point;
  }

  // Calc distance info:
  if (!agent_pose_info.is_parallel)  // TODO by zx：其他场景不需要赋值么
  {
    // critical_y(x)_in(out) means the y( or x) -coordinate value of
    // conflict point when the agent cutin(out) in the front view of av
    const double critical_y_in =
        (sin_heading_diff < -0.1) ? av_half_width : -av_half_width;
    const double critical_y_out = -critical_y_in;
    const double cot_heading_diff = cos_heading_diff / sin_heading_diff;
    const double agent_distance_to_conflict_zone =
        (critical_y_in - agent_pose_info.lower_conflict_edge_head_point.y()) /
        sin_heading_diff;
    const double critical_x_in =
        agent_pose_info.lower_conflict_edge_head_point.x() +
        agent_distance_to_conflict_zone * cos_heading_diff;
    const double av_distance_to_conflict_zone =
        std::min(critical_x_in - 2.0 * critical_y_in * cot_heading_diff,
                 critical_x_in) -
        av_half_length;
    const double agent_distance_to_leave_conflict_zone =
        (critical_y_out - agent_pose_info.upper_conflict_edge_tail_point.y()) /
        sin_heading_diff;
    const double critical_x_out =
        agent_pose_info.upper_conflict_edge_tail_point.x() +
        agent_distance_to_leave_conflict_zone * cos_heading_diff;
    const double av_distance_to_leave_conflict_zone =
        std::max(critical_x_out + 2.0 * critical_y_in * cot_heading_diff,
                 critical_x_out) +
        av_half_length;
    agent_pose_info.av_distance_to_conflict_zone = av_distance_to_conflict_zone;
    agent_pose_info.av_distance_to_leave_conflict_zone =
        av_distance_to_leave_conflict_zone;
    agent_pose_info.agent_distance_to_conflict_zone =
        agent_distance_to_conflict_zone;
    agent_pose_info.agent_distance_to_leave_conflict_zone =
        agent_distance_to_leave_conflict_zone;
  }

  return agent_pose_info;
}

Risk RiskField::CalcAgentLateralRisk(const Box2d& av_box, double av_speed,
                                     const AgentPoseInfo& agent_pose_info,
                                     double agent_speed,
                                     const ObjectDecisionType& lateral_decision,
                                     const ObjBehavior& obj_behavior,
                                     bool use_lat_virtual_time,
                                     std::string* debug) {
  Risk lat_risk_for_yield;
  lat_risk_for_yield.risk = kInitRisk;
  if (agent_pose_info.av_distance_to_conflict_zone < -kEpsilon &&
      agent_speed > 0.5) {
    return lat_risk_for_yield;
  }

  double nudge_point_x = agent_pose_info.x_min;
  double nudge_point_y = 0.0;
  double virtual_time = 2.0;
  double risk = 0.0;
  double risk_x = 0.0;
  double risk_theta = 0.0;
  double risk_v = 0.0;
  double risk_t = 0.0;
  const double sin_heading_diff = agent_pose_info.sin_heading_diff;
  const double cos_heading_diff = agent_pose_info.cos_heading_diff;
  double dv_x = av_speed - agent_speed * agent_pose_info.cos_heading_diff;
  const double dv_x_sq = dv_x * dv_x;
  const double& av_half_length_front = av_box.half_length();
  const double& av_half_length_back = av_box.half_length();
  const double& av_half_width = av_box.half_width();

  // Determine nudge point y:
  double sig = 0.0;
  if (lateral_decision == ObjectDecisionType::OBJECT_DECISION_TYPE_LEFT_NUDGE) {
    nudge_point_y = agent_pose_info.y_max;
    sig = -1.0;
  } else if (lateral_decision ==
             ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE) {
    // Nudge point is the point with ymin;
    nudge_point_y = agent_pose_info.y_min;
    sig = 1.0;
  } else {
    return lat_risk_for_yield;
  }

  // Determine nudge point x:
  if (agent_pose_info.is_parallel || agent_pose_info.is_vertical) {
    nudge_point_x = agent_pose_info.x_min;
  } else {
    nudge_point_x = agent_pose_info.x_min;
    for (auto& p : agent_pose_info.agent_box_points) {
      if (std::abs(p.y() - nudge_point_y) <= kEpsilon) {
        nudge_point_x = p.x();
      }
    }
  }

  if (cos_heading_diff < 0.0 && nudge_point_x <= -av_half_length_back) {
    nudge_point_x = agent_pose_info.x_max;
    if (nudge_point_x <= -av_half_length_back) {
      return lat_risk_for_yield;
    }
  }
  // yield的障碍物，障碍物离开主车车头就解除风险
  if (use_lat_virtual_time && cos_heading_diff > 0.0 &&
      nudge_point_x > av_half_length_front) {
    nudge_point_x = agent_pose_info.x_min;
    if (nudge_point_x > av_half_length_front) {
      return lat_risk_for_yield;
    }
  }
  // pass的障碍物，需要拉开距离才能解除风险
  if (!use_lat_virtual_time && cos_heading_diff > 0.0 &&
      nudge_point_x >
          av_half_length_front + kIdmK0ForCross * agent_speed + kIdmSafeDist &&
      dv_x <= -0.1) {
    nudge_point_x = agent_pose_info.x_min;
    if (nudge_point_x >
        av_half_length_front + kIdmK0ForCross * agent_speed + kIdmSafeDist) {
      return lat_risk_for_yield;
    }
  }
  if ((cos_heading_diff < 0.0 && dv_x >= 0.1) ||
      (cos_heading_diff > 0.0 && dv_x <= -0.1) ||
      (cos_heading_diff > 0.0 &&
       agent_pose_info.x_max >
           -av_half_length_back - kLongitudeDistDVCoeff * av_speed)) {
    double av_nudge_point_x = av_half_length_front;
    virtual_time = (nudge_point_x - av_nudge_point_x) / dv_x;
    if (!use_lat_virtual_time) {
      virtual_time = 0.0;
    }
    const double v2_sin_theta = agent_speed * sin_heading_diff;
    const double v2_cos_theta = agent_speed * cos_heading_diff;
    double y_T = nudge_point_y + virtual_time * v2_sin_theta;
    double lat_distance_T = sig * y_T - av_half_width;
    lat_risk_for_yield.lat_distance_T = lat_distance_T;
    const double min_lat_dis = (obj_behavior == ObjBehavior::SYNC_LEFT_TURN)
                                   ? kSyncMinLatDistance
                                   : kOppoMinLatDistance;
    double ideal_lat_distance = kLatDistDVCoeff * std::abs(dv_x) + min_lat_dis;
    const PiecewiseLinearFunction<double> shrinkage_coeff_plf(
        {-2.0, -1.0, 0.0, 0.5, 1.0, 1.5, 2.0},
        {0.2, 0.3, 0.4, 0.5, 0.7, 0.8, 1.0});
    double shrinkage_coeff = shrinkage_coeff_plf(lat_distance_T);
    lat_risk_for_yield.shrinkage_coeff = shrinkage_coeff;

    const double virtual_kappa = 0.00;
    risk = ideal_lat_distance - lat_distance_T -
           sig * virtual_time * agent_speed * virtual_kappa;
    risk_x = sig * (v2_sin_theta / dv_x);
    risk_theta =
        sig *
        (nudge_point_x + virtual_time * v2_cos_theta -
         v2_sin_theta * ((nudge_point_y + virtual_time * v2_sin_theta) / dv_x));
    risk_v = risk_x * virtual_time - virtual_time * virtual_kappa;
    risk_t = -risk_x * v2_cos_theta - sig * v2_sin_theta;
    // *debug += "lat_risk_keyinfo, virtual_time: " +
    // std::to_string(virtual_time) + " dv_x " + std::to_string(dv_x) +
    //           " lat_distance_T " + std::to_string(lat_distance_T) + "
    //           ideal_lat_distance " + std::to_string(ideal_lat_distance) +
    //           "\n";
  } else {
    return lat_risk_for_yield;
  }

  lat_risk_for_yield.risk = risk;
  lat_risk_for_yield.risk_x = risk_x;
  lat_risk_for_yield.risk_v = risk_v;
  lat_risk_for_yield.risk_theta = risk_theta;
  lat_risk_for_yield.risk_t = risk_t;
  lat_risk_for_yield.virtual_time = virtual_time;
  return lat_risk_for_yield;
}

EnvInfo RiskField::CalEnvInfo(const RoadInfo& road_info) {
  const double start_point_heading =
      NormalizeAngle(road_info.start_point_heading);
  const double cos_start_point_heading = std::cos(start_point_heading);
  const double sin_start_point_heading = std::sin(start_point_heading);
  Vec2d start_point_end(road_info.start_point.x() + cos_start_point_heading,
                        road_info.start_point.y() + sin_start_point_heading);

  const double go_point_heading = NormalizeAngle(road_info.go_point_heading);
  const double cos_go_point_heading = std::cos(go_point_heading);
  const double sin_go_point_heading = std::sin(go_point_heading);
  Vec2d go_point_end(road_info.go_point.x() + cos_go_point_heading,
                     road_info.go_point.y() + sin_go_point_heading);

  EnvInfo enm_info;
  enm_info.start_point_start = road_info.start_point;
  enm_info.start_point_end = start_point_end;
  enm_info.go_point_start = road_info.go_point;
  enm_info.go_point_end = go_point_end;
  enm_info.start_point_heading = start_point_heading;
  enm_info.sin_start_point_heading = sin_start_point_heading;
  enm_info.cos_start_point_heading = cos_start_point_heading;
  enm_info.go_point_heading = go_point_heading;
  enm_info.sin_go_point_heading = sin_go_point_heading;
  enm_info.cos_go_point_heading = cos_go_point_heading;
  enm_info.start_point_is_valid = road_info.start_point_is_valid;

  return enm_info;
}

Risk RiskField::CalGoPointEnvRisk(const Box2d& av_box, double av_speed,
                                  const EnvInfo& enm_info_origin,
                                  const ObjectDecisionType& lateral_decision,
                                  double virtual_time, std::string* debug) {
  double sign =
      lateral_decision == ObjectDecisionType::OBJECT_DECISION_TYPE_LEFT_NUDGE
          ? 1.0
          : -1.0;

  Risk enm_risk_for_yield;
  enm_risk_for_yield.risk = kInitRisk;
  EnvInfo enm_info = enm_info_origin;
  if (enm_info.start_point_is_valid) {
    double start_go_heading_diff = NormalizeAngle(enm_info.go_point_heading -
                                                  enm_info.start_point_heading);
    if (lateral_decision ==
        ObjectDecisionType::OBJECT_DECISION_TYPE_LEFT_NUDGE) {
      const PiecewiseLinearFunction<double> heading_compensate_degree_plf(
          {0.2 * M_PI, 0.55 * M_PI}, {0.11 * M_PI, 0.0});
      double heading_compensation =
          heading_compensate_degree_plf(start_go_heading_diff);
      enm_info.go_point_heading += heading_compensation;
    } else if (lateral_decision ==
               ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE) {
      const PiecewiseLinearFunction<double> heading_compensate_degree_plf(
          {-0.55 * M_PI, -0.2 * M_PI}, {-0.0, -0.05 * M_PI});
      double heading_compensation =
          heading_compensate_degree_plf(start_go_heading_diff);
      enm_info.go_point_heading += heading_compensation;
    } else {
    }
    enm_info.cos_go_point_heading = std::cos(enm_info.go_point_heading);
    enm_info.sin_go_point_heading = std::sin(enm_info.go_point_heading);
    Vec2d go_point_end(
        enm_info.go_point_start.x() + enm_info.cos_go_point_heading,
        enm_info.go_point_start.y() + enm_info.sin_go_point_heading);
    enm_info.go_point_end = go_point_end;
  }
  auto go_line = enm_info.go_point_end - enm_info.go_point_start;
  auto av_go_line = av_box.center() - enm_info.go_point_start;
  const double go_point_L0 = go_line.CrossProd(av_go_line);
  double go_point_heading_diff =
      NormalizeAngle(av_box.heading() - enm_info.go_point_heading);
  if (go_point_heading_diff < -0.5 * M_PI &&
      lateral_decision == ObjectDecisionType::OBJECT_DECISION_TYPE_LEFT_NUDGE) {
    go_point_heading_diff = -0.49 * M_PI;
  }
  if (go_point_heading_diff > 0.5 * M_PI &&
      lateral_decision ==
          ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE) {
    go_point_heading_diff = 0.49 * M_PI;
  }
  const double sin_go_point_heading_diff = std::sin(go_point_heading_diff);
  const double cos_go_point_heading_diff = std::cos(go_point_heading_diff);
  const double go_point_risk =
      -sign * (go_point_L0 +
               (av_box.half_length() + kTurnRadiusCompensation) *
                   sin_go_point_heading_diff +  // half_length_front是什么？
               virtual_time * av_speed * sin_go_point_heading_diff);

  double risk_ththa =
      -sign * virtual_time * av_speed * cos_go_point_heading_diff -
      sign * (av_box.half_length() + kTurnRadiusCompensation) *
          cos_go_point_heading_diff;
  double risk_v = -sign * virtual_time * sin_go_point_heading_diff;
  double risk_x = -sign * sin_go_point_heading_diff;
  double risk_t = 0.0;
  // *debug += "go point enm_risk,L0 " + std::to_string(go_point_L0) +
  //           " heading_diff: " + std::to_string(go_point_heading_diff) +
  //           " go_point_heading: " + std::to_string(enm_info.go_point_heading)
  //           + "\n";
  enm_risk_for_yield.risk = go_point_risk;
  enm_risk_for_yield.risk_theta = risk_ththa;
  enm_risk_for_yield.risk_v = risk_v;
  enm_risk_for_yield.risk_x = risk_x;
  enm_risk_for_yield.risk_t = risk_t;
  enm_risk_for_yield.virtual_time = virtual_time;

  return enm_risk_for_yield;
}

Risk RiskField::CalStartPointEnvRisk(const Box2d& av_box, double av_speed,
                                     const EnvInfo& enm_info,
                                     const ObjectDecisionType& lateral_decision,
                                     double virtual_time, std::string* debug) {
  double sign =
      lateral_decision == ObjectDecisionType::OBJECT_DECISION_TYPE_LEFT_NUDGE
          ? 1.0
          : -1.0;
  Risk enm_risk_for_yield;
  if (enm_info.start_point_is_valid) {
    auto start_line = enm_info.start_point_end - enm_info.start_point_start;
    auto av_line = av_box.center() - enm_info.start_point_start;
    const double start_point_L0 = start_line.CrossProd(av_line);
    const double start_point_heading_diff =
        NormalizeAngle(av_box.heading() - enm_info.start_point_heading);
    const double sin_start_point_heading_diff =
        std::sin(start_point_heading_diff);
    const double cos_start_point_heading_diff =
        std::cos(start_point_heading_diff);
    const double start_point_risk =
        -sign *
        (start_point_L0 + av_box.half_length() * sin_start_point_heading_diff +
         virtual_time * av_speed * sin_start_point_heading_diff);
    double risk_ththa =
        -sign * virtual_time * av_speed * cos_start_point_heading_diff -
        sign * av_box.half_length() * cos_start_point_heading_diff;
    double risk_v = -sign * virtual_time * sin_start_point_heading_diff;
    double risk_x = -sign * sin_start_point_heading_diff;
    double risk_t = 0.0;
    enm_risk_for_yield.risk = start_point_risk;
    enm_risk_for_yield.risk_theta = risk_ththa;
    enm_risk_for_yield.risk_v = risk_v;
    enm_risk_for_yield.risk_x = risk_x;
    enm_risk_for_yield.risk_t = risk_t;
    enm_risk_for_yield.virtual_time = virtual_time;
  }

  if (enm_info.start_point_is_valid) {
    PiecewiseLinearFunction<double> length_degree_plf(
        {0.5 * M_PI, 3.0 * M_PI / 4.0}, {0.5, 14.0});
    if (lateral_decision ==
        ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE) {
      length_degree_plf = PiecewiseLinearFunction<double>(
          {-3.0 * M_PI / 4.0, -0.5 * M_PI}, {10.0, 0.5});
    }
    auto inserted_point =
        GetLineInsertion(enm_info.go_point_start, enm_info.go_point_end,
                         enm_info.start_point_start, enm_info.start_point_end);
    if (!inserted_point.has_value()) {
      return enm_risk_for_yield;
    }
    //交点如果在start point后面，说明有异常
    double start_to_inserted_heading =
        atan2(inserted_point->y() - enm_info.start_point_start.y(),
              inserted_point->x() - enm_info.start_point_start.x());
    if (cos(start_to_inserted_heading - enm_info.start_point_heading) < 0.5) {
      return enm_risk_for_yield;
    }
    double inserted_angle = NormalizeAngle(enm_info.go_point_heading -
                                           enm_info.start_point_heading);
    double length = length_degree_plf(inserted_angle);
    // *debug += "inserted_angle: " + std::to_string(inserted_angle) + " length:
    // " + std::to_string(length) + "\n";
    Vec2d upper_point_start;
    Vec2d upper_point_end;
    upper_point_start.set_x(inserted_point->x() +
                            length * cos(enm_info.start_point_heading + M_PI));
    upper_point_start.set_y(inserted_point->y() +
                            length * sin(enm_info.start_point_heading + M_PI));
    upper_point_end.set_x(inserted_point->x() +
                          length * enm_info.cos_go_point_heading);
    upper_point_end.set_y(inserted_point->y() +
                          length * enm_info.sin_go_point_heading);
    double upper_point_heading =
        atan2(upper_point_end.y() - upper_point_start.y(),
              upper_point_end.x() - upper_point_start.x());
    const double upper_segment_length =
        (upper_point_end - upper_point_start).Length();
    auto upper_line = upper_point_end - upper_point_start;
    const double upper_point_L0 =
        upper_line.CrossProd(av_box.center() - upper_point_start) /
        upper_segment_length;
    const double upper_point_heading_diff =
        NormalizeAngle(av_box.heading() - upper_point_heading);
    const double sin_upper_point_heading_diff =
        std::sin(upper_point_heading_diff);
    const double cos_upper_point_heading_diff =
        std::cos(upper_point_heading_diff);
    const double upper_point_risk =
        -sign *
        (upper_point_L0 + av_box.half_length() * sin_upper_point_heading_diff +
         virtual_time * av_speed * sin_upper_point_heading_diff);
    if (upper_point_risk > enm_risk_for_yield.risk) {
      double risk_ththa =
          -sign * virtual_time * av_speed * cos_upper_point_heading_diff -
          sign * av_box.half_length() * cos_upper_point_heading_diff;
      double risk_v = -sign * virtual_time * sin_upper_point_heading_diff;
      double risk_x = -sign * sin_upper_point_heading_diff;
      double risk_t = 0.0;
      enm_risk_for_yield.risk = upper_point_risk;
      enm_risk_for_yield.risk_theta = risk_ththa;
      enm_risk_for_yield.risk_v = risk_v;
      enm_risk_for_yield.risk_x = risk_x;
      enm_risk_for_yield.risk_t = risk_t;
      enm_risk_for_yield.virtual_time = virtual_time;
    }
  }

  return enm_risk_for_yield;
}

Risk RiskField::CalcAgentLongitudinalRiskForCrossYield(
    const Box2d& av_box, double av_speed, const AgentPoseInfo& agent_pose_info,
    double agent_speed, std::string* debug) {
  // Calculate the longitudinal risk for the decision yield
  // d1 is the distance of av to the virtual conflict-zone
  Risk longitudinal_risk_for_yield;
  longitudinal_risk_for_yield.risk = kInitRisk;
  const double d1 = agent_pose_info.av_distance_to_conflict_zone;
  // d2 is the distance of agent vehicle to
  const double d2 = agent_pose_info.agent_distance_to_leave_conflict_zone;
  const double av_half_length_front = av_box.half_length();
  const double av_half_width = av_box.half_width();
  const double sin_heading_diff = agent_pose_info.sin_heading_diff;
  const double cos_heading_diff = agent_pose_info.cos_heading_diff;
  if (cos_heading_diff > 0.0 && agent_pose_info.x_min > av_half_length_front) {
    return longitudinal_risk_for_yield;
  }
  // TODO: when collision zone near ego or back of ego, need gaming deduce
  if (!agent_pose_info.is_parallel && d1 > kEpsilon) {
    const double cot_heading_diff = cos_heading_diff / sin_heading_diff;
    const double d1_theta =
        av_half_width + (av_half_length_front + d1) * cot_heading_diff;
    // The partial derivative of d2 w.r.t. theta, the heading of av
    const double d2_theta = (av_half_length_front + d1) / sin_heading_diff;

    // The Ideal Distance to the conflict zone
    const double ideal_time_gap =
        d2 / std::max(agent_speed, 0.1) + kIdmK0ForCross +
        kIdmAlpha * (av_speed - agent_speed * cos_heading_diff);

    // Partial derivative of ideal_time_gap w.r.t. theta
    const double ideal_time_gap_theta = d2_theta / std::max(agent_speed, 0.1);

    // Partial derivative of ideal_time_gap w.r.t. av_speed
    const double ideal_time_gap_v = kIdmAlpha;

    const double risk = ideal_time_gap * av_speed + kIdmSafeDist - d1;
    // *debug += "long_risk key info: \n";
    // *debug += "av_speed: " + std::to_string(av_speed) + "\n";
    // *debug += "d1: " + std::to_string(d1) + "\n";
    // *debug += "d2: " + std::to_string(d2) + "\n";
    // *debug += "ideal_time_gap: " + std::to_string(ideal_time_gap) + "\n";
    // *debug += "ideal_time_gap_v: " + std::to_string(ideal_time_gap_v) + "\n";

    // The partial derivative of risk w.r.t. x
    longitudinal_risk_for_yield.risk = risk;
    longitudinal_risk_for_yield.risk_x = 1.0;
    longitudinal_risk_for_yield.risk_theta =
        ideal_time_gap_theta * av_speed - d1_theta;
    longitudinal_risk_for_yield.risk_v =
        ideal_time_gap + ideal_time_gap_v * av_speed;
    longitudinal_risk_for_yield.risk_t = -av_speed;
  }

  return longitudinal_risk_for_yield;
}

VehicleState RiskField::CalcNextVehicleStateWithConstKappa(
    const VehicleState& vehicle_state, double dt) {
  VehicleState av_state_i_plus;
  const double vehicle_theta = (vehicle_state.theta);
  double cos_theta_cur = std::cos(vehicle_theta);
  double sin_theta_cur = std::sin(vehicle_theta);
  double v_next = std::max(vehicle_state.v, 1.0);
  double theta_next = NormalizeAngle(
      vehicle_state.theta + (vehicle_state.kappa * vehicle_state.v) * dt);
  double cos_theta_next = std::cos(theta_next);
  double sin_theta_next = std::sin(theta_next);
  double x_next =
      vehicle_state.x +
      0.5 * (vehicle_state.v * cos_theta_cur + v_next * cos_theta_next) * dt;
  double y_next =
      vehicle_state.y +
      0.5 * (vehicle_state.v * sin_theta_cur + v_next * sin_theta_next) * dt;
  av_state_i_plus.x = x_next;
  av_state_i_plus.y = y_next;
  av_state_i_plus.theta = theta_next;
  av_state_i_plus.kappa = vehicle_state.kappa;
  av_state_i_plus.v = v_next;
  av_state_i_plus.a = 0.0;

  return av_state_i_plus;
}
AgentPoseInfo RiskField::CalcLookForwardPoseInfo(
    const VehicleState& vehicle_state, const Box2d& av_box,
    const Box2d& agent_box, double cur_simu_time, double go_point_heading) {
  const double dt = 0.4;
  const double max_simlation_time = 6.0;
  const double simulation_stop_heading_diff_thresh = 0.2 * M_PI;
  int N = std::floor((max_simlation_time - cur_simu_time) / dt);
  int start_index = std::floor(cur_simu_time / dt);
  VehicleState next_vehicle_state = vehicle_state;
  double critical_time = 0.0;
  double critical_s = 0.0;
  AgentPoseInfo critical_pose_info;
  for (int i = 0; i < N; ++i) {
    next_vehicle_state =
        CalcNextVehicleStateWithConstKappa(next_vehicle_state, dt);
    critical_time = dt * i;
    critical_s += dt * next_vehicle_state.v;
    const Box2d look_forward_av_box(
        Vec2d(next_vehicle_state.x, next_vehicle_state.y),
        next_vehicle_state.theta, av_box.length(), av_box.width());
    critical_pose_info = CalcAgentPoseInfo(look_forward_av_box, agent_box);
    if (std::fabs(next_vehicle_state.theta - go_point_heading) <
        simulation_stop_heading_diff_thresh) {
      break;
    }
    if (critical_pose_info.av_distance_to_conflict_zone <= 3.0 &&
        !critical_pose_info.is_parallel) {
      break;
    }
  }
  critical_pose_info.av_distance_to_conflict_zone += critical_s;
  return critical_pose_info;
}
Risk RiskField::CalcAgentLookForwardLongitudinalRiskForCrossYield(
    const Box2d& av_box, double av_speed, double lookforward_kappa,
    const AgentPoseInfo& agent_pose_info, double agent_speed,
    std::string* debug) {
  // Calculate the longitudinal risk for the decision yield
  // d1 is the distance of av to the virtual conflict-zone
  Risk longitudinal_risk_for_yield;
  longitudinal_risk_for_yield.risk = kInitRisk;
  const double d1 = agent_pose_info.av_distance_to_conflict_zone;
  // d2 is the distance of agent vehicle to
  const PiecewiseLinearFunction<double> kappa_buffer_plf(
      {0.001, 0.01, 0.03, 0.05, 0.1}, {0.0, 1.0, 1.0, 1.0, 1.0});
  const double d2 = agent_pose_info.agent_distance_to_leave_conflict_zone +
                    kappa_buffer_plf(std::abs(lookforward_kappa));
  const double av_half_length_front = av_box.half_length();
  const double av_half_width = av_box.half_width();
  const double sin_heading_diff = agent_pose_info.sin_heading_diff;
  const double cos_heading_diff = agent_pose_info.cos_heading_diff;
  if (cos_heading_diff > 0.0 && agent_pose_info.x_min > av_half_length_front) {
    return longitudinal_risk_for_yield;
  }
  // TODO: when collision zone near ego or back of ego, need gaming deduce
  if (!agent_pose_info.is_parallel && d1 > kEpsilon) {
    const double cot_heading_diff = cos_heading_diff / sin_heading_diff;
    const double d1_theta =
        av_half_width + (av_half_length_front + d1) * cot_heading_diff;
    // The partial derivative of d2 w.r.t. theta, the heading of av
    const double d2_theta = (av_half_length_front + d1) / sin_heading_diff;

    // The Ideal Distance to the conflict zone
    const double ideal_time_gap =
        d2 / std::max(agent_speed, 0.1) + kIdmK0ForCross +
        kIdmAlpha * (av_speed - agent_speed * cos_heading_diff);

    // Partial derivative of ideal_time_gap w.r.t. theta
    const double ideal_time_gap_theta = d2_theta / std::max(agent_speed, 0.1);

    // Partial derivative of ideal_time_gap w.r.t. av_speed
    const double ideal_time_gap_v = kIdmAlpha;

    const double risk = ideal_time_gap * av_speed + kIdmSafeDist - d1;
    // *debug += "long_risk key info: \n";
    // *debug += "av_speed: " + std::to_string(av_speed) + "\n";
    // *debug += "d1: " + std::to_string(d1) + "\n";
    // *debug += "d2: " + std::to_string(d2) + "\n";
    // *debug += "ideal_time_gap: " + std::to_string(ideal_time_gap) + "\n";

    // The partial derivative of risk w.r.t. x
    longitudinal_risk_for_yield.risk = risk;
    longitudinal_risk_for_yield.risk_x = 1.0;
    longitudinal_risk_for_yield.risk_theta =
        ideal_time_gap_theta * av_speed - d1_theta;
    longitudinal_risk_for_yield.risk_theta =
        std::min(longitudinal_risk_for_yield.risk_theta, 25.0);
    longitudinal_risk_for_yield.risk_theta =
        std::max(longitudinal_risk_for_yield.risk_theta, -25.0);
    longitudinal_risk_for_yield.risk_v =
        ideal_time_gap + ideal_time_gap_v * av_speed;
    longitudinal_risk_for_yield.risk_t = -av_speed;
  }

  return longitudinal_risk_for_yield;
}

}  // namespace st::planning