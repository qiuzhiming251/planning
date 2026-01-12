#include "acc/acc_path_corridor_map.h"

#include <fstream>
#include <optional>
#include <string>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "acc/acc_path_corridor.h"
#include "acc/acc_util.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/ref_line/smooth_reference_line_result.h"
#include "planner/planner_manager/planner_defs.h"
#include "router/drive_passage_builder.h"
#include "decider/scheduler/path_boundary_builder.h"
#include "plan_common/util/gradient_points_smoother.h"

#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "decider/scheduler/fsd_lane_selector.h"
namespace st::planning {

namespace {
inline constexpr double kPreviewSecs = 8.0;                      // s.
inline constexpr double kMinPathLength = 80.0;                   // m.
inline constexpr double kMaxPathLength = 350.0;                  // m.
inline constexpr double kCorridorExtendLengthTimeHorizon = 4.0;  // s.
inline constexpr double kMaxLateralBiasMeters = 0.7;             // m.
using LaneConstPtr = std::shared_ptr<const ::ad_byd::planning::Lane>;
using MapPtr = std::shared_ptr<::ad_byd::planning::Map>;
using LaneSequencePtr = ad_byd::planning::LaneSequencePtr;
using ElementId = uint64_t;

void GenerateCurrentLaneSequencesId(const Vec2d &start_point,
                                  const double start_point_v,
                                  const LaneConstPtr &start_lane,
                                  const MapPtr &map, 
                                  std::vector<ElementId>& lane_ids,
                                  LaneSequencePtr &cur_lane_sequence) 
{
  const auto &nearest_lane = start_lane;
  if (!nearest_lane) {
    LOG_ERROR << "cannot generate lane sequence: nearest_lane donot exist!!!";
    return;
  }

  if (!cur_lane_sequence) {
    std::vector<LaneConstPtr> t_lanes;
    t_lanes.emplace_back(nearest_lane);
    cur_lane_sequence = std::make_shared<::ad_byd::planning::LaneSequence>(t_lanes);
  }

  const auto &last_lanes = cur_lane_sequence;
  if (!last_lanes) {
    LOG_ERROR << "cannot generate lane sequence: cur_lane sequence donot exist!!!";
    return;
  }

  // step 1. get lanes-seq from last_lanes
  auto lanes = last_lanes->lanes();

  // step 2. get next lane forward until end
  LaneConstPtr next_lane = lanes.back();

  std::unordered_set<uint64_t> check_lanes;
  while (true) {
   
    if (map->is_on_highway()) {
      next_lane = map->GetOptimalNextLane(next_lane, true, 0);
    } 
    else {
      next_lane = map->GetContinueNextLane(next_lane->id());
    }
    if ((!next_lane) || next_lane->id() == 0 ||
        check_lanes.count(next_lane->id()) > 0) {
      break;
    }
    check_lanes.insert(next_lane->id());
    lanes.emplace_back(next_lane);
  };
  LOG_INFO << "lane sequence size is : " << lanes.size();
  for(int i = 0; i < lanes.size(); i++){
    lane_ids.push_back(lanes[i]->id());
    LOG_INFO << "lane id is : " << lanes[i]->id();
  }
}

/**
 * @return true if updated
 */
bool MovingAverageKappa(DiscretizedPath* path,
                        std::vector<double> window_weight, int iter) {
  if (path->size() < window_weight.size() || window_weight.size() < 1) {
    return false;
  }
  const int n = window_weight.size();
  for (int i = 0; i < iter; ++i) {
    for (int j = 0; j < path->size() - n; ++j) {
      double avg = 0.0;
      for (int w = 0; w < n; w++) {
        avg += window_weight[w] * (*path)[j + w].kappa();
      }
      (*path)[j].set_kappa(avg);
    }
  }
  return true;
}

inline std::pair<double, double> EstimateAccelAndConstSpeedTime(double v0,
                                                                double a,
                                                                double max_v,
                                                                double s) {
  constexpr double kLatSpeedEpsilon = 0.05;
  constexpr double kConstAccelEpsilon = 0.01;
  if (max_v < v0 + kLatSpeedEpsilon) {
    max_v = v0;
  }
  if (a < kConstAccelEpsilon) {
    return {s / max_v, 0.0};
  }
  double t1 = (max_v - v0) / a;
  double t2 = (s - 0.5 * a * t1 * t1 - v0 * t1) / max_v;
  if (t2 <= 0.0) {
    t1 = (-v0 + std::sqrt(v0 * v0 + 2 * a * s)) / a;  // QuadraticRoot()
  }
  t1 = std::clamp(t1, 0.0, 3.0);
  t2 = std::clamp(t2, 0.0, 5.0);
  return {t1, t2};
}

struct PointToLaneMatchResult {
  bool valid = false;
  FrenetCoordinate sl;
  mapping::LanePoint lane_pt;
  ad_byd::planning::LaneConstPtr lane_ptr = nullptr;
  std::string DebugString() const;
};

std::string PointToLaneMatchResult::DebugString() const {
  return absl::StrFormat("valid: %d, sl(%.2f, %.2f), lane_pt(%s,%.2f)", valid,
                         sl.s, sl.l, lane_pt.lane_id(), lane_pt.fraction());
}

inline double CalcSpeedHeadingFactor(double mps) {
  static const PiecewiseLinearFunction<double, double> kMpsToHeadingFactorPlf =
      {{0.0, 30}, {10, 30}};
  return kMpsToHeadingFactorPlf(mps);
}

absl::StatusOr<PointToLaneMatchResult> FindBestLane(
    const PlannerSemanticMapManager& psmm, Vec2d start_pos, double radius,
    double heading, double speed) {
  const auto& nearest_lane =
      psmm.GetNearestLaneWithHeading(start_pos, heading, radius, M_PI_4);
  if (nearest_lane == nullptr) {
    return absl::NotFoundError(
        absl::StrFormat("Cannot find nearest lane. pos: (%.2f, %.2f)",
                        start_pos.x(), start_pos.y()));
  }
  std::vector<ad_byd::planning::LaneConstPtr> candidate_start_lanes;
  if (!nearest_lane->IsVirtual() && nearest_lane->center_line().IsValid()) {
    VLOG(3) << "Ignore nearest virtual lane. " << nearest_lane->id();
    candidate_start_lanes.push_back(nearest_lane);
  } else {
    const auto& left_lane =
        psmm.FindCurveLaneByIdOrNull(nearest_lane->left_lane_id());
    const auto& right_lane =
        psmm.FindCurveLaneByIdOrNull(nearest_lane->right_lane_id());
    if (left_lane != nullptr && !left_lane->IsVirtual() &&
        left_lane->center_line().IsValid()) {
      candidate_start_lanes.push_back(left_lane);
    }
    if (right_lane != nullptr && !right_lane->IsVirtual() &&
        right_lane->center_line().IsValid()) {
      candidate_start_lanes.push_back(right_lane);
    }
  }

  std::vector<PointToLaneMatchResult> lane_point_matches;

  for (const auto& lane_ptr : candidate_start_lanes) {
    auto smooth_or =
        ResampleAndSmoothPoints(lane_ptr->points(), /*tolerance=*/1.0,
                                /*max_iter*/ 200, /*eps=*/0.01, /*alpha=*/0.15);
    if (smooth_or.ok() && smooth_or->size() > 2) {
      ad_byd::planning::math::LineCurve2d smooth_center_line(*smooth_or);
      auto* center_line = const_cast<ad_byd::planning::math::LineCurve2d*>(
          &lane_ptr->center_line());
      center_line->Clear();
      *center_line = smooth_center_line;
      auto* lane_info =
          const_cast<ad_byd::planning::LaneInfo*>(&lane_ptr->lane_info());
      lane_info->points = *smooth_or;
    }
    double start_fraction = 0.0;
    const auto ff =
        BuildBruteForceFrenetFrame(lane_ptr->points(),
                                   /*down_sample_raw_points=*/false);
    if (!ff.ok()) {
      continue;
    };
    FrenetCoordinate sl;
    Vec2d normal;
    std::pair<int, int> index_pair;
    double alpha;
    ff->XYToSL(start_pos, &sl, &normal, &index_pair, &alpha);
    start_fraction = std::clamp(
        sl.s / std::fmax(lane_ptr->curve_length(), 1e-2), 0.0, 1.0 - 1e-2);

    if (std::fabs(sl.l) > kMaxLateralBiasMeters) {
      continue;
    }
    lane_point_matches.push_back({
        .valid = true,
        .sl = sl,
        .lane_pt = {lane_ptr->id(), start_fraction},
        .lane_ptr = lane_ptr,
    });
  }
  if (lane_point_matches.empty()) {
    return absl::NotFoundError("Cannot find available lane paths.");
  }
  // Need more details to calc cost.
  auto& match_result =
      *std::min_element(lane_point_matches.begin(), lane_point_matches.end(),
                        [](const auto& lhs, const auto& rhs) {
                          return std::fabs(lhs.sl.l) < std::fabs(rhs.sl.l);
                        });

  VLOG(3) << "Choose the best lane path: " << match_result.DebugString();
  return std::move(match_result);
}

}  // namespace

AccPathCorridor BuildAccPathCorridorWithMap(
    const PlannerSemanticMapManager& psmm,
    const SmoothedReferenceLineResultMap& smooth_result_map,
    const PoseProto& pose, const SpacetimeTrajectoryManager& st_traj_mgr,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, double corridor_step_s,
    double total_preview_time, std::optional<double> cruising_speed_limit) {
  Vec2d av_pos = {pose.pos_smooth().x(), pose.pos_smooth().y()};
  Vec2d start_pos = {plan_start_point.path_point().x(),
                     plan_start_point.path_point().y()};

  auto match_result_or = FindBestLane(psmm, start_pos,
                                      /*radius=*/kDefaultLaneWidth, pose.yaw(),
                                      pose.vel_body().x());
  if (!match_result_or.ok()) {
    return BuildErrorAccPathCorridorResult(
        std::string(match_result_or.status().message()));
  }
  //has checked already
  // if (std::fabs(match_result_or->sl.l) > kMaxLateralBiasMeters) {
  //   LOG_INFO << "false@@2:match_result_or->sl.l not ok!";
  //   return BuildErrorAccPathCorridorResult("Too lateral bias to center lines.");
  // }

  //The ids of currently found best lane and its successors
  std::vector<ElementId> lane_ids_acc;
  ad_byd::planning::LaneSequencePtr cur_lane_sequence = nullptr;
  GenerateCurrentLaneSequencesId(start_pos , plan_start_point.v() , 
                                 match_result_or->lane_ptr , psmm.map_ptr() , lane_ids_acc , cur_lane_sequence);
  //{match_result_or->lane_pt.lane_id()}
  mapping::LanePath lane_path(
      psmm.map_ptr(), lane_ids_acc,
      match_result_or->lane_pt.fraction(),
      /*end_fraction=*/1.0, /*lane_path_in_forward_direction=*/true);
  constexpr double kMinAccLanePathLength = 10.0;
  constexpr double kMinAccLanePathChecked = 30.0;
  if (lane_path.length() < kMinAccLanePathChecked) {
    return BuildErrorAccPathCorridorResult("Too short lane path.");
  }

  double required_planning_horizon = std::clamp(
      pose.vel_body().x() * (kPreviewSecs + kCorridorExtendLengthTimeHorizon),
      kMinPathLength, kMaxPathLength);
  auto dp_or = BuildDrivePassageFromLanePath(
      psmm, lane_path, corridor_step_s, /*avoid_loop=*/false,
      /*avoid_notcontinuous=*/false,
      /*backward_extend_len=*/10.0, required_planning_horizon,
      cruising_speed_limit, FrenetFrameType::kQtfmKdTree);
  if (!dp_or.ok() || dp_or->size() <= 1) {
    return BuildErrorAccPathCorridorResult(
        std::string(dp_or.status().message()));
  }
  auto dp = std::move(dp_or).value();
  auto sl_boundary_or = BuildPathBoundaryFromPose(
      psmm, dp, plan_start_point, vehicle_geom_params, st_traj_mgr,
      LaneChangeStateProto(), smooth_result_map,
      /*borrow=*/false, /*shoule_smooth=*/true);
  if (!sl_boundary_or.ok()) {
    return BuildErrorAccPathCorridorResult(
        std::string(sl_boundary_or.status().message()));
  }
  const Station& last_station = dp.station(StationIndex(dp.size() - 1));
  const auto start_pos_sl_or = dp.QueryFrenetCoordinateAt(start_pos);
  if (!start_pos_sl_or.ok()) {
    return BuildErrorAccPathCorridorResult(
        std::string(start_pos_sl_or.status().message()));
  }
  const double start_s = start_pos_sl_or->s;
  const double start_l = start_pos_sl_or->l;
  // Need pp state to dp center line.
  double loaded_map_dist = last_station.accumulated_s() - start_s;
  if (loaded_map_dist < 0.0) {
    return BuildErrorAccPathCorridorResult("Out of lane path range.");
  }
  loaded_map_dist = lane_path.length() - start_s;
  loaded_map_dist = std::clamp(loaded_map_dist, kMinPathLength, kMaxPathLength);

  const double extend_length = std::clamp(
      std::fabs(GetLonSpeed(pose)) * (total_preview_time + kPreviewSecs),
      kMinPathLength, kMaxPathLength);

  const auto [accel_t, const_t] = EstimateAccelAndConstSpeedTime(
      std::fabs(GetLatSpeed(pose)),
      /*a=*/0.28, /*max_v=*/1.0, std::fabs(start_l));
  const double t = accel_t + const_t;
  constexpr double kMinEstimateLen = 20.0;
  constexpr double kMaxEstimateLen = 80.0;
  const double concat_dist =
      std::clamp(GetLonSpeed(pose) * t, kMinEstimateLen, kMaxEstimateLen);
  std::vector<Vec2d> center_xy;
  std::vector<double> s_vec;
  const double kDrivePassageStepS = 2.0;
  const int concat_n = concat_dist / kDrivePassageStepS + 1;
  center_xy.reserve(concat_n);
  s_vec.reserve(concat_n);
  // Resample points.
  double l = start_l;
  double laterl_step = 1.0 / concat_n * start_l;
  center_xy.push_back(start_pos);
  for (auto s = start_s + kDrivePassageStepS; s < last_station.accumulated_s();
       s += kDrivePassageStepS) {
    auto pt = sl_boundary_or->QueryReferenceCenterXY(s);
    center_xy.push_back(pt);
  }
  constexpr double kSmoothTolerance = 1.0;  // m.
  const auto smooth_or =
      ResampleAndSmoothPoints(center_xy, kDrivePassageStepS, /*max_iter*/ 100,
                              /*eps=*/0.01, /*alpha=*/0.15);
  if (smooth_or.ok()) {
    center_xy = *smooth_or;
  }
  s_vec.clear();
  s_vec.reserve(center_xy.size());
  s_vec.push_back(0.0);
  int i, j;
  for (i = 0, j = i + 1; j < static_cast<int>(center_xy.size());) {
    double dist = (center_xy[j] - center_xy[i]).norm();
    if (dist < kSmoothTolerance) {
      j++;
      continue;
    } else {
      s_vec.push_back(s_vec.back() + dist);
      center_xy[i + 1] = center_xy[j];
      ++i;
      ++j;
    }
  }
  center_xy.resize(s_vec.size());
  if (s_vec.back() < kMinAccLanePathLength) {
    return BuildErrorAccPathCorridorResult("Too short lane path length");
  }

  double extended_length =
      std::max(std::fabs(GetLonSpeed(pose)) *
                   (total_preview_time + kCorridorExtendLengthTimeHorizon),
               loaded_map_dist);
  extended_length = std::min(extended_length, kMaxPathLength);
  LOG_EVERY_N(INFO, 30) << "s.size: " << s_vec.size()
                        << ", center.size:" << center_xy.size()
                        << "dp length: " << last_station.accumulated_s()
                        << ", start_s: " << start_s << ", start_l: " << start_l
                        << ", corridor_step_s: " << corridor_step_s
                        << ", accel_t: " << accel_t << ", const_t: " << const_t
                        << ", t: " << t
                        << ", extended_length: " << extended_length
                        << ", concat_dist: " << concat_dist
                        << ", loaded_map_dist: " << loaded_map_dist
                        << ", heading: " << last_station.tangent()
                        << ", plan_start_point: "
                        << plan_start_point.ShortDebugString();

  auto path = ExtendedCubicSplinePath(center_xy, s_vec, last_station.tangent(),
                                      corridor_step_s, kPathSampleInterval,
                                      extended_length, kMaxAllowedAccKappa);
  std::vector<double> s_plf;
  std::vector<double> kappa_plf;
  s_plf.push_back(0.0);
  kappa_plf.push_back(path.Evaluate(concat_dist).kappa());
  s_plf.push_back(loaded_map_dist);
  kappa_plf.push_back(path.Evaluate(loaded_map_dist).kappa());
  bool debug_kappa = false;
  if (debug_kappa) {
    static int i = 0;
    std::ofstream f(std::string(absl::StrFormat("kappa_%03d.csv", i)),
                    std::ios::trunc);
    for (const auto& pt : path) {
      f << pt.s() << "," << pt.kappa() << '\n';
    }
    std::ofstream xy_f(std::string(absl::StrFormat("xy_%03d.csv", i)),
                       std::ios::trunc);
    for (const auto& pt : path) {
      xy_f << pt.x() << "," << pt.y() << '\n';
    }

    std::ofstream concat_f(std::string(absl::StrFormat("concat_%03d.csv", i)),
                           std::ios::trunc);
    concat_f << concat_dist << "," << loaded_map_dist << '\n';
    i++;
  }

  MovingAverageKappa(&path,
                     {
                         1.0 / 10,
                         1.0 / 10,
                         1.0 / 10,
                         1.0 / 10,
                         1.0 / 10,
                         1.0 / 10,
                         1.0 / 10,
                         1.0 / 10,
                         1.0 / 10,
                         1.0 / 10,
                     },
                     /*iter = */ 10);
  if (debug_kappa) {
    static int i = 0;
    std::ofstream f(std::string(absl::StrFormat("smooth_kappa_%03d.csv", i)),
                    std::ios::trunc);
    for (const auto& pt : path) {
      f << pt.s() << "," << pt.kappa() << '\n';
    }
    i++;
  }
  //kappa check within the first kMinAccLanePathChecked(m) of the path
  double max_kappa = std::numeric_limits<double>::min();
  double kMaxAllowedkappa = 1.0/100.0; //150m
  int pt_num = 1;
  for(const auto& pt : path){
    if(pt.s() > kMinAccLanePathChecked){
      break;
    }
    max_kappa = std::max(max_kappa,std::fabs(pt.kappa()));
    pt_num++;
  }
  LOG_INFO << "[BuildAccPathCorridorWithMap]max_kappa is : " << max_kappa;
  
  if (max_kappa > kMaxAllowedkappa) {
    return BuildErrorAccPathCorridorResult("The first kMinAccLanePathChecked(m) of the path has big kappa");
  }
  return {
      .build_status = OkPlannerStatus(),
      .type = AccPathCorridorType::MAP,
      .boundary = std::move(sl_boundary_or).value(),
      .frenet_frame =
          std::move(dp.MoveFrenetFrame()),  // ! DONOT use dp after move.
      .path = std::move(path),
      .loaded_map_dist = loaded_map_dist,
      .kappa_s = {s_plf, kappa_plf},
  };
}

}  // namespace st::planning
