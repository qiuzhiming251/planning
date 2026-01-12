#include "acc/acc_path_corridor_vehicle.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "acc/acc_path_corridor.h"
#include "acc/acc_util.h"
#include "plan_common/math/cubic_spline.h"
#include "plan_common/math/fast_math.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/timer.h"
#include "planner/planner_manager/planner_defs.h"

namespace st::planning {
namespace {
constexpr double kCredibleCorridorRadian = M_PI / 3;             // 60.
constexpr double kMinPathLength = 80.0;                          // m.
constexpr double kMaxPathLength = 350.0;                         // m.
constexpr double kExpandedBoundaryMinS = 9.0;                    // m.
constexpr double kExpandedBoundaryMaxS = 21.0;                   // m.
inline constexpr double kCorridorExtendLengthTimeHorizon = 8.0;  // s.

inline double CalcLoadLength(double mps) {
  static const PiecewiseLinearFunction<double, double> kMpsToLoadMetersPlf = {
      {0.0, 10.0, 20.0, 30}, {20.0, 60.0, 100.0, 150.0}};
  return kMpsToLoadMetersPlf(mps);
}

const PiecewiseLinearFunction<double, double> kInnerBoundaryHalfWidthPlf = {
    /*s_ratio=*/{0.5, 0.75, 1.0},
    /*inner_boundary_half_width=*/{1.2, 1.0, 0.8}};
const PiecewiseLinearFunction<double, double> kOuterBoundaryHalfWidthPlf = {
    /*s_ratio=*/{0.5, 0.75, 1.0},
    /*outer_boundary_half_width=*/{1.5, 1.3, 1.1}};
const PiecewiseLinearFunction<double, double> kExpandedBoundaryHalfWidthPlf = {
    /*steering angle(degree)=*/{20.0, 30.0, 50},
    /*expanded_boundary_half_width=*/{1.5, 2.0, 2.5}};
}  // namespace

AccPathCorridor BuildAccPathCorridorWithoutMap(
    const PoseProto& pose, const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, double corridor_step_s,
    double total_preview_time, double front_wheel_angle, double av_kappa) {
  Timer timer(__FUNCTION__);

  double load_length = CalcLoadLength(GetLonSpeed(pose));
  load_length =
      std::min(kCredibleCorridorRadian / (std::fabs(av_kappa) + kKappaEpsilon),
               load_length);
  const int n = static_cast<int>(load_length / corridor_step_s) + 1;
  LOG_INFO << "load_length: " << load_length << ", av_kappa: " << av_kappa
           << ", path points: " << n
           << ", front_wheel_angle: " << front_wheel_angle;
  std::vector<double> s_vec, ref_center_l, right_l, left_l, target_right_l,
      target_left_l;
  s_vec.reserve(n);
  ref_center_l.reserve(n);
  right_l.reserve(n);
  left_l.reserve(n);
  target_right_l.reserve(n);
  target_left_l.reserve(n);
  std::vector<Vec2d> inner_right_xy, inner_left_xy, outer_right_xy,
      outer_left_xy, ref_center_xy;
  inner_right_xy.reserve(n);
  inner_left_xy.reserve(n);
  outer_right_xy.reserve(n);
  outer_left_xy.reserve(n);
  ref_center_xy.reserve(n);
  UniCycleModelState cur_state{
      .x = plan_start_point.path_point().x(),
      .y = plan_start_point.path_point().y(),
      .heading = plan_start_point.path_point().theta(),
      .kappa = av_kappa,
  };

  const double init_heading = cur_state.heading;
  const double front_edge_to_center =
      vehicle_geom_params.front_edge_to_center();
  const double av_half_width = 0.5 * vehicle_geom_params.width();
  const PiecewiseLinearFunction<double, double>
      near_av_inner_boundary_half_width_plf = {
          {0.0, front_edge_to_center}, {av_half_width, av_half_width + 1.0}};
  const PiecewiseLinearFunction<double, double>
      near_av_outer_boundary_half_width_plf = {
          {0.0, front_edge_to_center}, {av_half_width, av_half_width + 1.5}};
  for (int i = 0; i < n; ++i) {
    const double cur_s = i * corridor_step_s;
    const auto ratio = cur_s / load_length;
    double inner_boundary_half_width = kInnerBoundaryHalfWidthPlf(ratio);
    double outer_boundary_half_width = kOuterBoundaryHalfWidthPlf(ratio);
    if (cur_s < kExpandedBoundaryMinS) {
      inner_boundary_half_width = near_av_inner_boundary_half_width_plf(cur_s);
      outer_boundary_half_width = near_av_outer_boundary_half_width_plf(cur_s);
    } else if (cur_s > kExpandedBoundaryMinS && cur_s < kExpandedBoundaryMaxS &&
               std::fabs(front_wheel_angle) >
                   kExpandedBoundaryHalfWidthPlf.x().front()) {
      inner_boundary_half_width =
          kExpandedBoundaryHalfWidthPlf(front_wheel_angle);
      outer_boundary_half_width =
          kExpandedBoundaryHalfWidthPlf(front_wheel_angle);
    }
    auto min_max_pair =
        std::minmax(inner_boundary_half_width, outer_boundary_half_width);
    inner_boundary_half_width = min_max_pair.first;
    outer_boundary_half_width = min_max_pair.second;
    // May need +/- a slight boundary by kappa.
    s_vec.push_back(cur_s);
    ref_center_l.push_back(0.0);
    right_l.push_back(-outer_boundary_half_width);
    left_l.push_back(outer_boundary_half_width);
    target_right_l.push_back(-inner_boundary_half_width);
    target_left_l.push_back(inner_boundary_half_width);

    Vec2d pos(cur_state.x, cur_state.y);
    const auto tangent = Vec2d::FastUnitFromAngle(cur_state.heading);
    inner_left_xy.emplace_back(
        LatPoint(pos, tangent, inner_boundary_half_width));
    inner_right_xy.emplace_back(
        LatPoint(pos, tangent, -inner_boundary_half_width));
    outer_left_xy.emplace_back(
        LatPoint(pos, tangent, outer_boundary_half_width));
    outer_right_xy.emplace_back(
        LatPoint(pos, tangent, -outer_boundary_half_width));
    ref_center_xy.push_back(std::move(pos));
    // Fix kappa, a circle.
    cur_state = GenUniCycleModelState(cur_state, corridor_step_s,
                                      /*kappa_decay=*/1.0, init_heading,
                                      /*max_delta_heading=*/M_PI_2);
  }

  // Extent path points to extended length.
  const double extend_length =
      std::clamp(std::fabs(GetLonSpeed(pose)) *
                     (total_preview_time + kCorridorExtendLengthTimeHorizon),
                 kMinPathLength, kMaxPathLength);
  const auto heading2d = Vec2d::UnitFromAngle(cur_state.heading);
  auto path = ExtendedCubicSplinePath(ref_center_xy, s_vec, heading2d,
                                      corridor_step_s, kPathSampleInterval,
                                      extend_length, kMaxAllowedAccKappa);
  auto frenet_frame_or = BuildQtfmEnhancedKdTreeFrenetFrame(
      ref_center_xy, /*down_sample_raw_points=*/true);
  if (!frenet_frame_or.ok()) {
    return BuildErrorAccPathCorridorResult(
        "Build frenet frame failed without map.");
  }

  auto opt_right_xy = outer_right_xy;
  auto opt_left_xy = outer_left_xy;
  auto opt_right_l = right_l;
  auto opt_left_l = left_l;

  auto path_boundary = PathSlBoundary(
      std::move(s_vec), std::move(ref_center_l), std::move(right_l),
      std::move(left_l), std::move(opt_right_l), std::move(opt_left_l),
      std::move(target_right_l), std::move(target_left_l),
      std::move(ref_center_xy), std::move(outer_right_xy),
      std::move(outer_left_xy), std::move(opt_right_xy), std::move(opt_left_xy),
      std::move(inner_right_xy), std::move(inner_left_xy));
  PiecewiseLinearFunction<double> kappa_s(
      std::vector<double>{0.0, path.length()},
      std::vector<double>{av_kappa, av_kappa});
  return AccPathCorridor{
      .build_status = OkPlannerStatus(),
      .type = AccPathCorridorType::NO_MAP,
      .boundary = std::move(path_boundary),
      .frenet_frame = std::make_unique<QtfmEnhancedKdTreeFrenetFrame>(
          std::move(frenet_frame_or).value()),
      .path = std::move(path),
      .loaded_map_dist = load_length,
      .kappa_s = std::move(kappa_s)};
}

}  // namespace st::planning
