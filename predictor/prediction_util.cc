#include "predictor/prediction_util.h"

#include <algorithm>
#include <vector>

#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/util/path_util.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction.h"
#include "predictor/prediction_defs.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/prediction.pb.h"
#include "modules/cnoa_pnc/planning/proto/prediction_common.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st {
namespace prediction {

void InitTraj(PredictedTrajectory* const trajectory, const double curr_acc,
              std::vector<double>* const split_s) {
  const auto& pts = trajectory->points();
  split_s->emplace_back(0.0);
  double s = 0.0;

  for (size_t i = 0; i < pts.size(); i++) {
    // traj_points_.emplace_back(std::make_pair(states[i].position.x(),
    // states[i].position.y())); traj_heading_.emplace_back(states[i].heading);
    if (i < pts.size() - 1) {
      double dis = (pts.at(i + 1).pos() - pts.at(i).pos()).Length();
      s = s + dis;
      split_s->emplace_back(s);
    }
  }
}

bool RefineTrajByAcc(PredictedTrajectory* const trajectory,
                     const double curr_acc, const double acc_ts_sec) {
  if (trajectory == nullptr) return false;
  const auto& pts = trajectory->points();
  if (trajectory->points().size() < 2) return false;
  const double cv_ts_sec = std::max(0.0, kPredictionDuration - acc_ts_sec);
  // 以CA沿s方向采样0-2s轨迹点，然后以CV采样2-10s轨迹
  PredictedTrajectory refined_traj;
  refined_traj.set_probability(trajectory->probability());
  refined_traj.set_index(trajectory->index());
  refined_traj.set_type(trajectory->type());
  refined_traj.mutable_points()->emplace_back(std::move(pts.at(0)));
  std::vector<double> split_s;
  constexpr double t = kPredictionTimeStep;
  InitTraj(trajectory, curr_acc, &split_s);
  // Vec2d prev_pos = pts.at(0).pos();
  double prev_s = 0.0;
  double prev_theta = pts.at(0).theta();
  double prev_kappa = pts.at(0).kappa();
  double prev_t = 0.0;
  double prev_v = pts.at(0).v();
  double prev_a = curr_acc;
  double ds = 0.0;
  const int acc_point_num =
      static_cast<int>(acc_ts_sec / kPredictionTimeStep) - 1;
  DLOG(INFO) << "refine init state " << prev_s << " " << prev_theta << " "
             << prev_kappa << " " << prev_t << " " << prev_v << " " << prev_a;
  size_t si = 1;
  for (size_t i = 1; i < kPredictionPointNum; ++i) {
    prev_t = i * t;
    double rest_s = split_s.back() - prev_s;
    double tmp_a = prev_a;
    if (i < acc_point_num + 1) {
      tmp_a = std::min(
          prev_a,
          std::max(
              0.0,
              (rest_s - prev_v * (acc_ts_sec - prev_t) - prev_v * cv_ts_sec) /
                  (0.5 * (acc_ts_sec - prev_t) * (acc_ts_sec - prev_t) +
                   (acc_ts_sec - prev_t))));
      if (prev_v + prev_a * t < 0.0) {
        tmp_a = -prev_v / t;
      }
      ds = prev_v * t + 0.5 * tmp_a * t * t;
    } else {
      tmp_a = 0.0;
      ds = prev_v * t;
    }
    if (prev_s + ds > split_s.at(si)) {
      // DLOG(INFO) << "== skip split s: " << prev_s + ds << " > "
      //            << split_s.at(si) << ". " << si << " i " << i;
      si = std::min(split_s.size() - 1, si + 1);
    }
    prev_a = tmp_a;
    prev_v += prev_a * t;
    prev_s += ds;
    // DLOG(INFO) << " prev s: " << prev_s << "/" << split_s.back();
    // get point from new_s
    PredictedTrajectoryPoint refined_pt;
    Vec2d new_pos = (pts.at(si).pos() - pts.at(si - 1).pos()) *
                        (prev_s - split_s.at(si - 1)) /
                        std::max(0.1, split_s.at(si) - split_s.at(si - 1)) +
                    pts.at(si - 1).pos();

    // DLOG(INFO) << "i " << i << " ds " << ds << " v " << prev_v << " a "
    //            << prev_a << " pos " << new_pos.x() << ", " << new_pos.y();
    refined_pt.set_pos(new_pos);
    refined_pt.set_s(prev_s);
    refined_pt.set_theta(pts.at(si - 1).theta());
    refined_pt.set_kappa(pts.at(si - 1).kappa());
    refined_pt.set_t(pts.at(0).t() + prev_t);
    refined_pt.set_v(prev_v);
    refined_pt.set_a(prev_a);
    refined_traj.mutable_points()->emplace_back(std::move(refined_pt));
  }
  *trajectory = refined_traj;
  return true;
}

planning::DiscretizedPath PredictedTrajectoryPointsToPathPoints(
    const std::vector<prediction::PredictedTrajectoryPoint>& points) {
  planning::DiscretizedPath discretized_path;
  discretized_path.reserve(points.size());
  for (const auto& pt : points) {
    PathPoint path_pt;
    path_pt.set_x(pt.pos().x());
    path_pt.set_y(pt.pos().y());
    path_pt.set_s(pt.s());
    path_pt.set_theta(pt.theta());
    path_pt.set_kappa(pt.kappa());
    discretized_path.push_back(std::move(path_pt));
  }
  return discretized_path;
}

planning::SpeedVector GenerateSpeedProfileByConstAccel(double v, double a,
                                                       int time_horizon,
                                                       double time_step,
                                                       double a_duration) {
  CHECK_GT(time_step, 0.0);
  planning::SpeedVector speed_profile;
  speed_profile.reserve(time_horizon);
  double curr_t = 0.0;
  speed_profile.emplace_back(curr_t, /*s=*/0.0, v, a, /*j=*/0.0);
  for (int i = 0; i < time_horizon; ++i) {
    curr_t += time_step;
    const auto& prev_speed_pt = speed_profile.back();
    double curr_a = 0.0;
    if (curr_t < a_duration) curr_a = a;
    const double curr_v = std::max(prev_speed_pt.v() + curr_a * time_step, 0.0);
    const double curr_s =
        prev_speed_pt.s() + 0.5 * (curr_v + prev_speed_pt.v()) * time_step;
    curr_a = (curr_v - prev_speed_pt.v()) / time_step;
    speed_profile.emplace_back(curr_t, curr_s, curr_v, curr_a, /*j=*/0.0);
  }
  return speed_profile;
}

absl::Status CombinePathAndSpeed(
    const planning::DiscretizedPath& path_data,
    const planning::SpeedVector& speed_data, double time_step,
    std::vector<PredictedTrajectoryPoint>* trajectory) {
  CHECK_NOTNULL(trajectory);
  if (path_data.size() < 2 || speed_data.size() < 2) {
    return absl::InternalError("path_data or speed_data size less than 2.");
  }
  trajectory->clear();
  double t = 0.0;
  while (t < speed_data.TotalTime()) {
    const auto speed_point = speed_data.EvaluateByTime(t);
    if (!speed_point.has_value()) {
      return absl::InternalError(
          absl::StrFormat("Fail to evaluate speed vector at time %.2f", t));
    }
    const PathPoint path_point = path_data.length() < kEpsilon
                                     ? path_data.front()
                                     : path_data.Evaluate(speed_point->s());
    PredictedTrajectoryPoint traj_point;
    traj_point.set_t(t);
    traj_point.set_s(path_point.s());
    traj_point.set_pos(planning::ToVec2d(path_point));
    traj_point.set_theta(path_point.theta());
    traj_point.set_kappa(path_point.kappa());
    traj_point.set_v(speed_point->v());
    traj_point.set_a(speed_point->a());
    trajectory->push_back(std::move(traj_point));
    t += time_step;
  }
  return absl::OkStatus();
}

std::vector<PredictedTrajectoryPoint>
ApolloTrajectoryPointsToPredictionTrajectoryPoints(
    const std::vector<ApolloTrajectoryPointProto>& points) {
  std::vector<PredictedTrajectoryPoint> pred_traj_points;
  pred_traj_points.reserve(points.size());
  for (const auto& traj_pt : points) {
    PredictedTrajectoryPoint pred_traj_pt;
    const auto& path_pt = traj_pt.path_point();
    pred_traj_pt.set_pos(planning::ToVec2d(path_pt));
    pred_traj_pt.set_t(traj_pt.relative_time());
    pred_traj_pt.set_s(path_pt.s());
    pred_traj_pt.set_theta(path_pt.theta());
    pred_traj_pt.set_kappa(path_pt.kappa());
    pred_traj_pt.set_v(traj_pt.v());
    pred_traj_pt.set_a(traj_pt.a());
    pred_traj_points.push_back(std::move(pred_traj_pt));
  }
  return pred_traj_points;
}

}  // namespace prediction
}  // namespace st
