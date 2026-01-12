

#include <cmath>
#include <numeric>
#include <string>

#include "Eigen/Cholesky"
#include "Eigen/QR"
#include "Eigen/SVD"
#include "absl/strings/str_cat.h"
#include "plan_common/math/circle_fitter.h"
//#include "plan_common/util/status_macros.h"

namespace st {

namespace {

absl::Status CheckData(const std::vector<Vec2d>& data,
                       const std::vector<double>& weights) {
  const int nd = data.size();
  if (nd < 3) {
    return absl::InvalidArgumentError(
        absl::StrCat("Inadequate data points (", nd, ")!"));
  }

  const int nw = weights.size();
  if (nw != 0 && nw != nd) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Weight size (", nw, ") does not match data size (", nd, ")!"));
  }
  for (const double w : weights) {
    if (w < 0.0) {
      return absl::InvalidArgumentError(
          absl::StrCat("Negative weight found: ", w, "!"));
    }
  }
  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<Circle> FitCircleToData(const std::vector<Vec2d>& data,
                                       const std::vector<double>& weights,
                                       LS_SOLVER solver, double* mse) {
  // RETURN_IF_ERROR(CheckData(data, weights));

  const size_t num_data = data.size();
  const Vec2d mean_pos =
      std::accumulate(
          data.begin(), data.end(), Vec2d(0.0, 0.0),
          [](const Vec2d& sum, const Vec2d& sample) { return sum + sample; }) /
      data.size();
  Eigen::Matrix3d lhs = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < num_data; ++i) {
    const double x = data[i].x() - mean_pos.x();
    const double x_2 = Sqr(x);
    const double x_3 = x * x_2;
    const double y = data[i].y() - mean_pos.y();
    const double y_2 = Sqr(y);
    const double y_3 = y * y_2;

    const double weight =
        weights.size() == num_data ? std::sqrt(weights[i]) : 1.0;
    const double coeff = 2.0 * weight;

    lhs(0, 0) += coeff * x_2;
    lhs(0, 1) += coeff * x * y;
    lhs(0, 2) += coeff * x;
    rhs(0, 0) -= coeff * (x_3 + x * y_2);

    lhs(1, 0) += coeff * x * y;
    lhs(1, 1) += coeff * y_2;
    lhs(1, 2) += coeff * y;
    rhs(1, 0) -= coeff * (y_3 + x_2 * y);

    lhs(2, 0) += coeff * x;
    lhs(2, 1) += coeff * y;
    lhs(2, 2) += coeff;
    rhs(2, 0) -= coeff * (y_2 + x_2);
  }

  Eigen::Vector3d sol;
  switch (solver) {
    case LS_SOLVER::SVD:
      sol = lhs.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(rhs);
      break;
    case LS_SOLVER::QR:
      sol = lhs.colPivHouseholderQr().solve(rhs);
      break;
    case LS_SOLVER::PINV:
      sol = (lhs.transpose() * lhs).ldlt().solve(lhs.transpose() * rhs);
  }

  const double center_x = sol[0] / -2.0;
  const double center_y = sol[1] / -2.0;
  const double radius =
      std::sqrt(std::abs(Sqr(center_x) + Sqr(center_y) - sol[2]));
  Circle res(Vec2d(center_x + mean_pos.x(), center_y + mean_pos.y()), radius);

  if (mse != nullptr) {
    *mse = std::accumulate(data.begin(), data.end(), 0.0,
                           [&res](double sum, const Vec2d& sample) {
                             return sum + Sqr(res.DistanceTo(sample));
                           }) /
           data.size();
  }

  return res;
}

}  // namespace st
