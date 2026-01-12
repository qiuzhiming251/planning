

#include <algorithm>
#include <numeric>
#include <ostream>
#include <string>

#include "Eigen/Jacobi"
#include "Eigen/SVD"
#include "plan_common/log.h"
#include "plan_common/math/line_fitter.h"
#include "plan_common/math/stats.h"
#include "plan_common/math/util.h"
//#include "plan_common/util/string_util.h"

namespace st {

void LineFitter::CheckData() const {
  const int nd = data_.size();
  const int nw = weights_.size();
  CHECK(nw == 0 || nw == nd);
  for (const double w : weights_) {
    CHECK_GE(w, 0.0);
  }

  // debug
  // if (debug_) {
  //   VLOG(0) << "******** line fitter data ********";
  // VLOG(0) << " line fitter data: " << VecOfVec2dToString(data_);
  // VLOG(0) << " line fitter data weight: "
  //         << VecOfRealNumbersToString(weights_);
  //   VLOG(0) << "******** line fitter data ********";
  // }
}

void LineFitter::FitData(FITTER fitter, bool normalize, bool compute_mse) {
  // Compute the statistics
  Vec2d sample_mean;
  const auto sample_cov =
      Weighted2DSampleMeanAndCovarianceMatrix(data_, &sample_mean, weights_);

  // debug
  if (debug_) {
    VLOG(2) << "******** line fitter stats ********";
    VLOG(2) << " line fitter data mean: " << sample_mean.transpose();
    VLOG(2) << " line fitter data cov: " << sample_cov;
    VLOG(2) << "******** line fitter stats ********";
  }

  if (fitter == FITTER::DEMING) {
    const Eigen::JacobiSVD<Eigen::Matrix2d> svd(
        sample_cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // line normal is the eigen vector of the smallest eigenvalue.
    const auto line_normal = svd.matrixU().block<2, 1>(0, 1);
    const double bias = sample_mean.dot(line_normal);
    param_ = Vec3d(line_normal(0), line_normal(1), bias);
  } else {
    CHECK(false);
  }

  if (normalize) {
    param_ /= param_.head(2).norm();
    if (debug_) {
      VLOG(3) << "  normalized param = " << param_.transpose();
    }
  }

  if (compute_mse) {
    mse_ = std::accumulate(
        data_.begin(), data_.end(), 0.0,
        [this](const double& sum, const Vec2d& sample) {
          return sum +
                 Sqr(param_(0) * sample(0) + param_(1) * sample(1) + param_(2));
        });
  }
}

}  // namespace st
