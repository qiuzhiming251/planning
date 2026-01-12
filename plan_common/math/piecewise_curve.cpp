
#include <cmath>

#include "plan_common/math/double.h"
#include "plan_common/math/piecewise_curve.h"

namespace ad_byd {
namespace planning {
namespace math {

PiecewiseCurve::PiecewiseCurve(const std::vector<double>& pose_point,
                               const std::vector<double>& time_piece,
                               const double& start_v, const double& end_v) {
  knot_size_ = static_cast<int>(pose_point.size());
  time_sequence_.clear();
  time_sequence_.emplace_back(0.0);
  for (auto& time : time_piece) {
    time_sequence_.emplace_back(time_sequence_.back() + time);
  }
  if (knot_size_ >= 2) {
    // initialize curve parameters size
    ai_.resize(knot_size_ - 1);
    bi_.resize(knot_size_ - 1);
    ci_.resize(knot_size_ - 1);
    di_.resize(knot_size_ - 1);
    // initialize equation matrix size
    array_a_.resize(knot_size_);
    array_b_.resize(knot_size_);
    array_c_.resize(knot_size_);
    array_d_.resize(knot_size_);
    array_m_.resize(knot_size_);
    GetCurve(pose_point, time_piece, start_v, end_v);
  }
}

void PiecewiseCurve::GetCurve(const std::vector<double>& pose_point,
                              const std::vector<double>& time_piece,
                              const double& start_v, const double& end_v) {
  InitializeEquations(pose_point, time_piece, start_v, end_v);
  SolveDiaTriangle(pose_point);
  GetCurveCoefs(pose_point, time_piece);
}

void PiecewiseCurve::InitializeEquations(const std::vector<double>& pose_points,
                                         const std::vector<double>& time_piece,
                                         const double& start_v,
                                         const double& end_v) {
  // initialize left equation matrix
  /*
     [B0, C0, 0, .....]
     [A1, B1, C1, 0, ....]
     [0, A2, B2, C2, 0, ...]
     ...
     [0, ..., An-2, Bn-2, Cn-2]
     [0, ...., An-2, Bn-1]
  */
  // ignore A0
  array_a_[0] = 0;
  array_a_[knot_size_ - 1] = time_piece[knot_size_ - 2];
  // B
  array_b_[0] = 2 * time_piece[0];
  array_b_[knot_size_ - 1] = 2 * time_piece[knot_size_ - 2];
  // ignore Cn-1
  array_c_[knot_size_ - 1] = 0;
  array_c_[0] = time_piece[0];

  // other elements
  for (int i = 1; i < knot_size_ - 1; ++i) {
    // A1- An-1 effective
    array_a_[i] = time_piece[i - 1];
    // B0 - Bn - 1 is effective
    array_b_[i] = 2 * (time_piece[i - 1] + time_piece[i]);
    // C0 - Cn-2 is effective
    array_c_[i] = time_piece[i];
  }

  // add end points constrain using start & end v
  array_d_[0] =
      6.0 * ((pose_points[1] - pose_points[0]) / time_piece[0] - start_v);
  array_d_[knot_size_ - 1] =
      6.0 *
      (end_v - (pose_points[knot_size_ - 1] - pose_points[knot_size_ - 2]) /
                   time_piece[knot_size_ - 2]);

  // initial right equation matrix
  for (int i = 1; i < knot_size_ - 1; ++i) {
    array_d_[i] =
        6.0 * ((pose_points[i + 1] - pose_points[i]) / time_piece[i] -
               (pose_points[i] - pose_points[i - 1]) / time_piece[i - 1]);
  }
}

void PiecewiseCurve::SolveDiaTriangle(const std::vector<double>& pose_points) {
  // caculate dia matrix (dimension, A, B, C, D)
  // M = [None] * dimension
  // from dia upper triangular matrix
  array_c_[0] /= array_b_[0];
  array_d_[0] /= array_b_[0];
  for (int i = 1; i < knot_size_; ++i) {
    double tmp = array_b_[i] - array_a_[i] * array_c_[i - 1];
    if (std::fabs(tmp) < std::numeric_limits<double>::epsilon()) {
      return;
    }
    array_c_[i] /= tmp;
    array_d_[i] = (array_d_[i] - array_a_[i] * array_d_[i - 1]) / tmp;
  }
  // get last M directely
  array_m_[knot_size_ - 1] = array_d_[knot_size_ - 1];
  for (int i = 0; i < knot_size_ - 1; ++i) {
    int j = knot_size_ - 2 - i;
    array_m_[j] = array_d_[j] - array_c_[j] * array_m_[j + 1];
  }
}

void PiecewiseCurve::GetCurveCoefs(const std::vector<double>& pose_points,
                                   const std::vector<double>& time_piece) {
  for (int i = 0; i < knot_size_ - 1; ++i) {
    ai_[i] = pose_points[i];
    bi_[i] =
        (pose_points[i + 1] - pose_points[i]) / time_piece[i] -
        (2.0 * time_piece[i] * array_m_[i] + time_piece[i] * array_m_[i + 1]) /
            6.0;
    ci_[i] = array_m_[i] / 2.0;
    di_[i] = (array_m_[i + 1] - array_m_[i]) / (6.0 * time_piece[i]);
  }
}

int PiecewiseCurve::GetPieceForTime(const double& time) const {
  for (int i = 0; i < static_cast<int>(time_sequence_.size()) - 1; ++i) {
    if (Double::Compare(time, time_sequence_[i]) != Double::CompareType::LESS &&
        Double::Compare(time, time_sequence_[i + 1]) !=
            Double::CompareType::GREATER) {
      return i;
    }
    // if (Double::Compare(time, time_sequence_[i]) * Double::Compare(time,
    // time_sequence_[i + 1]) != 1) return i;
  }
  return -1;
}

bool PiecewiseCurve::EvaluateCurve(const int& order, const double& time,
                                   double* const value) const {
  if (ai_.empty() || bi_.empty() || ci_.empty() || di_.empty() ||
      array_a_.empty() || array_b_.empty() || array_c_.empty() ||
      array_d_.empty() || array_m_.empty()) {
    return false;
  }
  int piece = GetPieceForTime(time);
  if (piece < 0) {
    return false;
  }
  double d_t = time - time_sequence_[piece];
  switch (order) {
    case 0: {
      *value = ai_[piece] +
               d_t * (bi_[piece] + d_t * (ci_[piece] + di_[piece] * d_t));
      break;
    }
    case 1: {
      *value = bi_[piece] + d_t * (2 * ci_[piece] + 3 * di_[piece] * d_t);
      break;
    }
    case 2: {
      *value = 2 * ci_[piece] + 6 * di_[piece] * d_t;
      break;
    }
    default:
      return false;
  }
  return true;
}

}  // namespace math
}  // namespace planning
}  // namespace ad_byd
