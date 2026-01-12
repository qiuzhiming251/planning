
#ifndef AD_BYD_PLANNING_MATH_PIECEWISE_CURVE_H
#define AD_BYD_PLANNING_MATH_PIECEWISE_CURVE_H
#include <vector>

namespace ad_byd {
namespace planning {
namespace math {

class PiecewiseCurve {
 public:
  PiecewiseCurve(const std::vector<double>& pose_points,
                 const std::vector<double>& time_piece, const double& start_v,
                 const double& end_v);
  bool EvaluateCurve(const int& order, const double& time,
                     double* const value) const;

 private:
  void GetCurve(const std::vector<double>& pose_points,
                const std::vector<double>& time_piece, const double& start_v,
                const double& end_v);
  void InitializeEquations(const std::vector<double>& pose_points,
                           const std::vector<double>& time_piece,
                           const double& start_v, const double& end_v);
  void SolveDiaTriangle(const std::vector<double>& pose_points);
  void GetCurveCoefs(const std::vector<double>& pose_points,
                     const std::vector<double>& time_piece);
  int GetPieceForTime(const double& time) const;

 private:
  int knot_size_ = 0;
  // piecewise curve param
  std::vector<double> ai_;
  std::vector<double> bi_;
  std::vector<double> ci_;
  std::vector<double> di_;

  // equations
  std::vector<double> time_sequence_;
  std::vector<double> array_a_;
  std::vector<double> array_b_;
  std::vector<double> array_c_;
  std::vector<double> array_d_;
  std::vector<double> array_m_;
};
}  // namespace math
}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MATH_PIECEWISE_CURVE_H