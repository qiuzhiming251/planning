

#ifndef ST_PLANNING_MATH_STATS
#define ST_PLANNING_MATH_STATS

#include <limits>
#include <numeric>
#include <vector>

#include "Eigen/Dense"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"

namespace st {

template <typename T>
T Sum(const std::vector<T>& samples) {
  return std::accumulate(samples.begin(), samples.end(), T());
}

// Caller is responsible for ensuring samples is not empty.
template <typename T>
T Mean(const std::vector<T>& samples) {
  return Sum(samples) / samples.size();
}

// Caller is responsible for ensuring samples and weights are not empty, and
// are of the same size and legal value (e.g., weight being positive).
template <typename T, typename WeightType>
T WeightedMean(
    const std::vector<T>& samples,
    const std::vector<WeightType>& weights = std::vector<WeightType>()) {
  CHECK(weights.size() == 0 || weights.size() == samples.size());
  T stat = T();
  for (int i = 0; i < samples.size(); ++i) {
    stat += samples[i] * (weights.size() > 0 ? weights[i] : 1.0);
  }
  return stat / (weights.size() > 0 ? Sum(weights) : samples.size());
}

// Caller is responsible for ensuring samples is not empty.
template <typename T>
T SampleVariance(const std::vector<T>& samples) {
  if (samples.size() == 1) return 0.0;
  const int n = samples.size();
  const T sum_squares = std::accumulate(
      samples.begin(), samples.end(), 0.0,
      [](const T& sum, const T& value) { return sum + Sqr(value); });
  return (sum_squares - Sqr(Sum(samples)) / n) / (n - 1);
}

// Caller is responsible for ensuring samples is not empty.
template <typename T>
T WeightedSampleVariance(const std::vector<T>& samples,
                         const std::vector<T>& weights) {
  if (samples.size() == 1) return 0.0;
  const int n = samples.size();
  CHECK(weights.size() == n);
  const double weighted_mean = WeightedMean(samples, weights);
  T stat = T();
  for (int i = 0; i < samples.size(); ++i) {
    stat += Sqr(samples[i] - weighted_mean) * weights[i];
  }
  return n / (n - 1) / Sum(weights) * stat;
}

// Caller is responsible for ensuring samples is not empty.
template <typename T, typename W>
Eigen::Matrix<T, 2, 2> Weighted2DSampleMeanAndCovarianceMatrix(
    const std::vector<Vec2<T>>& samples, Vec2<T>* sample_mean,
    const std::vector<W>& sample_weights = std::vector<W>{}) {
  using Mat2T = Eigen::Matrix<T, 2, 2>;
  const bool has_weight = !sample_weights.empty();
  CHECK(sample_mean != nullptr);
  CHECK(!has_weight || samples.size() == sample_weights.size());
  *sample_mean =
      has_weight ? WeightedMean(samples, sample_weights) : Mean(samples);
  if (samples.size() == 1) return Mat2T::Zero();
  const W weight_sum = has_weight ? Sum(sample_weights) : samples.size();
  Mat2T stat = Mat2T::Zero();
  for (int i = 0; i < samples.size(); ++i) {
    stat += (has_weight ? sample_weights[i] : 1.0) *
            (samples[i] - *sample_mean) *
            (samples[i] - *sample_mean).transpose();
  }
  return stat / ((1.0 - 1.0 / samples.size()) * weight_sum);
}

// Caller is responsible for ensuring samples is not empty.
template <typename T>
T PopulationVariance(const std::vector<T>& samples) {
  const int n = samples.size();
  const T sum_squares = std::accumulate(
      samples.begin(), samples.end(), 0.0,
      [](const T& sum, const T& value) { return sum + Sqr(value); });
  return (sum_squares - Sqr(Sum(samples)) / n) / n;
}

template <typename T>
T SampleStdDev(const std::vector<T>& samples) {
  return std::sqrt(SampleVariance(samples));
}

template <typename T>
T PopulationStdDev(const std::vector<T>& samples) {
  return std::sqrt(PopulationVariance(samples));
}

// Caller is responsible for ensuring samples is not empty.
template <typename T>
T SampleMeanStdError(const std::vector<T>& samples) {
  return std::sqrt(SampleVariance(samples) / samples.size());
}

}  // namespace st

#endif  // ST_PLANNING_MATH_STATS
