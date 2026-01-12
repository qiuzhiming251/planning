

#ifndef ONBOARD_MATH_UTIL_H_
#define ONBOARD_MATH_UTIL_H_

#ifdef __SSE4_1__
#include <immintrin.h>
#include <smmintrin.h>  // for _MM_FROUND_NO_EXC, _MM_EXTRACT_FLOAT, _mm_round_sd, _mm_round_ss
#include <xmmintrin.h>  // for _mm_load_ss, _mm_undefined_ps
#ifndef __AVX512F__
#include <emmintrin.h>
#endif  // __AVX512F__
#endif  // __SSE4_1__

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "plan_common/base/macros.h"
#include "plan_common/log.h"

namespace st {

namespace math_util_internal {

#ifdef __SSE4_1__

template <int kRoundingMode>
inline int RoundHelper(float val) {
#ifdef __AVX512F__
  return _mm_cvt_roundss_i32(_mm_load_ss(&val), kRoundingMode);
#else
  const auto t =
      _mm_round_ss(_mm_undefined_ps(), _mm_load_ss(&val), kRoundingMode);
  float rounded;
  _MM_EXTRACT_FLOAT(rounded, t, 0);
  return static_cast<int>(rounded);
#endif  // __AVX512F__
}

template <int kRoundingMode>
inline int RoundHelper(double val) {
#ifdef __AVX512F__
  return _mm_cvt_roundsd_i32(_mm_load_ss(&val), kRoundingMode);
#else
  const auto rounded = _mm_cvtsd_f64(
      _mm_round_sd(_mm_undefined_pd(), _mm_load_sd(&val), kRoundingMode));
  return static_cast<int>(rounded);
#endif  // __AVX512F__
}

#endif  // __SSE4_1__

}  // namespace math_util_internal

template <typename T>
inline int FloorToInt(T val) {
#ifdef __SSE4_1__
  return math_util_internal::RoundHelper<_MM_FROUND_TO_NEG_INF |
                                         _MM_FROUND_NO_EXC>(val);
#else
  return static_cast<int>(std::floor(val));
#endif
}

template <typename T>
inline int CeilToInt(T val) {
#ifdef __SSE4_1__
  return math_util_internal::RoundHelper<_MM_FROUND_TO_POS_INF |
                                         _MM_FROUND_NO_EXC>(val);
#else
  return static_cast<int>(std::ceil(val));
#endif
}

// Note that RoundToInt rounds half to even.
// E.g., it rounds 1.5 to 2, and 2.5 to 2.
template <typename T>
inline int RoundToInt(T val) {
#ifdef __SSE4_1__
  return math_util_internal::RoundHelper<_MM_FROUND_TO_NEAREST_INT |
                                         _MM_FROUND_NO_EXC>(val);
#else
  return static_cast<int>(std::round(val));
#endif
}

template <typename T>
inline int TruncateToInt(T val) {
#ifdef __SSE4_1__
  return math_util_internal::RoundHelper<_MM_FROUND_TO_ZERO |
                                         _MM_FROUND_NO_EXC>(val);
#else
  return static_cast<int>(val);
#endif
}

template <typename T>
inline T Sign(T val) {
  if (val > 0) return 1;
  if (val < 0) return -1;
  return 0;
}

template <typename T>
inline bool IsInt(T val) {
  return val == TruncateToInt(val);
}

template <typename T>
bool InRange(T val, T bound0, T bound1) {
  if (bound0 > bound1) std::swap(bound0, bound1);
  return val >= bound0 && val <= bound1;
}

template <typename T>
constexpr T d2r(T d) {
  static_assert(std::is_floating_point<T>::value);
  return d * T(M_PI / 180.);
}
template <typename T>
constexpr T r2d(T r) {
  static_assert(std::is_floating_point<T>::value);
  return r * T(180. / M_PI);
}

inline constexpr double Mph2Mps(double mph) { return mph * (1609.34 / 3600.0); }
inline constexpr double Mps2Mph(double mps) { return mps * (3600.0 / 1609.34); }

inline constexpr double Kph2Mps(double kph) { return kph * (1000.0 / 3600.0); }
inline constexpr double Mps2Kph(double mps) { return mps * 3.6; }
inline constexpr double kMathEpsilon = 1e-10;
inline constexpr double CentiMeterToMeter(double centimeter) {
  return centimeter * 1e-2;
}
inline constexpr double MeterToCentiMeter(double meter) { return meter * 1e2; }

template <typename T>
constexpr T Sqr(T val) {
  return val * val;
}

template <typename T>
constexpr T SingleSideSqr(T val) {
  return (val > 0.0) ? (val * val) : 0.0;
}

// https://en.wikipedia.org/wiki/Rectifier_(neural_networks)
template <typename T>
constexpr T ReLU(T val) {
  return (val > 0.0) ? val : 0.0;
}

// https://en.wikipedia.org/wiki/Heaviside_step_function
template <typename T>
constexpr T UnitStep(T val) {
  return (val > 0.0) ? 1.0 : 0.0;
}

template <typename T>
constexpr T Cube(T val) {
  return val * Sqr(val);
}

template <typename T>
constexpr T Quar(T val) {
  return val * Cube(val);
}

template <typename T>
constexpr T Quin(T val) {
  return val * Quar(val);
}

template <typename T, typename U>
constexpr std::enable_if_t<std::is_integral_v<U>, T> Power(T val, U exp) {
  CHECK(exp >= 0);
  T res = 1;
  while (exp > 0) {
    if (exp & 1) res *= val;
    exp >>= 1;
    val *= val;
  }
  return res;
}

template <typename T>
T Hypot(T a, T b) {
  return std::sqrt(Sqr(a) + Sqr(b));
}

template <typename T>
T Hypot(T a, T b, T c) {
  return std::sqrt(Sqr(a) + Sqr(b) + Sqr(c));
}

template <typename T, typename TS>
constexpr T Lerp(T a, T b, TS s) {
  return a + (b - a) * s;
}

/**
 * @brief Linear interpolation between two points of type T.
 * @param x0 The coordinate of the first point.
 * @param t0 The interpolation parameter of the first point.
 * @param x1 The coordinate of the second point.
 * @param t1 The interpolation parameter of the second point.
 * @param t The interpolation parameter for interpolation.
 * @param x The coordinate of the interpolated point.
 * @return Interpolated point.
 */
template <typename T>
T Lerp(const T& x0, const double t0, const T& x1, const double t1,
       const double t, bool clamp = false) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    return x0;
  }
  if (clamp) {
    if (t0 <= t1) {
      if (t <= t0) {
        return x0;
      }
      if (t >= t1) {
        return x1;
      }
    } else {
      if (t >= t0) {
        return x0;
      }
      if (t <= t1) {
        return x1;
      }
    }
  }
  const double r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x;
}

template <typename T>
constexpr T LerpFactor(T a, T b, T value) {
  if (b == a) return 0.0;
  return (value - a) / (b - a);
}

template <typename T>
constexpr T WrapIfNegative(T val, T range) {
  return val < 0.0 ? val + range : val;
}

// Wrap val to range [min_val, max_val).
template <typename T>
constexpr T WrapToRange(T val, T min_val, T max_val) {
  return WrapIfNegative(std::fmod(val - min_val, max_val - min_val),
                        max_val - min_val) +
         min_val;
}

// Normalize an angle in radius into [-pi, pi).
template <typename T>
T NormalizeAngle(T angle) {
  constexpr T kPi = T(M_PI);
  constexpr T kTwoPi = T(2.0 * M_PI);
  constexpr T kInvTwoPi = T(1.0 / (2.0 * M_PI));
  // fast path
  constexpr T epsilon = 1e-6;
  if (angle >= -kPi - epsilon && angle < kPi + epsilon) {
    return angle;
  }
  T k = std::round(angle * kInvTwoPi);
  angle -= k * kTwoPi;
  if (angle >= kPi) {
    angle -= kTwoPi;
  } else if (angle < -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

// Wrap angle to [0, 2 * PI).
template <typename T>
T WrapAngle(T angle) {
  T new_angle = std::fmod(angle, M_PI * 2.0);
  return new_angle < T(0.0) ? new_angle + T(M_PI * 2.0) : new_angle;
}

// Wrap angle to [0, 360.0).
template <typename T>
T WrapAngleInDegree(T angle_in_degree) {
  T new_angle_in_degree = std::fmod(angle_in_degree, 360.0);
  return new_angle_in_degree < T(0.0) ? new_angle_in_degree + T(360.0)
                                      : new_angle_in_degree;
}

template <typename T, typename TS>
constexpr T LerpAngle(T a, T b, TS s) {
  return NormalizeAngle(a + NormalizeAngle(b - a) * s);
}

// Calculate the difference between angle from and to. The returned range is
// between [-PI, PI).
template <typename T>
constexpr T AngleDifference(T from, T to) {
  return NormalizeAngle(to - from);
}

// Compute the average angle of a group of angles in radians.
inline double MeanAngle(const std::vector<double>& angles) {
  if (angles.empty()) return 0.0;
  double sin_sum = 0.0;
  double cos_sum = 0.0;
  for (const auto angle : angles) {
    sin_sum += std::sin(angle);
    cos_sum += std::cos(angle);
  }
  return std::atan2(sin_sum, cos_sum);
}

// Get the opposite direction's angle.
inline double OppositeAngle(double angle) {
  return NormalizeAngle(angle + M_PI);
}

template <typename T>
inline T SafeClamp(const T x, const T min, const T max,
                   bool failure_upon_exception = true) {
  constexpr double kEpsilon = 1e-8;
  if (min > max - kEpsilon) {
    VLOG(1) << absl::StreamFormat(
        "Illegal clamp config: min = %7.6f > max = %7.6f !", min, max);
  }
  if (failure_upon_exception) CHECK_LE(min, max);
  return std::clamp(x, min, max);
}

template <typename T>
inline void UpdateMax(const T x, T* max_val) {
  *max_val = std::max(x, *max_val);
}

template <typename T>
inline void UpdateMin(const T x, T* min_val) {
  *min_val = std::min(x, *min_val);
}

template <typename T>
inline T Min(T&& t) {
  return std::forward<T>(t);
}
template <typename T0, typename T1, typename... Ts>
inline typename std::common_type<T0, T1, Ts...>::type Min(T0&& v0, T1&& v1,
                                                          Ts&&... vs) {
  if (v0 < v1) {
    return Min(v0, std::forward<Ts>(vs)...);
  } else {
    return Min(v1, std::forward<Ts>(vs)...);
  }
}
template <typename T>
inline T Max(T&& t) {
  return std::forward<T>(t);
}
template <typename T0, typename T1, typename... Ts>
inline typename std::common_type<T0, T1, Ts...>::type Max(T0&& v0, T1&& v1,
                                                          Ts&&... vs) {
  if (v0 > v1) {
    return Max(v0, std::forward<Ts>(vs)...);
  } else {
    return Max(v1, std::forward<Ts>(vs)...);
  }
}

template <typename T>
inline T EvaluatePolynominal(const std::vector<T>& coef, T var) {
  T res = 0;
  for (auto i = coef.rbegin(); i != coef.rend(); ++i) {
    res = res * var + *i;
  }
  return res;
}

// Returns the real root of ax^2 + bx + c = 0.0. The roots are in non-decreasing
// orders and complex roots are ignored.
std::vector<double> QuadraticRoot(double a, double b, double c);

// The discrete double set is defined as:
// {x|\exist int xid, x = xid * base}
// where base > 0.0 is a double.
// Denoted as Discrete(base).
//
// The following function returns the maximum
// double y \in Discrete(base), such that y<=x.
double FloorWithBase(double x, double base);

// Sigmoid
inline double Sigmoid(const double x) { return 1.0 / (1.0 + std::exp(-x)); }

template <typename T>
std::vector<double> SoftMax(const std::vector<T>& input) {
  double m = std::numeric_limits<double>::lowest();
  int size = input.size();
  for (int i = 0; i < size; ++i) {
    if (m < input[i]) {
      m = input[i];
    }
  }

  double sum = 0.0;
  for (int i = 0; i < size; ++i) {
    sum += exp(input[i] - m);
  }

  const double constant = m + log(sum);
  std::vector<double> output(size, 0.0);
  for (int i = 0; i < size; ++i) {
    output[i] = exp(input[i] - constant);
  }
  return output;
}

double QuadraticFunctionMinValue(double a, double b, double c, double x0,
                                 double x1);

// Return the cumulative product in [m, n], if m > n, return 0;
inline constexpr int64_t CumProd(int m, int n) {
  return (m > n) ? 0 : ((m == n) ? m : (CumProd(m, n - 1) * n));
}

inline constexpr double PowerThreeHalves(double x) {
  // CHECK_GE(x, 0.0);
  return x * std::sqrt(x);
}
}  // namespace st

#endif  // ONBOARD_MATH_UTIL_H_
