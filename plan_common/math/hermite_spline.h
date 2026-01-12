

#ifndef ONBOARD_MATH_HERMITE_SPLINE_H_
#define ONBOARD_MATH_HERMITE_SPLINE_H_

namespace st {

template <typename T, typename TS>
constexpr T CubicHermiteLerp(T a, T b, T adot, T bdot, TS s) {
  // https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Interpolation_on_a_single_interval
  const auto s2 = s * s;
  const auto s3 = s2 * s;
  return (2.0 * s3 - 3.0 * s2 + 1.0) * a + (s3 - 2.0 * s2 + s) * adot +
         (-2.0 * s3 + 3.0 * s2) * b + (s3 - s2) * bdot;
}

// dCubicHermiteLerp(a, b, adot, bdot, s)/ds.
template <typename T, typename TS>
constexpr T CubicHermiteDerivative(T a, T b, T adot, T bdot, TS s) {
  const auto s2 = s * s;
  return (6.0 * s2 - 6.0 * s) * a + (3.0 * s2 - 4.0 * s + 1.0) * adot +
         (-6.0 * s2 + 6.0 * s) * b + (3.0 * s2 - 2.0 * s) * bdot;
}

// d^2CubicHermiteLerp(a, b, adot, bdot, s)/ds^2.
template <typename T, typename TS>
constexpr T CubicHermiteSecondDerivative(T a, T b, T adot, T bdot, TS s) {
  return (12.0 * s - 6.0) * a + (6.0 * s - 4.0) * adot + (-12.0 * s + 6.0) * b +
         (6.0 * s - 2.0) * bdot;
}

template <typename T, typename TS>
constexpr T LinearCubicLinearLerp(T a, T b, T adot, T bdot, TS s) {
  if (s > 1.0) return b + bdot * (s - 1.0);
  if (s < 0.0) return a + adot * s;
  return CubicHermiteLerp(a, b, adot, bdot, s);
}

template <typename T, typename TS>
constexpr T QuinticHermiteLerp(T a, T b, T adot, T bdot, T addot, T bddot,
                               TS s) {
  // https://www.rose-hulman.edu/~finn/CCLI/Notes/day09.pdf
  const TS s2 = s * s;
  const TS s3 = s2 * s;
  const TS s4 = s3 * s;
  const TS s5 = s4 * s;
  return (1.0 - 10.0 * s3 + 15.0 * s4 - 6.0 * s5) * a +
         (s - 6.0 * s3 + 8.0 * s4 - 3.0 * s5) * adot +
         (0.5 * s2 - 1.5 * s3 + 1.5 * s4 - 0.5 * s5) * addot +
         (0.5 * s3 - s4 + 0.5 * s5) * bddot +
         (-4.0 * s3 + 7.0 * s4 - 3.0 * s5) * bdot +
         (10.0 * s3 - 15.0 * s4 + 6.0 * s5) * b;
}

// dQuinticHermiteLerp(a, b, adot, bdot, addot, bddot)/ds.
template <typename T, typename TS>
constexpr T QuinticHermiteDerivative(T a, T b, T adot, T bdot, T addot, T bddot,
                                     TS s) {
  const TS s2 = s * s;
  const TS s3 = s2 * s;
  const TS s4 = s3 * s;
  return (-30.0 * s2 + 60.0 * s3 - 30.0 * s4) * a +
         (1.0 - 18.0 * s2 + 32.0 * s3 - 15.0 * s4) * adot +
         (s - 4.5 * s2 + 6.0 * s3 - 2.5 * s4) * addot +
         (1.5 * s2 - 4.0 * s3 + 2.5 * s4) * bddot +
         (-12.0 * s2 + 28.0 * s3 - 15.0 * s4) * bdot +
         (30.0 * s2 - 60.0 * s3 + 30.0 * s4) * b;
}

// d^2QuinticHermiteLerp(a, b, adot, bdot, addot, bddot)/ds^2.
template <typename T, typename TS>
constexpr T QuinticHermiteSecondDerivative(T a, T b, T adot, T bdot, T addot,
                                           T bddot, TS s) {
  const TS s2 = s * s;
  const TS s3 = s2 * s;
  return (-60.0 * s + 180.0 * s2 - 120.0 * s3) * a +
         (-36.0 * s + 96.0 * s2 - 60.0 * s3) * adot +
         (1.0 - 9.0 * s + 18.0 * s2 - 10.0 * s3) * addot +
         (3.0 * s - 12.0 * s2 + 10.0 * s3) * bddot +
         (-24.0 * s + 84.0 * s2 - 60.0 * s3) * bdot +
         (60.0 * s - 180.0 * s2 + 120.0 * s3) * b;
}

// d^3QuinticHermiteLerp(a, b, adot, bdot, addot, bddot)/ds^3.
template <typename T, typename TS>
constexpr T QuinticHermiteThirdDerivative(T a, T b, T adot, T bdot, T addot,
                                          T bddot, TS s) {
  const TS s2 = s * s;
  return (-60.0 + 360.0 * s - 360.0 * s2) * a +
         (-36.0 + 192.0 * s - 180.0 * s2) * adot +
         (-9.0 + 36.0 * s - 30.0 * s2) * addot +
         (3.0 - 24.0 * s + 30.0 * s2) * bddot +
         (-24.0 + 168.0 * s - 180.0 * s2) * bdot +
         (60.0 - 360.0 * s + 360.0 * s2) * b;
}

template <typename T, typename TS>
constexpr T QuadraticQuinticQuadraticLerp(T a, T b, T adot, T bdot, T addot,
                                          T bddot, TS s) {
  if (s > 1.0) return b + bdot * (s - 1.0) + bddot * 0.5 * Sqr(s - 1.0);
  if (s < 0.0) return a + adot * s + addot * 0.5 * Sqr(s);
  return QuinticHermiteLerp(a, b, adot, bdot, addot, bddot, s);
}

}  // namespace st

#endif  // ONBOARD_MATH_HERMITE_SPLINE_H_
