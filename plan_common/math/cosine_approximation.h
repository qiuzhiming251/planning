
#ifndef ONBOARD_MATH_COSINE_APPROXIMATION_H_
#define ONBOARD_MATH_COSINE_APPROXIMATION_H_

// Reference to code in http://www.ganssle.com/approx/sincos.cpp.

namespace st::fast_math {

// *********************************************************
// ***
// ***   Routines to compute sine and cosine to 5.2 digits
// ***  of accuracy.
// ***
// *********************************************************
//
//    Cos_52 computes cosine (x)
//
//  Accurate to about 5.2 decimal digits over the range [0, pi/2].
//  The input argument is in radians.
//
//  Algorithm:
//    cos(x)= c1 + c2*x**2 + c3*x**4 + c4*x**6
//   which is the same as:
//    cos(x)= c1 + x**2(c2 + c3*x**2 + c4*x**4)
//    cos(x)= c1 + x**2(c2 + x**2(c3 + c4*x**2))
//
inline double Cos_52(double x) {
  const double c1 = 0.9999932946;
  const double c2 = -0.4999124376;
  const double c3 = 0.0414877472;
  const double c4 = -0.0012712095;
  const double x2 = x * x;  // The input argument squared
  return (c1 + x2 * (c2 + x2 * (c3 + c4 * x2)));
}

// *********************************************************
// ***
// ***   Routines to compute sine and cosine to 7.3 digits
// ***  of accuracy.
// ***
// *********************************************************
//
//    Cos_73 computes cosine (x)
//
//  Accurate to about 7.3 decimal digits over the range [0, pi/2].
//  The input argument is in radians.
//
//  Algorithm:
//    cos(x)= c1 + c2*x**2 + c3*x**4 + c4*x**6 + c5*x**8
//   which is the same as:
//    cos(x)= c1 + x**2(c2 + c3*x**2 + c4*x**4 + c5*x**6)
//    cos(x)= c1 + x**2(c2 + x**2(c3 + c4*x**2 + c5*x**4))
//    cos(x)= c1 + x**2(c2 + x**2(c3 + x**2(c4 + c5*x**2)))
//
inline double Cos_73(double x) {
  constexpr double c1 = 0.999999953464;
  constexpr double c2 = -0.499999053455;
  constexpr double c3 = 0.0416635846769;
  constexpr double c4 = -0.0013853704264;
  constexpr double c5 = 0.00002315393167;

  const double x2 = x * x;
  return (c1 + x2 * (c2 + x2 * (c3 + x2 * (c4 + c5 * x2))));
}

// *********************************************************
// ***
// ***   Routines to compute sine and cosine to 12.1 digits
// ***  of accuracy.
// ***
// *********************************************************
//
//    Cos_121 computes cosine (x)
//
//  Accurate to about 12.1 decimal digits over the range [0, pi/2].
//  The input argument is in radians.
//
//  Algorithm:
//    cos(x)= c1 + c2*x**2 + c3*x**4 + c4*x**6 + c5*x**8 + c6*x**10 +
// c7*x**12
//   which is the same as:
//    cos(x)= c1 + x**2(c2 + c3*x**2 + c4*x**4 + c5*x**6 + c6*x**8 +
// c7*x**10)     cos(x)= c1 + x**2(c2 + x**2(c3 + c4*x**2 + c5*x**4 +
// c6*x**6 + c7*x**8 ))     cos(x)= c1 + x**2(c2 + x**2(c3 + x**2(c4
// + c5*x**2 + c6*x**4 + c7*x**6 )))     cos(x)= c1 + x**2(c2 + x**2(c3 +
// x**2(c4 + x**2(c5 + c6*x**2 + c7*x**4 ))))     cos(x)= c1 + x**2(c2 +
// x**2(c3 + x**2(c4 + x**2(c5 + x**2(c6 + c7*x**2 )))))
//
inline double Cos_121(double x) {
  constexpr double c1 = 0.99999999999925182;
  constexpr double c2 = -0.49999999997024012;
  constexpr double c3 = 0.041666666473384543;
  constexpr double c4 = -0.001388888418000423;
  constexpr double c5 = 0.0000248010406484558;
  constexpr double c6 = -0.0000002752469638432;
  constexpr double c7 = 0.0000000019907856854;

  const double x2 = x * x;
  return (c1 +
          x2 * (c2 + x2 * (c3 + x2 * (c4 + x2 * (c5 + x2 * (c6 + c7 * x2))))));
}

template <int N = 7>
inline double CosPi2(double angle) {
  switch (N) {
    case 5:
      return Cos_52(angle);
    case 7:
      return Cos_73(angle);
    case 12:
      return Cos_121(angle);
  }
}

}  // namespace st::fast_math

#endif  // ONBOARD_MATH_COSINE_APPROXIMATION_H_
