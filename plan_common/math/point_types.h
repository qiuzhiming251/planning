

#ifndef ONBOARD_MATH_POINT_TYPES_H_
#define ONBOARD_MATH_POINT_TYPES_H_

#include "plan_common/math/vec.h"

// introduce point types inherited from Vec2/Vec3， so that user could add
// customized fields while using kdtree or anyother point related algorithms.

namespace st::point_types {

template <typename T>
using PointXYZBase = Vec3<T>;

using PointXYZ = PointXYZBase<float>;

template <typename T>
using PointXYBase = Vec2<T>;

template <typename T>
struct PointXYZIBase : public PointXYZBase<T> {
  typedef T Type;

  using PointXYZBase<T>::PointXYZBase;

  PointXYZIBase(const PointXYZBase<T>& p, const uint8_t intsty)
      : PointXYZBase<T>(p), intensity(intsty) {}

  uint8_t intensity = 0;
};
using PointXYZI = PointXYZIBase<float>;

template <typename T>
struct PointXYZTBase : public PointXYZBase<T> {
  typedef T Type;

  using PointXYZBase<T>::PointXYZBase;

  PointXYZTBase(const PointXYZBase<T>& p, const double ts)
      : PointXYZBase<T>(p), timestamp(ts) {}

  double timestamp = 0.0;
};
using PointXYZT = PointXYZTBase<float>;

template <typename T>
struct PointXYZHBase : public PointXYZBase<T> {
  typedef T Type;

  using PointXYZBase<T>::PointXYZBase;

  PointXYZHBase(const PointXYZBase<T>& p, const float hght)
      : PointXYZBase<T>(p), height(hght) {}

  float height = 0.0f;
};
using PointXYZH = PointXYZHBase<float>;

template <typename T>
struct PointXYZITHBase : public PointXYZBase<T> {
  typedef T Type;

  using PointXYZBase<T>::PointXYZBase;

  double timestamp = 0.0;
  uint8_t intensity = 0;
  float height = 0.0f;
};
using PointXYZITH = PointXYZITHBase<float>;

template <typename T>
struct PointXYIdxBase : public PointXYBase<T> {
  typedef T Type;

  using PointXYBase<T>::PointXYBase;

  PointXYIdxBase(const PointXYBase<T>& p, const int idx)
      : PointXYBase<T>(p), index(idx) {}

  int index = -1;
};
using PointXYIdx = PointXYIdxBase<double>;

}  // namespace st::point_types

#endif  // ONBOARD_MATH_POINT_TYPES_H_
