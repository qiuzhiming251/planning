

#ifndef ONBOARD_MATH_GEOMETRY_AFFINE_TRANSFORMATION_H_
#define ONBOARD_MATH_GEOMETRY_AFFINE_TRANSFORMATION_H_

#include <vector>

#include "plan_common/math/eigen.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/affine_transformation.pb.h"

namespace st {

// A class representing an affine transformation for 3D homogeneous coordinates
// that provide convenient support for applying common transformations such as
// translation, rotation and scaling, and conversion between transformation
// representations such as 4x4 matrices, vectors/quaternions, and protos.
//
// Note that although the interface uses only the definitions from vec.h (Vec2d
// and Vec3d, not using EVec2d/EVec3d directly), there are several places in the
// implementation where conversions have to be made between Vec and EVec,
// because the QVec implementation is incomplete.
class AffineTransformation {
 public:
  // Identity transformation.
  AffineTransformation();

  // Construction from a 4x4 matrix. Fails on invalid matrices (those whose
  // fourth rows aren't (0, 0, 0, 1).
  explicit AffineTransformation(const Mat4d& mat);

  // Construction from another transformation.
  AffineTransformation(const AffineTransformation& other);

  // Construction from coefficients of a 3x3 or 4x4 matrix (row major).
  explicit AffineTransformation(const std::vector<double>& coeffs);

  // Construction from protos.
  explicit AffineTransformation(const TranslationProto& translation);
  explicit AffineTransformation(const RotationProto& rotation);
  explicit AffineTransformation(const ScalingProto& scaling);
  explicit AffineTransformation(const RigidTransformationProto& transformation);
  explicit AffineTransformation(
      const AffineTransformationProto& transformation);
  explicit AffineTransformation(
      const AffineTransformationSequenceProto& sequence);

  // Some conversion utilities. Quaternions involved in the functions in this
  // class are always assumed to be unit quaternions.
  static Vec3d TranslationProtoToVec3d(const TranslationProto& translation);
  static void Vec3dToTranslationProto(const Vec3d& vec,
                                      TranslationProto* translation);

  static Quaternion RotationProtoToQuaternion(const RotationProto& rotation);
  static void QuaternionToRotationProto(Quaternion quaternion,
                                        RotationProto* rotation);

  static Vec3d ScalingProtoToVec3d(const ScalingProto& scaling);
  static void Vec3dToScalingProto(const Vec3d& vec, ScalingProto* scaling);

  // Convention of Euler angles: the yaw-pitch-roll tuple refers to the
  // following sequence:
  // - a rotation about (1, 0, 0) by roll (positive is banking right), followed
  //   (left-multiplied) by
  // - a rotation about (0, 1, 0) in the world frame by pitch (positive is
  //   diving), followed by
  // - a rotation around (0, 0, 1) in the world frame by yaw (CCW in bird eye
  //   view).
  //
  // In other words, a transformation specified by (yaw, pitch, roll) is
  // equivalent to
  // AffineTransformation::FromRotation(yaw, 0, 0, 1) *
  // AffineTransformation::FromRotation(pitch, 0, 1, 0) *
  // AffineTransformation::FromRotation(roll, 1, 0, 0)
  //
  // Reference:
  //   http://planning.cs.uiuc.edu/node102.html
  //
  // In literature, this convention is called Tait-Bryan angles with the x-y-z
  // (extrinsic) order, or equivalently, with the z-y'-x" (intrinsic) order.
  //
  //
  static Quaternion YawPitchRollToQuaternion(const Vec3d& yaw_pitch_roll);
  static Vec3d QuaternionToYawPitchRoll(const Quaternion& quaternion);

  // Factories from various basic transformations, for convenient construction.
  static AffineTransformation FromTranslation(double x, double y, double z);
  static AffineTransformation FromTranslation(const Vec3d& translation);

  // Rotation CCW about axis (ax, ay, az) by angle theta (radians).
  // The input axis (ax, ay, az) does not have to be normalized.
  static AffineTransformation FromRotation(double theta, double ax, double ay,
                                           double az);
  static AffineTransformation FromRotation(double theta, const Vec3d& axis);
  static AffineTransformation FromRotation(const Quaternion& quaternion);

  static AffineTransformation FromScaling(double s);
  static AffineTransformation FromScaling(double sx, double sy, double sz);
  static AffineTransformation FromScaling(const Vec3d& scaling);

  static AffineTransformation FromYawPitchRoll(const Vec3d& yaw_pitch_roll);
  static AffineTransformation FromYawPitchRoll(double yaw, double pitch,
                                               double roll);

  static AffineTransformation FromMat4d(const Mat4d& mat);
  static AffineTransformation FromTranslationProto(
      const TranslationProto& translation);
  static AffineTransformation FromRotationProto(const RotationProto& rotation);
  static AffineTransformation FromScalingProto(const ScalingProto& scaling);
  static AffineTransformation FromRigidTransformationProto(
      const RigidTransformationProto& transformation);
  static AffineTransformation FromAffineTransformationProto(
      const AffineTransformationProto& transformation);
  static AffineTransformation FromAffineTransformationSequenceProto(
      const AffineTransformationSequenceProto& sequence);

  template <typename ExtrinsicsType>
  static AffineTransformation FromExtrinsics(const ExtrinsicsType& extrinsics) {
    return AffineTransformation::FromTranslation(extrinsics.x(), extrinsics.y(),
                                                 extrinsics.z())
        .ApplyYawPitchRoll(extrinsics.yaw(), extrinsics.pitch(),
                           extrinsics.roll());
  }

  // Methods to set the transformation.
  AffineTransformation& SetIdentity();

  AffineTransformation& SetTranslation(double x, double y, double z);
  AffineTransformation& SetTranslation(const Vec3d& translation);

  // Rotation CCW about axis (ax, ay, az) by angle theta (radians).
  // The input axis (ax, ay, az) does not have to be normalized.
  AffineTransformation& SetRotation(double theta, double ax, double ay,
                                    double az);
  AffineTransformation& SetRotation(double theta, const Vec3d& axis);
  AffineTransformation& SetRotation(Quaternion quaternion);

  AffineTransformation& SetScaling(double s);
  AffineTransformation& SetScaling(double sx, double sy, double sz);
  AffineTransformation& SetScaling(const Vec3d& scaling);

  AffineTransformation& SetYawPitchRoll(const Vec3d& yaw_pitch_roll);
  AffineTransformation& SetYawPitchRoll(double yaw, double pitch, double roll);

  AffineTransformation& Set(const Mat4d& mat);
  AffineTransformation& Set(const AffineTransformation& other);
  AffineTransformation& Set(const TranslationProto& translation);
  AffineTransformation& Set(const RotationProto& rotation);
  AffineTransformation& Set(const ScalingProto& scaling);
  AffineTransformation& Set(const RigidTransformationProto& transformation);
  AffineTransformation& Set(const AffineTransformationProto& transformation);
  AffineTransformation& Set(const AffineTransformationSequenceProto& sequence);

  // Methods to apply (right-multiply) another transformation on this one.
  // Note that applying transformation B on transformation A will make
  // transformation B operate on the operand vector first, followed by
  // A (i.e. A is replaced by A*B), similar to the convention in OpenGL.
  AffineTransformation& ApplyTranslation(double x, double y, double z);
  AffineTransformation& ApplyTranslation(const Vec3d& translation);

  AffineTransformation& ApplyRotation(double theta, double ax, double ay,
                                      double az);
  AffineTransformation& ApplyRotation(double theta, const Vec3d& axis);
  AffineTransformation& ApplyRotation(const Quaternion& quaternion);

  AffineTransformation& ApplyScaling(double s);
  AffineTransformation& ApplyScaling(double sx, double sy, double sz);
  AffineTransformation& ApplyScaling(const Vec3d& scaling);

  AffineTransformation& ApplyYawPitchRoll(const Vec3d& yaw_pitch_roll);
  AffineTransformation& ApplyYawPitchRoll(double yaw, double pitch,
                                          double roll);

  AffineTransformation& Apply(const Mat4d& mat);
  AffineTransformation& Apply(const AffineTransformation& transformation);
  AffineTransformation& Apply(const TranslationProto& translation);
  AffineTransformation& Apply(const RotationProto& rotation);
  AffineTransformation& Apply(const ScalingProto& scaling);
  AffineTransformation& Apply(const RigidTransformationProto& transformation);
  AffineTransformation& Apply(const AffineTransformationProto& transformation);
  AffineTransformation& Apply(
      const AffineTransformationSequenceProto& sequence);

  // Exporting to protos.
  void ToProto(AffineTransformationProto* transformation) const;

  // Operators.
  AffineTransformation operator*(const Mat4d& mat) const {
    return AffineTransformation(mat_ * mat);
  }
  AffineTransformation operator*(
      const AffineTransformation& transformation) const {
    return AffineTransformation(mat_ * transformation.mat_);
  }

  // Get the inverse tranformation of this one.
  AffineTransformation Inverse() const {
    return AffineTransformation(mat_.inverse());
  }

  // Access the raw matrix representation.
  const Mat4d& mat() const { return mat_; }

  // Extract the translation, rotation and scaling parts. The translational
  // part is always in the fourth column, while the rotation and scaling parts
  // are in the top-left 3x3 block. Note that while uniform scaling commutes
  // with rotation, non-uniform scaling does not, thus it is in general not
  // possible to interpret an arbitrary 3x3 matrix as a single rotation and a
  // single scaling (no matter which is after which). By SVD, the 3x3 block can
  // be decomposed into a rotation, a scaling and another rotation; in case the
  // scaling is uniform, the SVD is not unique and the two rotation can be
  // combined into a single one.
  // Due to this complexity, generally speaking the rotation and scaling
  // component extraction methods below should only be used on transformations
  // that are purely rotation or scaling by construction, and one cannot expect
  // the GetRotation() and GetScaling() results can be combined to recover the
  // original transformation.
  //
  // Extract the translation part.
  Vec3d GetTranslation() const;
  void GetTranslationProto(TranslationProto* translation) const;

  // Extract the rotation part. Per discussion above, this is only meaningful
  // when the SVD gives a (numerically) uniform scaling. In practice these
  // functions should only be called on transformations that are guaranteed to
  // remain spectrally uniform by construction (e.g. created from sequences of
  // translations, rotations and uniform scalings). If not guaranteed, use the
  // more general SVD method below. Therefore, these functions will crash on
  // transformations containing non-uniform scaling.
  Quaternion GetRotation() const;
  double GetRotationAngle(Vec3d* axis = nullptr) const;
  void GetRotationProto(RotationProto* rotation) const;
  Vec3d GetRotationYawPitchRoll() const;

  // Extract the scaling part. Note that in general SVD loses the axis ordering
  // info because it does not care about permutation in the matrices of
  // singular vectors. It also loses the signs of scaling factors if there are
  // inversions (singular values are all non-negative) for the same reason. In
  // the context of decomposition this makes perfect sense (the axis ordering
  // and inversion info has been absorbed into the rotation components) but it
  // does break the desired property that GetScaling() be a robust inverse of
  // SetScaling(). For example, if a transformation is created with SetScaling
  // (1.0, 2.0, -3.0), the SVD singular values could be (3.0, 2.0, 1.0).
  // Given this issue, we implement special handling for the case where the
  // transformation in question is rotation-free, and return the main diagonal
  // directly in those cases; otherwise we just return the singular values in
  // descending order, without even correcting for chirality (e.g. a scaling
  // of (3.0, 1.0, 2.0) combined with any rotation will be returned as
  // (3.0, 2.0, 1.0)).
  Vec3d GetScaling() const;
  void GetScalingProto(ScalingProto* scaling) const;

  // General SVD on the top-left 3x3 block. Using this (along with
  // GetTranslation), an arbitrary transformation can be decomposed into a
  // rotation, a (non-uniform) scaling, another rotation, and a translation,
  // in order.
  void Get3x3SVD(Mat3d* u, Mat3d* v, Vec3d* s) const;

  // Transform a 4D vector (homogeneous coordinates).
  Vec4d Transform(const Vec4d& x) const { return mat_ * x; }

  // Apply the transformation on a point (first converting the point to the
  // homogeneous coordinates and then transform).
  Vec3d TransformPoint(const Vec3d& x) const {
    // Return the vector transformed by the top-left 3x3 block (rotation and
    // scaling) followed by translation by the fourth column. No need to do the
    // homogeneous division because mat_'s fourth row is always (0, 0, 0, 1) and
    // the homogenized x always has w = 1.
    return Vec3d(mat_.block<3, 3>(0, 0) * x + mat_.block<3, 1>(0, 3));
  }

  // Apply the transformation on a vector. The difference is that a vector is
  // the difference between two points in the same coordinate frame, so it
  // is not affected by the translation component of the transformation.
  // In other words, transforming a point is subjecting the point to all the
  // translation, rotation and scaling operations, while transforming a vector
  // will only apply rotation and scaling (only the top left 3x3 block).
  Vec3d TransformVector(const Vec3d& x) const {
    return Vec3d(mat_.block<3, 3>(0, 0) * x);
  }

  // Apply the transformation on a covector. Covectors are not difference
  // between points (like vectors), but instead are measurements for vectors;
  // they trasnform covariantly rather than contravariantly like vectors do
  // (i.e. covectors are transformed by the inverse transpose of the top left
  // 3x3 block; this makes a difference only when there's scaling).
  // In practice the most common covectors are the surface normals.
  Vec3d TransformCovector(const Vec3d& x) const {
    return Vec3d(mat_.block<3, 3>(0, 0).transpose().fullPivLu().solve(x));
  }

 private:
  void GetRotation(double* theta, Vec3d* axis) const;

  // The raw matrix representation. The fourth row must always be (0, 0, 0, 1).
  // All operations above either will respect this invariant by construction or
  // will fail a check upon attempting to violate it.
  Mat4d mat_;
};

}  // namespace st

#endif  // ONBOARD_MATH_GEOMETRY_AFFINE_TRANSFORMATION_H_
