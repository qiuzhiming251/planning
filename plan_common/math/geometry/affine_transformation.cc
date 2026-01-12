

#include <algorithm>
#include <cmath>
#include <ostream>
#include <utility>

#include <string.h>

#include "Eigen/Jacobi"
#include "plan_common/math/geometry/affine_transformation.h"
#include "plan_common/math/util.h"

namespace st {
namespace {

constexpr double kEpsilon = 1e-10;

// Compute the cross product matrix of a vector.
// Reference:
// https://en.wikipedia.org/wiki/Cross_product#Conversion_to_matrix_multiplication
Mat3d VectorToSkewSymmetricMatrix(const Vec3d& u) {
  Mat3d u_cross = Mat3d::Zero();
  u_cross(2, 1) = u.x();
  u_cross(0, 2) = u.y();
  u_cross(1, 0) = u.z();
  u_cross(1, 2) = -u.x();
  u_cross(2, 0) = -u.y();
  u_cross(0, 1) = -u.z();
  return u_cross;
}

Vec3d SkewSymmetricMatrixToVector(const Mat3d& u_cross) {
  // Assert the skew symmetric matrix is well-formed.
  CHECK_LT(std::abs(u_cross(2, 1) + u_cross(1, 2)), kEpsilon);
  CHECK_LT(std::abs(u_cross(0, 2) + u_cross(2, 0)), kEpsilon);
  CHECK_LT(std::abs(u_cross(1, 0) + u_cross(0, 1)), kEpsilon);
  CHECK_LT(std::abs(u_cross(0, 0)), kEpsilon);
  CHECK_LT(std::abs(u_cross(1, 1)), kEpsilon);
  CHECK_LT(std::abs(u_cross(2, 2)), kEpsilon);
  return Vec3d(u_cross(2, 1), u_cross(0, 2), u_cross(1, 0));
}

// Compute a 3x3 matrix for rotation about unit vector u by angle theta.
// Reference:
// https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
Mat3d RotationMatrixFromAngleAndUnitAxis(double cos_theta, double sin_theta,
                                         const Vec3d& u) {
  // [u]_x
  Mat3d u_cross = VectorToSkewSymmetricMatrix(u);
  // u * u^T
  Mat3d u_ut = u * u.transpose();
  return cos_theta * Mat3d::Identity() + sin_theta * u_cross +
         (1.0 - cos_theta) * u_ut;
}

// Compute the angle theta (-pi to pi) and axis u from a 3x3 rotation matrix.
// Reference:
// https://en.wikipedia.org/wiki/Rotation_matrix#Determining_the_axis
// https://en.wikipedia.org/wiki/Rotation_matrix#Determining_the_angle
// https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
void AngleAndUnitAxisFromRotationMatrix(const Mat3d& rot, double* theta,
                                        Vec3d* u) {
  CHECK(theta != nullptr);
  CHECK(u != nullptr);
  const Vec3d axis = SkewSymmetricMatrixToVector(rot - rot.transpose());
  const double n = axis.norm();

  double sin_theta = n * 0.5;
  CHECK_LT(sin_theta, 1.0 + kEpsilon);
  sin_theta = std::min(1.0, sin_theta);

  double cos_theta = (rot.trace() - 1.0) * 0.5;
  CHECK_LT(cos_theta, 1.0 + kEpsilon);
  CHECK_GT(cos_theta, -1.0 - kEpsilon);
  cos_theta = std::min(1.0, std::max(-1.0, cos_theta));

  if (n < kEpsilon) {
    if (rot(0, 0) < (-1 + kEpsilon) && rot(1, 1) < (-1 + kEpsilon)) {
      *theta = std::acos(cos_theta);
      *u = Vec3d(0.0, 0.0, 1.0);
      return;
    } else if ((rot(0, 0) < (-1 + kEpsilon) && rot(2, 2) < (-1 + kEpsilon))) {
      *theta = std::acos(cos_theta);
      *u = Vec3d(0.0, 1.0, 0.0);
      return;
    } else if ((rot(1, 1) < (-1 + kEpsilon) && rot(2, 2) < (-1 + kEpsilon))) {
      *theta = std::acos(cos_theta);
      *u = Vec3d(1.0, 0.0, 0.0);
      return;
    } else {
      *theta = 0.0;
      *u = Vec3d(1.0, 0.0, 0.0);
      return;
    }
    // The rotation is almost a no-op.
  }
  *theta = std::acos(cos_theta);
  if (sin_theta < 0.0) *theta = -*theta;
  *u = axis / n;
}

}  // namespace

AffineTransformation::AffineTransformation() { SetIdentity(); }

AffineTransformation::AffineTransformation(const Mat4d& mat) { Set(mat); }

AffineTransformation::AffineTransformation(const AffineTransformation& other) {
  mat_ = other.mat_;
}

AffineTransformation::AffineTransformation(
    const TranslationProto& translation) {
  Set(translation);
}

AffineTransformation::AffineTransformation(const RotationProto& rotation) {
  Set(rotation);
}

AffineTransformation::AffineTransformation(const ScalingProto& scaling) {
  Set(scaling);
}

AffineTransformation::AffineTransformation(
    const RigidTransformationProto& transformation) {
  Set(transformation);
}

AffineTransformation::AffineTransformation(
    const AffineTransformationProto& transformation) {
  Set(transformation);
}

AffineTransformation::AffineTransformation(
    const AffineTransformationSequenceProto& sequence) {
  Set(sequence);
}

AffineTransformation::AffineTransformation(const std::vector<double>& coeffs) {
  SetIdentity();
  CHECK(coeffs.size() == 9 || coeffs.size() == 16);
  if (coeffs.size() == 9) {
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> m;
    memcpy(m.data(), coeffs.data(), sizeof(double) * 9);
    mat_.block<3, 3>(0, 0) = m;
  } else {
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> m;
    memcpy(m.data(), coeffs.data(), sizeof(double) * 16);
    mat_ = m;
  }
}

Vec3d AffineTransformation::TranslationProtoToVec3d(
    const TranslationProto& translation) {
  return Vec3d(translation.x(), translation.y(), translation.z());
}

void AffineTransformation::Vec3dToTranslationProto(
    const Vec3d& vec, TranslationProto* translation) {
  CHECK(translation != nullptr);
  translation->Clear();
  translation->set_x(vec.x());
  translation->set_y(vec.y());
  translation->set_z(vec.z());
}

Quaternion AffineTransformation::RotationProtoToQuaternion(
    const RotationProto& rotation) {
  const double half_angle = d2r(rotation.angle()) * 0.5;
  Vec3d axis(rotation.ax(), rotation.ay(), rotation.az());
  if (axis.squaredNorm() == 0.0) axis = Vec3d(1.0, 0.0, 0.0);
  Quaternion q;
  q.w() = std::cos(half_angle);
  q.vec() = axis.normalized() * std::sin(half_angle);
  return q;
}

void AffineTransformation::QuaternionToRotationProto(Quaternion quaternion,
                                                     RotationProto* rotation) {
  CHECK(rotation != nullptr);
  rotation->Clear();
  const Vec3d axis = quaternion.vec().normalized();
  const double angle = r2d(std::acos(quaternion.w())) * 2.0;
  rotation->set_ax(axis.x());
  rotation->set_ay(axis.y());
  rotation->set_az(axis.z());
  rotation->set_angle(angle);
}

Vec3d AffineTransformation::ScalingProtoToVec3d(const ScalingProto& scaling) {
  return Vec3d(scaling.sx(), scaling.sy(), scaling.sz());
}

void AffineTransformation::Vec3dToScalingProto(const Vec3d& vec,
                                               ScalingProto* scaling) {
  CHECK(scaling != nullptr);
  scaling->Clear();
  scaling->set_sx(vec.x());
  scaling->set_sy(vec.y());
  scaling->set_sz(vec.z());
}

Quaternion AffineTransformation::YawPitchRollToQuaternion(
    const Vec3d& yaw_pitch_roll) {
  const double yaw = yaw_pitch_roll.x();
  const double pitch = yaw_pitch_roll.y();
  const double roll = yaw_pitch_roll.z();
  return AngleAxis(yaw, Vec3d::UnitZ()) * AngleAxis(pitch, Vec3d::UnitY()) *
         AngleAxis(roll, Vec3d::UnitX());
}

Vec3d AffineTransformation::QuaternionToYawPitchRoll(
    const Quaternion& quaternion) {
  // http://eigen.tuxfamily.org/dox/group__Geometry__Module.html#ga17994d2e81b723295f5bc3b1f862ed3b
  // Eigen's eulerAngles() implementation gives different ranges for the three
  // angles returned: the first is in [0, pi], while the latter two are in
  // [-pi, pi], contrary to our convention where yaw is in [-pi, pi), pitch in
  // [-pi/2, pi/2], and roll in [-pi, pi).
  // Therefore naively returning eulerAngles(2, 1, 0) may give very
  // wrong results: for a near-zero rotation this could return ~(pi, pi, pi)
  // instead of ~(0, 0, 0) which is desired, although the two sets of Euler
  // angles correspond to exactly the same rotation.
  // Equivalence between Euler angles:
  // (alpha, beta, gamma) is equivalent to (pi + alpha, pi - beta, pi + gamma).
  Vec3d angles = quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
  if (angles.y() > M_PI * 0.5) {
    angles.y() = M_PI - angles.y();
    angles.x() = NormalizeAngle(angles.x() + M_PI);
    angles.z() = NormalizeAngle(angles.z() + M_PI);
  } else if (angles.y() < -M_PI * 0.5) {
    angles.y() = -M_PI - angles.y();
    angles.x() = NormalizeAngle(angles.x() + M_PI);
    angles.z() = NormalizeAngle(angles.z() + M_PI);
  }
  return angles;
}

AffineTransformation AffineTransformation::FromTranslation(double x, double y,
                                                           double z) {
  return AffineTransformation().SetTranslation(x, y, z);
}

AffineTransformation AffineTransformation::FromTranslation(
    const Vec3d& translation) {
  return AffineTransformation().SetTranslation(translation);
}

AffineTransformation AffineTransformation::FromRotation(double theta, double ax,
                                                        double ay, double az) {
  return AffineTransformation().SetRotation(theta, ax, ay, az);
}

AffineTransformation AffineTransformation::FromRotation(double theta,
                                                        const Vec3d& axis) {
  return AffineTransformation().SetRotation(theta, axis);
}

AffineTransformation AffineTransformation::FromRotation(
    const Quaternion& quaternion) {
  return AffineTransformation().SetRotation(quaternion);
}

AffineTransformation AffineTransformation::FromScaling(double s) {
  return AffineTransformation().SetScaling(s);
}

AffineTransformation AffineTransformation::FromScaling(double sx, double sy,
                                                       double sz) {
  return AffineTransformation().SetScaling(sx, sy, sz);
}

AffineTransformation AffineTransformation::FromScaling(const Vec3d& scaling) {
  return AffineTransformation().SetScaling(scaling);
}

AffineTransformation AffineTransformation::FromYawPitchRoll(
    const Vec3d& yaw_pitch_roll) {
  return AffineTransformation().SetYawPitchRoll(yaw_pitch_roll);
}

AffineTransformation AffineTransformation::FromYawPitchRoll(double yaw,
                                                            double pitch,
                                                            double roll) {
  return AffineTransformation().SetYawPitchRoll(yaw, pitch, roll);
}

AffineTransformation AffineTransformation::FromMat4d(const Mat4d& mat) {
  return AffineTransformation().Set(mat);
}

AffineTransformation AffineTransformation::FromTranslationProto(
    const TranslationProto& translation) {
  return AffineTransformation().Set(translation);
}

AffineTransformation AffineTransformation::FromRotationProto(
    const RotationProto& rotation) {
  return AffineTransformation().Set(rotation);
}

AffineTransformation AffineTransformation::FromScalingProto(
    const ScalingProto& scaling) {
  return AffineTransformation().Set(scaling);
}

AffineTransformation AffineTransformation::FromRigidTransformationProto(
    const RigidTransformationProto& transformation) {
  return AffineTransformation().Set(transformation);
}

AffineTransformation AffineTransformation::FromAffineTransformationProto(
    const AffineTransformationProto& transformation) {
  return AffineTransformation().Set(transformation);
}

AffineTransformation
AffineTransformation::FromAffineTransformationSequenceProto(
    const AffineTransformationSequenceProto& sequence) {
  return AffineTransformation().Set(sequence);
}

AffineTransformation& AffineTransformation::SetIdentity() {
  mat_ = Mat4d::Identity();
  return *this;
}

AffineTransformation& AffineTransformation::SetTranslation(double x, double y,
                                                           double z) {
  mat_ = Mat4d::Identity();
  mat_(0, 3) = x;
  mat_(1, 3) = y;
  mat_(2, 3) = z;
  return *this;
}

AffineTransformation& AffineTransformation::SetTranslation(
    const Vec3d& translation) {
  mat_ = Mat4d::Identity();
  mat_(0, 3) = translation.x();
  mat_(1, 3) = translation.y();
  mat_(2, 3) = translation.z();
  return *this;
}

AffineTransformation& AffineTransformation::SetRotation(double theta, double ax,
                                                        double ay, double az) {
  return SetRotation(theta, Vec3d(ax, ay, az));
}

AffineTransformation& AffineTransformation::SetRotation(double theta,
                                                        const Vec3d& axis) {
  mat_ = Mat4d::Identity();
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);
  auto normalized_axis = axis;
  normalized_axis.normalize();
  mat_.block<3, 3>(0, 0) =
      RotationMatrixFromAngleAndUnitAxis(cos_theta, sin_theta, normalized_axis);
  return *this;
}

AffineTransformation& AffineTransformation::SetRotation(Quaternion quaternion) {
  mat_ = Mat4d::Identity();
  // This function can also be implemented by quaternion.toRotationMatrix().
  const double cos_theta_2 = quaternion.w();
  const double cos_theta_2_sqr = cos_theta_2 * cos_theta_2;
  const double sin_theta_2_sqr = quaternion.vec().squaredNorm();
  const double sin_theta_2 = std::sqrt(sin_theta_2_sqr);

  // Looser constraint here to prevent crash.
  constexpr double kLargerEpsilon = 1e-6;
  // Check the quaternion is well-formed.
  CHECK_LT(std::abs(cos_theta_2_sqr + sin_theta_2_sqr - 1.0), kLargerEpsilon)
      << "quaternion.w " << quaternion.w() << " quaternion.v "
      << quaternion.vec().transpose();

  if (sin_theta_2 < kEpsilon) {
    // The quaternion is almost a no-op. Leave this transformation as identity.
    return *this;
  }
  const Vec3d axis =
      Vec3d(quaternion.vec()) / sin_theta_2;  // Normalize the axis.
  const double cos_theta = cos_theta_2_sqr - sin_theta_2_sqr;
  const double sin_theta = 2.0 * cos_theta_2 * sin_theta_2;
  mat_.block<3, 3>(0, 0) =
      RotationMatrixFromAngleAndUnitAxis(cos_theta, sin_theta, axis);
  return *this;
}

AffineTransformation& AffineTransformation::SetYawPitchRoll(
    const Vec3d& yaw_pitch_roll) {
  return SetRotation(YawPitchRollToQuaternion(yaw_pitch_roll));
}

AffineTransformation& AffineTransformation::SetYawPitchRoll(double yaw,
                                                            double pitch,
                                                            double roll) {
  return SetRotation(YawPitchRollToQuaternion({yaw, pitch, roll}));
}

AffineTransformation& AffineTransformation::SetScaling(double s) {
  return SetScaling(s, s, s);
}

AffineTransformation& AffineTransformation::SetScaling(double sx, double sy,
                                                       double sz) {
  mat_ = Mat4d::Identity();
  mat_(0, 0) = sx;
  mat_(1, 1) = sy;
  mat_(2, 2) = sz;
  return *this;
}

AffineTransformation& AffineTransformation::SetScaling(const Vec3d& scaling) {
  return SetScaling(scaling.x(), scaling.y(), scaling.z());
}

AffineTransformation& AffineTransformation::Set(const Mat4d& mat) {
  // Strong assertions, with no numerical tolerance. No operations below
  // produce an "approximately" affine transformation; a 4x4 matrix is
  // either exactly an affine transformation (for homogeneous coordinates)
  // or it is not an affine transformation at all.
  CHECK_EQ(mat(3, 0), 0.0);
  CHECK_EQ(mat(3, 1), 0.0);
  CHECK_EQ(mat(3, 2), 0.0);
  CHECK_EQ(mat(3, 3), 1.0);
  mat_ = mat;
  return *this;
}

AffineTransformation& AffineTransformation::Set(
    const AffineTransformation& other) {
  mat_ = other.mat_;
  return *this;
}

AffineTransformation& AffineTransformation::Set(
    const TranslationProto& translation) {
  return SetTranslation(TranslationProtoToVec3d(translation));
}

AffineTransformation& AffineTransformation::Set(const RotationProto& rotation) {
  return SetRotation(RotationProtoToQuaternion(rotation));
}

AffineTransformation& AffineTransformation::Set(const ScalingProto& scaling) {
  return SetScaling(ScalingProtoToVec3d(scaling));
}

AffineTransformation& AffineTransformation::Set(
    const RigidTransformationProto& transformation) {
  Set(transformation.translation());
  return Apply(transformation.rotation());
}

AffineTransformation& AffineTransformation::Set(
    const AffineTransformationProto& transformation) {
  Mat4d mat;
  mat(0, 0) = transformation.m(0);
  mat(1, 0) = transformation.m(1);
  mat(2, 0) = transformation.m(2);
  mat(3, 0) = transformation.m(3);
  mat(0, 1) = transformation.m(4);
  mat(1, 1) = transformation.m(5);
  mat(2, 1) = transformation.m(6);
  mat(3, 1) = transformation.m(7);
  mat(0, 2) = transformation.m(8);
  mat(1, 2) = transformation.m(9);
  mat(2, 2) = transformation.m(10);
  mat(3, 2) = transformation.m(11);
  mat(0, 3) = transformation.m(12);
  mat(1, 3) = transformation.m(13);
  mat(2, 3) = transformation.m(14);
  mat(3, 3) = transformation.m(15);
  return Set(mat);
}

AffineTransformation& AffineTransformation::Set(
    const AffineTransformationSequenceProto& sequence) {
  SetIdentity();
  for (const AffineTransformationSequenceProto::Operation& operation :
       sequence.operations()) {
    switch (operation.operation_case()) {
      case AffineTransformationSequenceProto::Operation::kTranslation:
        Apply(operation.translation());
        break;
      case AffineTransformationSequenceProto::Operation::kRotation:
        Apply(operation.rotation());
        break;
      case AffineTransformationSequenceProto::Operation::kScaling:
        Apply(operation.scaling());
        break;
      case AffineTransformationSequenceProto::Operation::OPERATION_NOT_SET:
      default:
        break;
    }
  }
  return *this;
}

AffineTransformation& AffineTransformation::ApplyTranslation(double x, double y,
                                                             double z) {
  const Vec4d t(x, y, z, 1.0);
  mat_(0, 3) = mat_.row(0).dot(t);
  mat_(1, 3) = mat_.row(1).dot(t);
  mat_(2, 3) = mat_.row(2).dot(t);
  return *this;
}

AffineTransformation& AffineTransformation::ApplyTranslation(
    const Vec3d& translation) {
  return ApplyTranslation(translation.x(), translation.y(), translation.z());
}

AffineTransformation& AffineTransformation::ApplyRotation(double theta,
                                                          double ax, double ay,
                                                          double az) {
  return Apply(FromRotation(theta, ax, ay, az));
}

AffineTransformation& AffineTransformation::ApplyRotation(double theta,
                                                          const Vec3d& axis) {
  return Apply(FromRotation(theta, axis));
}

AffineTransformation& AffineTransformation::ApplyRotation(
    const Quaternion& quaternion) {
  return Apply(FromRotation(quaternion));
}

AffineTransformation& AffineTransformation::ApplyScaling(double s) {
  mat_.block<3, 3>(0, 0) *= s;
  return *this;
}

AffineTransformation& AffineTransformation::ApplyScaling(double sx, double sy,
                                                         double sz) {
  mat_.col(0) *= sx;
  mat_.col(1) *= sy;
  mat_.col(2) *= sz;
  return *this;
}

AffineTransformation& AffineTransformation::ApplyYawPitchRoll(
    const Vec3d& yaw_pitch_roll) {
  return Apply(FromYawPitchRoll(yaw_pitch_roll));
}

AffineTransformation& AffineTransformation::ApplyYawPitchRoll(double yaw,
                                                              double pitch,
                                                              double roll) {
  return Apply(FromYawPitchRoll({yaw, pitch, roll}));
}

AffineTransformation& AffineTransformation::ApplyScaling(const Vec3d& scaling) {
  return ApplyScaling(scaling.x(), scaling.y(), scaling.z());
}

AffineTransformation& AffineTransformation::Apply(const Mat4d& mat) {
  CHECK_EQ(mat(3, 0), 0.0);
  CHECK_EQ(mat(3, 1), 0.0);
  CHECK_EQ(mat(3, 2), 0.0);
  CHECK_EQ(mat(3, 3), 1.0);
  mat_ = mat_ * mat;
  return *this;
}

AffineTransformation& AffineTransformation::Apply(
    const AffineTransformation& transformation) {
  mat_ = mat_ * transformation.mat_;
  return *this;
}

AffineTransformation& AffineTransformation::Apply(
    const TranslationProto& translation) {
  return ApplyTranslation(TranslationProtoToVec3d(translation));
}

AffineTransformation& AffineTransformation::Apply(
    const RotationProto& rotation) {
  return ApplyRotation(RotationProtoToQuaternion(rotation));
}

AffineTransformation& AffineTransformation::Apply(const ScalingProto& scaling) {
  return ApplyScaling(ScalingProtoToVec3d(scaling));
}

AffineTransformation& AffineTransformation::Apply(
    const RigidTransformationProto& transformation) {
  return Apply(FromRigidTransformationProto(transformation));
}

AffineTransformation& AffineTransformation::Apply(
    const AffineTransformationProto& transformation) {
  return Apply(FromAffineTransformationProto(transformation));
}

AffineTransformation& AffineTransformation::Apply(
    const AffineTransformationSequenceProto& sequence) {
  return Apply(FromAffineTransformationSequenceProto(sequence));
}

void AffineTransformation::ToProto(
    AffineTransformationProto* transformation) const {
  transformation->Clear();
  transformation->mutable_m()->Resize(16, 0.0);
  for (int i = 0; i < 16; ++i) transformation->set_m(i, mat_.data()[i]);
}

Vec3d AffineTransformation::GetTranslation() const {
  return Vec3d(mat_.col(3).segment<3>(0));
}

void AffineTransformation::GetTranslationProto(
    TranslationProto* translation) const {
  Vec3dToTranslationProto(GetTranslation(), translation);
}

void AffineTransformation::GetRotation(double* theta, Vec3d* axis) const {
  CHECK(theta != nullptr);
  CHECK(axis != nullptr);
  Mat3d u, v;
  Vec3d s;
  Get3x3SVD(&u, &v, &s);
  // Assume the transformation doesn't contain any non-uniform scaling part.
  CHECK_LT(std::abs(s.x() - s.y()), kEpsilon);
  CHECK_LT(std::abs(s.x() - s.z()), kEpsilon);
  // No need to correct for chirality: although u and v may have negative
  // determinants, they can only be both negative at the same time or both
  // positive at the same time, so u * v^T is always correctly right-handed.
  const Mat3d rot = u * v.transpose();
  AngleAndUnitAxisFromRotationMatrix(rot, theta, axis);
}

Quaternion AffineTransformation::GetRotation() const {
  double theta;
  Vec3d axis;
  GetRotation(&theta, &axis);
  axis *= std::sin(theta * 0.5);
  return Quaternion(std::cos(theta * 0.5), axis.x(), axis.y(), axis.z());
}

double AffineTransformation::GetRotationAngle(Vec3d* axis) const {
  double theta;
  Vec3d axis_local;
  GetRotation(&theta, &axis_local);
  if (axis != nullptr) *axis = axis_local;
  return theta;
}

void AffineTransformation::GetRotationProto(RotationProto* rotation) const {
  QuaternionToRotationProto(GetRotation(), rotation);
}

Vec3d AffineTransformation::GetRotationYawPitchRoll() const {
  return QuaternionToYawPitchRoll(GetRotation());
}

Vec3d AffineTransformation::GetScaling() const {
  // Special case for rotation-free matrices.
  if (mat_(0, 1) == 0.0 && mat_(0, 2) == 0.0 && mat_(1, 2) == 0.0 &&
      mat_(1, 0) == 0.0 && mat_(2, 0) == 0.0 && mat_(2, 1) == 0.0) {
    return Vec3d(mat_(0, 0), mat_(1, 1), mat_(2, 2));
  }
  // Return the singular values (no chirality correction).
  Vec3d s;
  Get3x3SVD(/*u=*/nullptr, /*v=*/nullptr, &s);
  return s;
}

void AffineTransformation::GetScalingProto(ScalingProto* scaling) const {
  Vec3dToScalingProto(GetScaling(), scaling);
}

void AffineTransformation::Get3x3SVD(Mat3d* u, Mat3d* v, Vec3d* s) const {
  // Eigen's JacobiSVD is numerically accurate and fast for small matrices.
  unsigned int opts = (u != nullptr ? Eigen::ComputeFullU : 0) |
                      (v != nullptr ? Eigen::ComputeFullV : 0);
  const Eigen::JacobiSVD<Mat3d> svd(mat_.block<3, 3>(0, 0), opts);
  if (u != nullptr) *u = svd.matrixU();
  if (v != nullptr) *v = svd.matrixV();
  if (s != nullptr) *s = svd.singularValues();
}

}  // namespace st
