

#include <ostream>

#include "plan_common/maps/maps_helper.h"

//#include "global/buffered_logger.h"
//#include "lite/check.h"

namespace st::mapping {
namespace {
// curvature.
constexpr double kCurvaturePrecision = 0.00001;
constexpr int kCurvatureMinRange = -5000;
constexpr int kCurvatureMaxRange = 5000;
// heading.
constexpr double kHeadingPrecision = 0.01;
constexpr int kHeadingMinRange = 0;
constexpr int kHeadingMaxRange = 36000;
// slope.
constexpr double kSlopePrecision = 0.1;
constexpr int kSlopeMinRange = -300;
constexpr int kSlopeMaxRange = 300;
// banking.
constexpr double kBankingPrecision = 0.1;
constexpr int kBankingMinRange = -300;
constexpr int kBankingMaxRange = 300;
}  // namespace

// ********************************* shape *********************************
double ScaleToValue(const int scale, const int min_scale, const int max_scale,
                    const double scale_precision) {
  const int tmp_scale = std::clamp(scale, min_scale, max_scale);
  return tmp_scale * scale_precision;
}

int ValueToScale(const double value, const int min_scale, const int max_scale,
                 const double scale_precision) {
  const int scale = value / scale_precision;
  return std::clamp(scale, min_scale, max_scale);
}

double ScaleToCurvature(const int scale) {
  return ScaleToValue(scale, kCurvatureMinRange, kCurvatureMaxRange,
                      kCurvaturePrecision);
}
int CurvatureToScale(const double curvature) {
  return ValueToScale(curvature, kCurvatureMinRange, kCurvatureMaxRange,
                      kCurvaturePrecision);
}
double ScaleToHeading(const int scale) {
  return ScaleToValue(scale, kHeadingMinRange, kHeadingMaxRange,
                      kHeadingPrecision);
}
int HeadingToScale(const double heading) {
  return ValueToScale(heading, kHeadingMinRange, kHeadingMaxRange,
                      kHeadingPrecision);
}
double ScaleToSlope(const int scale) {
  return ScaleToValue(scale, kSlopeMinRange, kSlopeMaxRange, kSlopePrecision);
}
int SlopeToScale(const double slope) {
  return ValueToScale(slope, kSlopeMinRange, kSlopeMaxRange, kSlopePrecision);
}
double ScaleToBanking(const int scale) {
  return ScaleToValue(scale, kBankingMinRange, kBankingMaxRange,
                      kBankingPrecision);
}
int BankingToScale(const double banking) {
  return ValueToScale(banking, kBankingMinRange, kBankingMaxRange,
                      kBankingPrecision);
}

}  // namespace st::mapping
