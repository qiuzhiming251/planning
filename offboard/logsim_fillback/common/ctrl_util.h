#pragma once

#include <map>
#include <vector>

#include "common/point_2d.h"
#include "common/type_def.h"
namespace worldview {
namespace util {
extern const std::map<VehicleType, double> kWheelBase;

extern const double kFrontWheel2Steering;
extern const double kSteerAngleMax;
extern const double kSteerAngleMin;
extern const double kSteerAngleRateMax;
extern const double kSteerAngleRateMin;

extern const std::vector<std::pair<double, util::Point2D>> kAccMaxLimit;
extern const std::vector<std::pair<double, util::Point2D>> kAccMinLimit;
extern const std::vector<std::pair<double, double>> kLatAccLimit;
}  // namespace util
}  // namespace worldview