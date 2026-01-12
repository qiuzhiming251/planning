#include "common/ctrl_util.h"

namespace worldview {
namespace util {
extern const std::map<VehicleType, double> kWheelBase = {
    {VEHICLE_TYPE_X01, 3.1},
    {VEHICLE_TYPE_X02, 3.0},
    {VEHICLE_TYPE_X03, 3.0},
    {VEHICLE_TYPE_X04, 3.1},
    {VEHICLE_TYPE_W01, 3.3}};

extern const double kFrontWheel2Steering = 15.8;
extern const double kSteerAngleMax = 480 * (M_PI / 180.0);
extern const double kSteerAngleMin = -480 * (M_PI / 180.0);
extern const double kSteerAngleRateMax = 500 * (M_PI / 180.0);
extern const double kSteerAngleRateMin = -500 * (M_PI / 180.0);

extern const std::vector<std::pair<double, util::Point2D>> kAccMaxLimit = {
    {0, {2.5, 5.0}},  {1, {2.5, 5.0}},   {3, {2.5, 5.0}},
    {5, {2.4, 5.0}},  {18, {1.8, 5.0}},  {56, {1.05, 5.0}},
    {72, {0.7, 5.0}}, {100, {0.4, 5.0}}, {118, {0.3, 5.0}}};
extern const std::vector<std::pair<double, util::Point2D>> kAccMinLimit = {
    {0, {-5, -2.0}},  {1, {-5, -2.5}},   {3, {-5, -5}},
    {5, {-5, -5}},    {18, {-5, -5.0}},  {56, {-5, -5.0}},
    {72, {-5, -5.0}}, {100, {-5, -5.0}}, {118, {-5, -5.0}}};
extern const std::vector<std::pair<double, double>> kLatAccLimit = {
    {3.0, 300.0}, {1.0, 2000.0}};
}  // namespace util
}  // namespace worldview