#include <array>
#include <cmath>
#include <string>
#include "plan_common/math/util.h"
namespace st::planning {

struct DynamicState {
  double x{0.0};
  double y{0.0};
  double heading{0.0};
  double curvature{0.0};
  double steer{0.0};
  double v{0.0};
  double lon_acc{0.0};
  double lon_jerk{0.0};
};

struct Control {
  double steer_rate{0.0};
  double acc{0.0};
};

class VehicleKinematicModel {
 public:
  static constexpr size_t STATE_NUM{5};
  static constexpr size_t CHANGED_DOT_NUM_IN_RK4{3};
  enum StateIdx { X = 0, Y = 1, THETA = 2, DELTA = 3, V = 4 };

 public:
  VehicleKinematicModel() = default;
  ~VehicleKinematicModel() = default;
  static void CalNextState(const double wheel_base, const Control &control,
                           const double dt, DynamicState *state,
                           const double v) {
    if (state == nullptr) {
      return;
    }
    const double x = state->x;
    const double y = state->y;
    const double theta = state->heading;
    const double delta = state->steer;
    const double L = wheel_base;
    const double gamma = control.steer_rate;
    const double a = control.acc;
    const auto x_t = CalRK4Kinematic(x, y, theta, delta, v, gamma, a, dt, L);
    auto ret = ToState(x_t);

    state->x = x_t[X];
    state->y = x_t[Y];
    state->heading = x_t[THETA];
    state->steer = x_t[DELTA];
    state->v = x_t[V];
    state->curvature = std::tan(ret.steer) / L;
    state->lon_acc = a;
    state->lon_jerk = (a - state->lon_acc) / dt;
  }

  static inline DynamicState ToState(const std::array<double, STATE_NUM> &x_t) {
    return {x_t[X], x_t[Y], x_t[THETA], 0.0, x_t[DELTA], x_t[V], 0.0, 0.0};
  }

  static std::array<double, STATE_NUM> CalRK4Kinematic(
      const double x, const double y, const double theta, const double delta,
      const double v, const double gamma, const double a, const double dt,
      const double L) {
    std::array<double, CHANGED_DOT_NUM_IN_RK4> k1{}, k2{}, k3{}, k4{};
    k1.fill(0), k2.fill(0), k3.fill(0), k4.fill(0);
    k1[X] = v * std::cos(theta);
    k1[Y] = v * std::sin(theta);
    k1[THETA] = v * tan(delta) / L;

    const double v1 = v + dt * a * 0.5;
    const double theta1 = theta + dt * 0.5 * k1[THETA];
    const double delta1 = delta + dt * 0.5 * gamma;
    k2[X] = v1 * std::cos(theta1);
    k2[Y] = v1 * std::sin(theta1);
    k2[THETA] = v1 * tan(delta1) / L;

    const double v2 = v1;
    const double theta2 = theta + dt * 0.5 * k2[THETA];
    k3[X] = v2 * std::cos(theta2);
    k3[Y] = v2 * std::sin(theta2);
    k3[THETA] = k2[THETA];

    const double v3 = v + dt * a;
    const double theta3 = theta + dt * k3[THETA];
    const double delta3 = delta + dt * gamma;
    k4[X] = v3 * std::cos(theta3);
    k4[Y] = v3 * std::sin(theta3);
    k4[THETA] = v3 * tan(delta3) / L;

    std::array<double, STATE_NUM> x_t{x, y, theta, delta3, v3};
    x_t[X] += ((k1[X] + 2 * k2[X] + 2 * k3[X] + k4[X]) * dt / 6);
    x_t[Y] += ((k1[Y] + 2 * k2[Y] + 2 * k3[Y] + k4[Y]) * dt / 6);
    x_t[THETA] +=
        ((k1[THETA] + 2 * k2[THETA] + 2 * k3[THETA] + k4[THETA]) * dt / 6);
    return x_t;
  }
};

}  // namespace st::planning