#pragma once
#include <string>

namespace st::planning {
struct OBBox2d {
  OBBox2d(const Vec2d& center, double heading, double half_length_front,
          double half_length_back, double half_width_left,
          double half_width_right)
      : center_(center),
        heading_(heading),
        half_length_front_(half_length_front),
        half_length_back_(half_length_back),
        half_width_left_(half_width_left),
        half_width_right_(half_width_right) {}

  OBBox2d(const Vec2d& center, double heading, double length, double width)
      : center_(center),
        heading_(heading),
        half_length_front_(0.5 * length),
        half_length_back_(0.5 * length),
        half_width_left_(0.5 * width),
        half_width_right_(0.5 * width) {}

  Vec2d center_;
  double heading_;
  double half_length_front_;
  double half_length_back_;
  double half_width_left_;
  double half_width_right_;
};

struct VehicleState {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double kappa = 0.0;
  double v = 0.0;
  double a = 0.0;
  std::string Debug() const { return "vehicle state"; }
};

struct VehicleAction {
  double target_kappa = 0.0;
  double target_a = 0.0;
  double jerk = 0.0;
  double dkappa = 0.0;
  std::string Debug() const { return "vehicle action"; }
};

struct ActionRange {
  double min_jerk = 0.0;
  double max_jerk = 0.0;
  double min_dkappa = 0.0;
  double max_dkappa = 0.0;
  std::string Debug() const { return "action range"; }
};
}  // namespace st::planning
