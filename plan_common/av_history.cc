#include "av_history.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include "plan_common/math/vec.h"
namespace st::planning {
namespace {
constexpr double kMinSpeedMps = 1.0;  // m/s.
}

std::optional<double> AvHistory::GetKappa() const {
  return kappas_.empty() ? std::nullopt : std::make_optional(kappas_.back());
}

bool AvHistory::PushPose(const PoseProto& pose) {
  if (poses_.empty() ||
      pose.timestamp() >= poses_.back().timestamp() + step_secs()) {
    // Should not use pose to calc kappa.
    constexpr double kSpeedEpsilon = 1E-6;
    auto v = std::max(std::abs(pose.vel_body().x()), kSpeedEpsilon);
    double kappa = std::clamp(pose.ar_smooth().z() / v, -kMaxKappa, kMaxKappa);
    kappas_.push_back(kappa);
    poses_.push_back(pose);
    return true;
  }
  return false;
}

std::optional<double> AvHistory::GetAvKappaCacheAverage() const {
  if (kappas_.empty()) {
    return std::nullopt;
  }
  int size = std::distance(kappas_.rbegin(), kappas_.rend());
  // Only choose latest 5 secs
  auto count = std::min(size, 50);
  double sum = std::accumulate(kappas_.rbegin(), kappas_.rbegin() + count, 0.0);
  return sum / count;
}

}  // namespace st::planning
