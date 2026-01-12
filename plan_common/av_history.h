#pragma once

#include <optional>

#include "boost/circular_buffer.hpp"
#include "modules/cnoa_pnc/planning/proto/positioning.pb.h"

namespace st::planning {

// Much like the `ObjectHistory`, step = 0.1s
class AvHistory {
 public:
  AvHistory(int sz = 100) : kappas_(sz), poses_(sz) {}
  /**
   * latest kappa;
   */
  std::optional<double> GetKappa() const;
  double step_secs() const { return step_secs_; }
  const boost::circular_buffer<PoseProto>& GetAvPoseCache() const {
    return poses_;
  }
  std::optional<double> GetAvKappaCacheAverage() const;

  bool PushPose(const PoseProto& pose);
  static AvHistory* instance() {
    static AvHistory av_history;
    return &av_history;
  }

 private:
  boost::circular_buffer<double> kappas_;
  boost::circular_buffer<PoseProto> poses_;
  double step_secs_ = 0.1;  // secs.

  static constexpr double kMaxKappa = 1 / 5.2;
};

}  // namespace st::planning
