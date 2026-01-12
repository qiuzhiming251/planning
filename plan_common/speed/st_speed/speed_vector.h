

#ifndef ONBOARD_PLANNER_SPEED_SPEED_VECTOR_H_
#define ONBOARD_PLANNER_SPEED_SPEED_VECTOR_H_

#include <memory>
#include <optional>
#include <vector>

#include "plan_common/speed/st_speed/speed_point.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"

namespace st::planning {

class SpeedVector : public std::vector<SpeedPoint> {
 public:
  SpeedVector() = default;

  explicit SpeedVector(std::vector<SpeedPoint> speed_points);

  std::optional<SpeedPoint> EvaluateByTime(double t) const;

  std::optional<SpeedPoint> EvaluateByTimeWithExtrapolation(double t) const;

  std::optional<SpeedPoint> EvaluateByS(double s,
                                        bool extend_backward = false) const;

  double TotalTime() const;

  // Assuming spatial traversed distance is monotonous
  double TotalLength() const;

  // Use container as a template to make it compatible with protobuf's repeated
  // field and vector.
  template <template <class...> class Container>
  void FromProto(const Container<SpeedPointProto>& speed_points) {
    reserve(speed_points.size());
    for (const auto& speed_point : speed_points) {
      emplace_back().FromProto(speed_point);
    }
    std::stable_sort(begin(), end(),
                     [](const SpeedPoint& p1, const SpeedPoint& p2) {
                       return p1.t() < p2.t();
                     });
  }

  void ToProto(SpeedPointsProto* speed_points) const {
    speed_points->Clear();
    for (auto it = begin(); it != end(); ++it) {
      it->ToProto(speed_points->add_speed_points());
    }
  }
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SPEED_SPEED_VECTOR_H_
