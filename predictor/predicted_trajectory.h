

#ifndef ST_PLANNING_PREDICTION_PREDICTED_TRAJECTORY
#define ST_PLANNING_PREDICTION_PREDICTED_TRAJECTORY

#include <algorithm>
#include <iterator>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/vec.h"
#include "plan_common/second_order_trajectory_point.h"
#include "predictor/prediction_defs.h"
#include "modules/cnoa_pnc/planning/proto/prediction.pb.h"
#include "modules/cnoa_pnc/planning/proto/prediction_common.pb.h"

namespace st {
namespace prediction {
class alignas(64) PredictedTrajectoryPoint
    : public planning::SecondOrderTrajectoryPoint {
 public:
  PredictedTrajectoryPoint() = default;
  explicit PredictedTrajectoryPoint(
      const PredictedTrajectoryPointProto& proto) {
    FromProto(proto);
  }
  // Create from a second-order trajectory point where bmv info is lost.
  explicit PredictedTrajectoryPoint(
      const planning::SecondOrderTrajectoryPoint& point)
      : planning::SecondOrderTrajectoryPoint(point) {}

  // Serialization to proto.
  void FromProto(const PredictedTrajectoryPointProto& proto);
  void ToProto(PredictedTrajectoryPointProto* proto) const;

  // template <class Archive>
  // void serialize( Archive & ar) {
  //   ar( cereal::base_class<PredictedTrajectoryPoint>( this ));
  // }
};

class PredictedTrajectory {
 public:
  PredictedTrajectory() {}
  explicit PredictedTrajectory(const PredictedTrajectoryProto& proto) {
    FromProto(/*shift_time=*/0.0, proto);
  }
  // Build trajectory with a given shift time.
  // TODO: Change it to a builder function.
  explicit PredictedTrajectory(double shift_time,
                               const PredictedTrajectoryProto& proto) {
    FromProto(shift_time, proto);
  }

  double probability() const { return probability_; }
  PredictionType type() const { return type_; }
  bool is_reversed() const { return is_reversed_; }
  int index() const { return index_; }
  const std::vector<PredictedTrajectoryPoint>& points() const {
    return points_;
  }
  void set_probability(double probability) { probability_ = probability; }
  void set_index(int index) { index_ = index; }
  void set_type(PredictionType type) { type_ = type; }

  std::vector<PredictedTrajectoryPoint>* mutable_points() { return &points_; }
  TrajectoryIntention intention() const { return intention_; }

  std::string DebugString() const;

  // Serialization to proto.
  // Can only read proto from decompressed message.
  void FromProto(const PredictedTrajectoryProto& proto);
  void ToProto(PredictedTrajectoryProto* proto, bool compress_traj) const;

 private:
  // Construct from proto with a relative timestamp.
  void FromProto(double shift_time, const PredictedTrajectoryProto& proto);

  double probability_ = 0.0;
  PredictionType type_;
  int index_;
  std::vector<PredictedTrajectoryPoint> points_;

  bool is_reversed_ = false;
  TrajectoryIntention intention_ = TrajectoryIntention::INTENTION_UNKNOWN;

  // template <typename Archive>
  // friend void serialize(Archive& ar, PredictedTrajectory&
  // predicted_trajectory);
};

}  // namespace prediction
}  // namespace st

#endif  // ST_PLANNING_PREDICTION_PREDICTED_TRAJECTORY
