

#include "predictor/predicted_trajectory.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <iterator>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"

//#include "lite/check.h"
//#include "lite/logging.h"
#include "plan_common/math/geometry/util.h"
//#include "predictor/prediction_message_compressor.h"

namespace st {
namespace prediction {

void PredictedTrajectoryPoint::FromProto(
    const PredictedTrajectoryPointProto& proto) {
  set_pos(Vec2dFromProto(proto.pos()));
  set_s(proto.s());
  set_theta(proto.theta());
  set_kappa(proto.kappa());

  set_t(proto.t());
  set_v(proto.v());
  set_a(proto.a());
}

void PredictedTrajectoryPoint::ToProto(
    PredictedTrajectoryPointProto* proto) const {
  Vec2dToProto(pos(), proto->mutable_pos());
  proto->set_s(s());
  proto->set_theta(theta());
  proto->set_kappa(kappa());
  proto->set_t(t());
  proto->set_v(v());
  proto->set_a(a());
}

std::string PredictedTrajectory::DebugString() const {
  return absl::StrFormat("type:%s(%d),prob:%7.6f,size:%d,is_reversed:%d",
                         PredictionType_Name(type_), type_, probability_,
                         points_.size(), is_reversed_);
}

void PredictedTrajectory::FromProto(const PredictedTrajectoryProto& proto) {
  FromProto(/*shift_time=*/0.0, proto);
}

void PredictedTrajectory::FromProto(double shift_time,
                                    const PredictedTrajectoryProto& proto) {
  // The proto message is supposed to have been decompressed by
  // DecompressObjectsPredictionProto
  // CHECK_EQ(proto.compressed_points().x_size(), 0);
  probability_ = proto.probability();
  type_ = proto.type();
  index_ = proto.index();
  is_reversed_ = proto.is_reversed();
  intention_ = proto.intention();
  points_.reserve(proto.points_size());
  if (proto.points_size() > 0) {
    if (type_ == PT_STATIONARY) {
      const auto& pt = proto.points(0);
      for (int i = 0; i < kPredictionPointNum; ++i) {
        points_.emplace_back();
        points_.back().FromProto(pt);
        points_.back().set_t(i * kPredictionTimeStep);
      }
    } else {
      int i = 0;
      while (i < proto.points_size() && proto.points(i).t() < shift_time) {
        ++i;
      }
      for (int j = i;
           j < proto.points_size() && points_.size() < kPredictionPointNum;
           ++j) {
        auto& pt = points_.emplace_back(proto.points(j));
        pt.set_t(pt.t() - proto.points(i).t());
        pt.set_s(pt.s() - proto.points(i).s());
      }
    }
  }
}

void PredictedTrajectory::ToProto(PredictedTrajectoryProto* proto,
                                  bool compress_traj) const {
  proto->Clear();
  proto->set_probability(probability_);
  proto->set_type(type_);
  proto->set_index(index_);
  // proto->set_predicted_channel(predicted_channel_);
  proto->set_is_reversed(is_reversed_);

  // If object is stationary, only send out one point.
  // Planning module needs to reconstruct the full traj.
  if (!points_.empty()) {
    if (type_ == PT_STATIONARY) {
      points_.front().ToProto(proto->add_points());
    } else {
      for (const auto& point : points_) {
        point.ToProto(proto->add_points());
      }
    }
  }
}

}  // namespace prediction
}  // namespace st
