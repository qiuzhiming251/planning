#ifndef ST_PLANNING_SPEED_SLT_INFO
#define ST_PLANNING_SPEED_SLT_INFO

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "plan_common/log.h"

//#include "lite/check.h"
//#include "lite/logging.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "plan_common/maps/overlap_info.h"
#include "plan_common/maps/st_boundary.h"
#include "plan_common/maps/st_point.h"
#include "plan_common/maps/vt_point.h"

namespace st::planning {
class SltInfo;

using SltInfoRef = std::unique_ptr<SltInfo>;
class SltInfo {
 public:
  SltInfo(const std::vector<NearestSlPoint>& sl_points, std::string traj_id,
          double t_interval = 0.1, double t_length = 8.0)
      : traj_id_(std::move(traj_id)),
        object_id_(RecoverObjectId(traj_id_)),
        t_length_(t_length) {
    t_interval_ = std::fmax(0.05, t_interval);
    FormatSltInfo(sl_points);
  }

  std::optional<NearestSlPoint> GetSltInfoByt(double t) {
    if (near_sl_points_.size() < 2) {
      return std::nullopt;
    }
    int index = std::clamp(CeilToInt(t / t_interval_), 0,
                           int(near_sl_points_.size()) - 1);
    return near_sl_points_[index];
  }

  const std::string& traj_id() const { return traj_id_; }
  const std::optional<std::string>& object_id() const { return object_id_; }

  StBoundaryProto::DecisionType decision_type() const { return decision_type_; }
  void set_decision_type(StBoundaryProto::DecisionType decision_type) {
    decision_type_ = decision_type;
  }

  StBoundaryProto::DecisionReason decision_reason() const {
    return decision_reason_;
  }
  void set_decision_reason(StBoundaryProto::DecisionReason decision_reason) {
    decision_reason_ = decision_reason;
  }

  StBoundaryProto::IgnoreReason ignore_reason() const { return ignore_reason_; }
  void set_ignore_reason(StBoundaryProto::IgnoreReason ignore_reason) {
    ignore_reason_ = ignore_reason;
  }

  const std::string& decision_info() const { return decision_info_; }
  void set_decision_info(const std::string& str) { decision_info_ = str; }

  bool IsVaild() { return valid_pt_num_ > 2; }

 private:
  void FormatSltInfo(const std::vector<NearestSlPoint>& sl_points);

  std::optional<std::string> RecoverObjectId(const std::string& traj_id);

  std::vector<std::optional<NearestSlPoint>> near_sl_points_;
  int valid_pt_num_ = 0;
  std::string traj_id_;
  std::optional<std::string> object_id_;
  StBoundaryProto::DecisionType decision_type_ = StBoundaryProto::UNKNOWN;
  StBoundaryProto::DecisionReason decision_reason_ =
      StBoundaryProto::UNKNOWN_REASON;
  StBoundaryProto::IgnoreReason ignore_reason_ = StBoundaryProto::NONE;
  std::string decision_info_;

  double t_interval_;
  double t_length_;
};

}  // namespace st::planning

#endif  // ST_PLANNING_SPEED_SLT_INFO
