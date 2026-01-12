

#ifndef ONBOARD_PLANNER_SPEED_ST_CLOSE_TRAJECTORY_H_
#define ONBOARD_PLANNER_SPEED_ST_CLOSE_TRAJECTORY_H_

#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"

namespace st::planning {

class StCloseTrajectory {
 public:
  struct StNearestPoint {
    double s = 0.0;
    double t = 0.0;
    // Relative to nearest path_segment direction.
    double v = 0.0;
    double lat_dist = 0.0;
    int obj_idx = 0;
    double lat_t = 0.0;
    std::string DebugString() const {
      return absl::StrCat("s: ", s, ", t: ", t, ", v: ", v,
                          " lat_dist: ", lat_dist, ", obj_idx: ", obj_idx);
    }
  };

  // The input 'st_nearest_points' must sort by t.
  StCloseTrajectory(std::vector<StNearestPoint> st_nearest_points,
                    std::string id, std::string traj_id, std::string object_id,
                    StBoundaryProto::ObjectType object_type, double probability,
                    bool is_stationary);

  std::optional<StNearestPoint> GetNearestPointByTime(double time) const;

  absl::Span<const StNearestPoint> st_nearest_points() const {
    return st_nearest_points_;
  }

  const std::string& id() const { return id_; }

  const std::string& traj_id() const { return traj_id_; }

  const std::string& object_id() const { return object_id_; }

  StBoundaryProto::ObjectType object_type() const { return object_type_; }

  double probability() const { return probability_; }

  bool is_stationary() const { return is_stationary_; }

  double min_t() const { return min_t_; }

  double max_t() const { return max_t_; }

  double min_s() const { return min_s_; }

  double max_s() const { return max_s_; }

 private:
  void Init();

  std::vector<StNearestPoint> st_nearest_points_;
  std::string id_;
  std::string traj_id_;
  std::string object_id_;
  StBoundaryProto::ObjectType object_type_;
  bool is_stationary_;
  double probability_ = 0.0;

  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();
  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SPEED_ST_CLOSE_TRAJECTORY_H_
