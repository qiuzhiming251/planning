#include "planner/speed_optimizer/slt_info.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
// IWYU pragma: no_include <type_traits>

#include <algorithm>
#include <cmath>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "plan_common/log_data.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/util.h"

namespace st::planning {

void SltInfo::FormatSltInfo(const std::vector<NearestSlPoint>& sl_points) {
  near_sl_points_.clear();
  if (sl_points.size() < 2) {
    return;
  }
  for (double t = 0.0; t < t_length_ + 1e-3; t += t_interval_) {
    if (t < sl_points.front().t) {
      near_sl_points_.push_back(std::nullopt);
    } else if (t > sl_points.back().t) {
      near_sl_points_.push_back(std::nullopt);
    } else {
      const auto it = std::lower_bound(
          sl_points.begin(), sl_points.end(), t,
          [](const NearestSlPoint& p, double t) { return p.t < t; });
      if (it == sl_points.begin() || it == sl_points.end()) {
        near_sl_points_.push_back(std::nullopt);
        continue;
      }
      NearestSlPoint near_pt;
      const auto it_pre = it - 1;
      const double factor = LerpFactor(it_pre->t, it->t, t);
      near_pt.av_heading =
          LerpAngle(it_pre->av_heading, it->av_heading, factor);
      near_pt.av_s = Lerp(it_pre->av_s, it->av_s, factor);
      near_pt.lat_dist = Lerp(it_pre->lat_dist, it->lat_dist, factor);
      near_pt.obj_heading =
          LerpAngle(it_pre->obj_heading, it->obj_heading, factor);
      near_pt.obj_v = Lerp(it_pre->obj_v, it->obj_v, factor);
      near_pt.obj_vl = Lerp(it_pre->obj_vl, it->obj_vl, factor);
      near_pt.obj_idx = Lerp(it_pre->obj_idx, it->obj_idx, factor);
      near_pt.t = t;
      near_sl_points_.push_back(std::move(near_pt));
      valid_pt_num_++;
    }
  }
  return;
}

std::optional<std::string> SltInfo::RecoverObjectId(
    const std::string& traj_id) {
  if (const auto found = traj_id.find("-idx"); found == std::string::npos) {
    LOG_ERROR << "traj_id of st_object [" << traj_id
              << "] should contain \"-idx\" but not!";
    return std::nullopt;
  } else {
    return traj_id.substr(0, found);
  }
}
}  // namespace st::planning
