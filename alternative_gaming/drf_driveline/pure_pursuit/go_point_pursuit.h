#pragma once
#include "plan_common/math/discretized_path.h"
#include "../vehicle_state.h"
#include <string>
namespace st::planning {
struct PurePursuitInfo {
  bool succ_projected = false;
  double pp_look_ahead_ds = 0.0;
  double pp_look_ahead_s = 0.0;
  double av_proj_s_on_path = 0.0;
  double pp_target_kappa = 0.0;
  double pp_target_arg_length = 0.0;
  double pp_target_curv_speed_limit = 0.0;
  double pp_target_acc = 0.0;
  double pp_dkappa = 0.0;
  double pp_jerk = 0.0;
  double pp_virtual_time = 0.0;
};

class RfPurePursuit {
 public:
  static void AddFrictionCircleConstraint(double cur_v, double target_kappa,
                                          double* target_acc);
  static PurePursuitInfo CalcPathTrackingAction(
      const DiscretizedPath& reference, const VehicleState& ego_state,
      double target_point_s, int init_index, double lat_acc_max,
      std::string* debug);

 private:
  static double CalcTargetDkappa(const VehicleState& av_state,
                                 double target_kappa, double virtual_time);
  static std::pair<PathPoint, bool> CalcProjPoint(
      const DiscretizedPath& reference, const PathPoint& p, int init_index);
};
}  // namespace st::planning
