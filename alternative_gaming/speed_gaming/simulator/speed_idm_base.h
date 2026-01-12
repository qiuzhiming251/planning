#pragma once
#include "speed_idm_common.h"

namespace st::planning {
struct SpeedIdmConfig {
  double update_coeff = 0.5;  // 与上一帧profile的相关的更新参数
  double speed_limit_look_forward_time = 2.0;
  bool fast_acc_to_dec = false;
  bool fast_dec_to_acc = false;
  bool no_inertial_update_in_first_frame = false;  //这里默认值待定
};
struct IdmInteractiveInfo {
  IdmInteractiveInfo() = default;
  const SpeedVector *agent_speed_profile;
  double agent_to_conflict_zone_dist;
  double agent_to_conflict_zone_time;
  double agent_leave_conflict_zone_dist;
  double agent_leave_conflict_zone_time;
  double ego_to_conflict_zone_dist;
  double ego_to_conflict_zone_time;
  double ego_leave_conflict_zone_dist;
  double ego_leave_conflict_zone_time;
  double ego_cross_angle;
  PlannerObject gaming_object;  // 修正命名空间分隔符
  DiscretizedPath av_path;      // 修正命名空间分隔符
};
class SpeedGamingIdmBase {
 public:
  SpeedGamingIdmBase() = default;
  SpeedGamingIdmBase(const SpeedGamingParams *speed_gaming_params_ptr) {
    speed_gaming_params_ = *speed_gaming_params_ptr;
    CalcStepTime(speed_gaming_params_.time_step);
  }
  ~SpeedGamingIdmBase() {}

  void SetConfig(const SpeedIdmConfig &config) { idm_config_ = config; }
  void SetGamingParams(const SpeedGamingParams &speed_gaming_params) {
    speed_gaming_params_ = speed_gaming_params;
  }

  virtual SpeedVector CalcYieldIdmSpeedVector(
      const SpeedIdmState &init_state, const SpeedVector &origin_speed_data,
      const IdmInteractiveInfo &interactive_info,
      const PiecewiseLinearFunction<double> &speed_limit_plf,
      IdmSimulateState *simulate_state) const = 0;
  virtual SpeedVector CalcPassIdmSpeedVector(
      const SpeedIdmState &init_state, const SpeedVector &origin_speed_data,
      const SpeedVector &upper_speed_bound,
      const SpeedVector &normal_speed_data,
      const IdmInteractiveInfo &interactive_info,
      const PiecewiseLinearFunction<double> &speed_limit_plf,
      IdmSimulateState *simulate_state) const = 0;

  double CalcUpperSpeedBoundAcc(double t, const SpeedIdmState &cur_state,
                                const SpeedVector &upper_speed_bound) const;

  double CalcNormalSpeedTrackAcc(double t, const SpeedIdmState &cur_state,
                                 const SpeedVector &normal_speed) const;

  const SpeedIdmConfig &GetConfig() const { return idm_config_; }

 protected:
  double CalcJerkByAcc(double cur_acc, double target_acc, double plerk) const;

  double CalcInertialTrackJerk(const SpeedVector &origin_speed_data,
                               const SpeedPoint &speed_point, int index) const;

  double CalcInertialTrackAcc(const SpeedVector &origin_speed_data,
                              const SpeedPoint &speed_point) const;

  SpeedIdmState UpdateNextEgoState(const SpeedIdmState &cur_state,
                                   double jerk) const;

  void CalcStepTime(double time_step) {
    time_step_ = time_step;
    time_step2_ = time_step * time_step;
    time_step3_ = time_step2_ * time_step;
  }

  double GetStepTime() const { return time_step_; }

  double GetStepTime2() const { return time_step2_; }

  double GetStepTime3() const { return time_step3_; }

 protected:
  SpeedIdmConfig idm_config_;
  double time_step_ = 0.2;
  double time_step2_ = 0.04;
  double time_step3_ = 0.008;
  SpeedGamingParams speed_gaming_params_;
};  // class
}  // namespace st::planning