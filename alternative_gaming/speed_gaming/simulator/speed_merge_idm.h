#pragma once
#include "speed_idm_base.h"

namespace st::planning {

class SpeedMergeIdm : public SpeedGamingIdmBase {
 public:
  SpeedMergeIdm() = default;
  SpeedMergeIdm(const SpeedGamingParams *speed_gaming_params)
      : SpeedGamingIdmBase(speed_gaming_params) {}

  SpeedVector CalcYieldIdmSpeedVector(
      const SpeedIdmState &init_state, const SpeedVector &origin_speed_data,
      const IdmInteractiveInfo &interactive_info,
      const PiecewiseLinearFunction<double> &speed_limit_plf,
      IdmSimulateState *simulate_state) const override;
  SpeedVector CalcPassIdmSpeedVector(
      const SpeedIdmState &init_state, const SpeedVector &origin_speed_data,
      const SpeedVector &upper_speed_bound,
      const SpeedVector &normal_speed_data,
      const IdmInteractiveInfo &interactive_info,
      const PiecewiseLinearFunction<double> &speed_limit_plf,
      IdmSimulateState *simulate_state) const override;
};

}  // namespace st::planning