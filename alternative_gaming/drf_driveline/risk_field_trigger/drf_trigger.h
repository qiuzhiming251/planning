#pragma once
#include "../drf_keyobj_decider/keyobj_decider.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "object_manager/object_history.h"
#include "modules/cnoa_pnc/planning/proto/gaming.pb.h"

namespace st::planning {
struct TriggerConfig {
  int rf_change_follow_car_mode_count_thresh = 5;
};
class RiskFieldTrigger {
 public:
  RiskFieldTrigger() = default;
  ~RiskFieldTrigger() = default;
  bool TriggerRiskField(const std::vector<RiskFieldKeyobj>& drf_keyobj,
                        const ObjectHistoryManager& obs_history,
                        const DiscretizedPath& init_path,
                        const DrivelineResultProto* last_driveline_result,
                        DrivelineResultProto* const cur_driveline_result,
                        std::unordered_set<std::string>* yield_obj_ids);

  bool TriggerLtRiskFieldDriveline(
      const std::vector<RiskFieldKeyobj>& drf_keyobj,
      const std::vector<RiskFieldKeyobj>& syn_go_straight_keycyclists,
      const ObjectHistoryManager& obs_history, const DiscretizedPath& init_path,
      const DrivelineResultProto* last_driveline_result,
      DrivelineResultProto* const cur_driveline_result,
      std::unordered_set<std::string>* yield_obj_ids);

 private:
  TriggerConfig config_;
  bool CheckKeyObjLeave(const RiskFieldKeyobj& key_obj,
                        const DiscretizedPath& init_path) const;
  bool CheckLeftKeyobjBufferEnough(const RiskFieldKeyobj& key_obj,
                                   const DiscretizedPath& init_path) const;
};

}  // namespace st::planning
