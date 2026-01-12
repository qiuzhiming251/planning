#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "speed_gaming_common.h"
#include "plan_common/log_data.h"
#include "plan_common/speed/st_speed/speed_vector.h"
#include "modules/cnoa_pnc/planning/proto/gaming.pb.h"

namespace st {
namespace planning {
void DumpSimResultGraph(std::string& group_name, const std::string& obj_id,
                        const GamingSimResult& sim_result);
void DumpConflictZoneGraph(std::string& group_name, std::string& key,
                           const GamingConflictZone& sim_conflict_zone);
void DumpConflictZoneGraph(std::string& group_name, const std::string& key,
                           const Log2DDS::Color& color,
                           const GamingConflictZone& sim_conflict_zone,
                           const LonGamingDecisionType decision_type);

void DumpGamingSpeedVectorGraph(std::string& group_name, const std::string& key,
                                const Log2DDS::Color& color,
                                const SpeedVector& speed_vector);

// void DumpVtGraphSpeedLimit(int plan_id, const SpeedFinderDebugProto&
// speed_finder_proto)
std::string ConvertEgoSpeedData2Log(const SpeedVector& speed_data);
std::string ConvertObjSpeedData2Log(const SpeedVector& speed_data);
std::string ConvertEgoSpeedDataAndObjSpeedData2Log(
    const GamingSimResult& sim_result);
std::string ConvertObjData2Log(const GamingSimResult& sim_result);
std::string ConvertEgoData2Log(const GamingSimResult& sim_result);
std::pair<double, double> calcObsCooperationGrad(
    GamingSimResult* yield_simu, GamingSimResult* pass_simu, std::string obs_id,
    SpacetimeObjectTrajectory* pred_traj,
    std::unordered_map<std::string, ObjectCooperationInfo>&
        object_cooperation_maps,
    std::string* cur_debug_info, IdmSimulateState* simulate_state);
}  // namespace planning
}  // namespace st
