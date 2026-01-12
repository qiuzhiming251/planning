#include "acc/acc_path_corridor.h"

#include <sstream>
#include <string>

namespace st::planning {

std::string AccPathCorridor::DebugString() const {
  std::stringstream ss;
  auto to_str_fn = [](std::string* out, const auto& v) {
    return out->append(v.DebugString());
  };
  ss << "build_status: " << build_status.ToString()
     << "type: " << AccPathCorridorType_Name(type) << ", boundary.center_xy: ["
     << absl::StrJoin(boundary.reference_center_xy_vector(), ", ", to_str_fn)
     << "], \n"
     << ", loaded_map_dist: " << loaded_map_dist << ", \n"
     << "kappa_s:{ x:[ " << absl::StrJoin(kappa_s.x(), ", ") << "], y:[ "
     << absl::StrJoin(kappa_s.y(), ", ") << "]";
  return ss.str();
}

AccPathCorridor BuildErrorAccPathCorridorResult(std::string_view error_msg) {
  return {
      .build_status = PlannerStatus(PlannerStatusProto::ACC_CORRIDOR_FAILED,
                                    std::string(error_msg)),
  };
}

}  // namespace st::planning
