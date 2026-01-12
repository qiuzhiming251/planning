#include "acc/acc_output.h"

#include <sstream>

#include "absl/strings/str_join.h"

namespace st::planning {

std::string AccOutput::DebugString() const {
  std::stringstream ss;
  auto to_str_fn = [](std::string* out, const auto& v) {
    return out->append(v.DebugString());
  };
  ss << "acc_path_corridors: [ "
     << absl::StrJoin(acc_path_corridors, ", ", to_str_fn) << "], \n"
     << "traj_points: [ " << absl::StrJoin(traj_points, ", ", to_str_fn)
     << "], \n"
     << ", traj_validation_result: " << traj_validation_result.DebugString();
  return ss.str();
}
}  // namespace st::planning
