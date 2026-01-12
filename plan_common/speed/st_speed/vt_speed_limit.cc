

#include "plan_common/speed/st_speed/vt_speed_limit.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <memory>

//#include "lite/check.h"

namespace st::planning {

void MergeVtSpeedLimit(const VtSpeedLimit& source, VtSpeedLimit* target) {
  // CHECK_NOTNULL(target);
  if (source.size() != target->size()) return;
  for (int i = 0; i < source.size(); ++i) {
    if (source[i].speed_limit < (*target)[i].speed_limit) {
      (*target)[i] = source[i];
    }
  }
  return;
}

}  // namespace st::planning
