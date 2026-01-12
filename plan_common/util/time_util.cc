

#include "plan_common/util/time_util.h"

namespace st {

absl::Time FromProto(const st::planning::Timestamp& proto) {
  return absl::FromUnixSeconds(proto.seconds()) +
         absl::Nanoseconds(proto.nanos());
}

void ToProto(absl::Time time, st::planning::Timestamp* proto) {
  const int64_t s = absl::ToUnixSeconds(time);
  proto->set_seconds(s);
  proto->set_nanos((time - absl::FromUnixSeconds(s)) / absl::Nanoseconds(1));
}

}  // namespace st
