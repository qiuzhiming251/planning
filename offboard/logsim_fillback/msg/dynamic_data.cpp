#include "common/log.h"
#include "msg/dynamic_data.h"
namespace worldview {

MsgInstance::PtrComp MsgInstance::PtrGreater =
    [](const Ptr &lhs, const Ptr &rhs) { return *lhs > *rhs; };
MsgInstance::PtrComp MsgInstance::PtrLess = [](const Ptr &lhs, const Ptr &rhs) {
  return *lhs < *rhs;
};

}  // namespace worldview