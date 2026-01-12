

#ifndef ONBOARD_PLANNER_OPTIMIZATION_IPOPT_IPOPT_UTIL_H_
#define ONBOARD_PLANNER_OPTIMIZATION_IPOPT_IPOPT_UTIL_H_

#include <string>

#include "IpReturnCodes.hpp"  // IWYU pragma: keep

#define HAVE_STDDEF_H
#undef HAVE_STDDEF_H

namespace st {
namespace planning {

std::string IpoptReturnStatusToString(Ipopt::ApplicationReturnStatus status);

}  // namespace planning
}  // namespace st
#endif  // ONBOARD_PLANNER_OPTIMIZATION_IPOPT_IPOPT_UTIL_H_
