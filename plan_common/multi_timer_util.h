

#ifndef ONBOARD_PLANNER_COMMON_MULTI_TIMER_UTIL_H_
#define ONBOARD_PLANNER_COMMON_MULTI_TIMER_UTIL_H_

#include <string>

//#include "global/timer.h"
//#include "timer_report.pb.h"
//#include "plan_common/util/source_location.h"
namespace st::planning {

void GetMultiTimerReport(const ScopedMultiTimer& timer,
                         MultiTimerReportProto* report_proto);

std::string PrintMultiTimerReport(const MultiTimerReportProto& report_proto);

void PrintMultiTimerReportStat(const ScopedMultiTimer& timer,
                               SourceLocation loc = LOC);
void PrintMultiTimerReportStat(const MultiTimerReportProto& report_proto);

void CombineMultiTimerProtos(MultiTimerReportProto* proto_1,
                             const MultiTimerReportProto& proto_2);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_COMMON_MULTI_TIMER_UTIL_H_
