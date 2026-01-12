

#include <sys/time.h>

#include "boost/ptr_container/ptr_vector.hpp"
#include "plan_common/log.h"
#include "plan_common/planning_macros.h"
#include "plan_common/util/logger_stack.h"
#include "plan_common/util/time_logger.h"

namespace ad_byd {
namespace planning {

TimeLogger::TimeLogger(const std::string& logger_name)
    : _logger_name(logger_name), _start_time(GetCurrentTimeStamp()), _total(0) {
  LoggerStack::GetStack()
      .StartSegment(_start_time)
      ->SetName(logger_name + ":total");              // total logger
  LoggerStack::GetStack().StartSegment(_start_time);  // first segment
}

void TimeLogger::ResetStartTime() {
  _start_time = GetCurrentTimeStamp();
  _total = 0.0;
}

void TimeLogger::ChangeSegment(const std::string& old_seg_name,
                               int64_t timestamp) {
  // end old segment and start new one
  LoggerStack::GetStack().EndSegment(timestamp,
                                     old_seg_name);  // end last segment
  LoggerStack::GetStack().StartSegment(timestamp);   // begin new segment
}

// in usec unit
void TimeLogger::RegisterTime(const std::string& time_seg_name) {
  if (_disable) {
    return;
  }
  int64_t end = GetCurrentTimeStamp();
  _total += (end - _start_time);
  _start_time = end;
  ChangeSegment(time_seg_name, end);
}

void TimeLogger::RegisterTimeAndPrint(const std::string& time_seg_name) {
  if (_disable) {
    return;
  }
  RegisterTime(time_seg_name);
  _printed = true;
}

void TimeLogger::SetDisable(bool disable) { _disable = disable; }

int64_t TimeLogger::GetCurrentTimeStamp() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

TimeLogger::~TimeLogger() {
  int64_t end = GetCurrentTimeStamp();

  // if you want to modify this, make sure the call times of start_segment equal
  // to end_segment + abandon_segment.
  LoggerStack::GetStack().EndSegment(
      end, _logger_name + ":end");  // end the last sub segment
  if (_printed) {
    LoggerStack::GetStack().EndSegment(end);  // end the whole logger segment
  } else {
    LoggerStack::GetStack().AbandonSegment();
  }
}

TimeLogger::TimeLogger(const std::string& logger_name,
                       const std::string& father_segment_name)
    : _logger_name(logger_name), _start_time(GetCurrentTimeStamp()), _total(0) {
  LoggerStack& stack = LoggerStack::GetStack();
  stack.StartSegment(_start_time)
      ->SetName(logger_name + ":total");  // total logger
  stack.StartSegment(_start_time);        // first segment
}

}  // namespace planning
}  // namespace ad_byd
