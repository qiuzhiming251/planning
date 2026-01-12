

#ifndef AD_BYD_PLANNING_UTILS_TIME_LOGGER_H
#define AD_BYD_PLANNING_UTILS_TIME_LOGGER_H

#include <chrono>
#include <string>

namespace ad_byd {
namespace planning {

class TimeLogger {
 public:
  TimeLogger(const std::string& logger_name);
  TimeLogger(const std::string& logger_name,
             const std::string& father_segment_name);

  // reset the start time with current time
  void ResetStartTime();

  // regist current time as end time, and record it with a name, then the start
  // time is current time
  void RegisterTime(const std::string& time_seg_name);

  // regist and output the record info to log
  void RegisterTimeAndPrint(const std::string& time_seg_name);

  // get wall-time(or real time) in micro second
  static int64_t GetCurrentTimeStamp();

  void SetDisable(bool disable);

  ~TimeLogger();

 private:
  void ChangeSegment(const std::string& old_seg_name, int64_t timestamp);

  std::string _logger_name;
  int64_t _start_time;
  int64_t _total;
  bool _disable = false;
  bool _printed = false;
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_UTILS_TIME_LOGGER_H
