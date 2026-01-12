

#ifndef AD_BYD_PLANNING_UTILS_LOGGER_STACK_H
#define AD_BYD_PLANNING_UTILS_LOGGER_STACK_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "boost/ptr_container/ptr_vector.hpp"

namespace ad_byd {
namespace planning {

class LoggerSegment {
 public:
  LoggerSegment();

  LoggerSegment* AddChild();

  LoggerSegment* SetName(const std::string& name);

  LoggerSegment* SetStartTime(const int64_t& start_time);

  LoggerSegment* SetEndTime(const int64_t& end_time);

  int64_t GetPeriod();

  std::ostream& PrintToJson(std::ostream& os);

  const std::string& name() const;

  uint64_t id() const;

  int64_t start_time() const;

  void Abandon();

  const boost::ptr_vector<LoggerSegment>* children() const {
    return &_children;
  }

  bool is_abandoned() const { return _is_abandoned; }

 private:
  static std::atomic<uint64_t> _s_segment_id;

  boost::ptr_vector<LoggerSegment> _children;
  std::string _name;
  int64_t _start_time = 0;
  int64_t _end_time = 0;
  bool _is_abandoned = false;
  uint64_t _id = 0;
};

class LoggerStack {
 public:
  static LoggerStack& GetStack();

  LoggerSegment* StartSegment(const int64_t& time);

  void EndSegment(const int64_t& time, const std::string& segment_name);

  void EndSegment(const int64_t& time);

  void AbandonSegment();

  LoggerSegment* root_segment();

  // if current_segment is nullptr, return 0
  // else return the id of current stack
  uint64_t current_id() const;

  static void SetThreadFatherId(const uint64_t name);
  static void PrintLoggerStackTrace();

 private:
  void PrintStack();

  static thread_local LoggerStack _stack;

  std::unique_ptr<LoggerSegment> _root_segment;
  uint64_t _father_id = 0;
  std::vector<LoggerSegment*> _current_stack;
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_UTILS_LOGGER_STACK_H
