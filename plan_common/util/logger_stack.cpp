

#include <atomic>

#include "plan_common/log.h"
#include <sys/time.h>

#include "boost/ptr_container/ptr_vector.hpp"
#include "plan_common/log.h"
#include "plan_common/planning_macros.h"
#include "plan_common/util/logger_stack.h"

namespace ad_byd {
namespace planning {

std::atomic<uint64_t> LoggerSegment::_s_segment_id(0);

LoggerSegment::LoggerSegment() { _id = ++_s_segment_id; }

LoggerSegment* LoggerSegment::AddChild() {
  _children.push_back(new LoggerSegment);
  return &_children.back();
}

LoggerSegment* LoggerSegment::SetName(const std::string& name) {
  _name = name;
  return this;
}

LoggerSegment* LoggerSegment::SetStartTime(const int64_t& start_time) {
  _start_time = start_time;
  return this;
}

LoggerSegment* LoggerSegment::SetEndTime(const int64_t& end_time) {
  _end_time = end_time;
  return this;
}

int64_t LoggerSegment::GetPeriod() { return _end_time - _start_time; }

int64_t LoggerSegment::start_time() const { return _start_time; }

std::ostream& LoggerSegment::PrintToJson(std::ostream& os) {
  os << "{";
  os << "\"name\": \"" << _name << "\", ";
  os << "\"id\": " << _id << ", ";
  os << "\"period\": " << GetPeriod();
  if (!_children.empty()) {
    bool have_child = false;
    for (auto& child : _children) {
      if (!child.is_abandoned()) {
        have_child = true;
        break;
      }
    }
    if (have_child) {
      os << ", \"children\": [";
      std::string delim = "";
      for (auto& child : _children) {
        if (!child.is_abandoned()) {
          os << delim;
          delim = ",";
          child.PrintToJson(os);
        }
      }
      os << "]";
    }
  }
  os << "}";
  return os;
}

const std::string& LoggerSegment::name() const { return _name; }

uint64_t LoggerSegment::id() const { return _id; }

void LoggerSegment::Abandon() { _is_abandoned = true; }

LoggerStack& LoggerStack::GetStack() { return _stack; }

LoggerSegment* LoggerStack::StartSegment(const int64_t& time) {
  if (_current_stack.empty()) {
    _root_segment.reset(new LoggerSegment);
    _root_segment->SetStartTime(time);
    _current_stack.push_back(_root_segment.get());
  } else {
    LoggerSegment* child =
        _current_stack.back()->AddChild()->SetStartTime(time);
    _current_stack.push_back(child);
  }
  return _current_stack.back();
}

void LoggerStack::EndSegment(const int64_t& time,
                             const std::string& segment_name) {
  CHECK(!_current_stack.empty());
  _current_stack.back()->SetName(segment_name)->SetEndTime(time);
  _current_stack.pop_back();
  if (_current_stack.empty()) {
    PrintStack();
  }
}

void LoggerStack::EndSegment(const int64_t& time) {
  CHECK(!_current_stack.empty());
  CHECK(!_current_stack.back()->name().empty());
  _current_stack.back()->SetEndTime(time);
  _current_stack.pop_back();
  if (_current_stack.empty()) {
    PrintStack();
    SetThreadFatherId(
        0);  // should clean up father id, because of the thread pool
  }
}

void LoggerStack::AbandonSegment() {
  CHECK(!_current_stack.empty());
  _current_stack.back()->Abandon();
  _current_stack.pop_back();
}

void LoggerStack::PrintStack() {
  if (_root_segment->is_abandoned()) {
    return;
  }

  std::stringstream ss;
  ss << "{\"father\": " << _father_id << ", \"start_time\": ";
  ss << _root_segment->start_time() << ", \"segments\": ";
  _root_segment->PrintToJson(ss);
  ss << "}";
  LERROR("%s", ss.str().c_str());
}

LoggerSegment* LoggerStack::root_segment() { return _root_segment.get(); }

uint64_t LoggerStack::current_id() const {
  return _current_stack.empty() ? 0 : _current_stack.back()->id();
}

void LoggerStack::SetThreadFatherId(const uint64_t id) {
  _stack._father_id = id;
}

thread_local LoggerStack LoggerStack::_stack;

void LoggerStack::PrintLoggerStackTrace() {
  std::stringstream ss;
  ss << "=================LoggerStackTrace Start=================\n";
  int cnt = 0;
  for (auto* segment : _stack._current_stack) {
    ss << "#" << cnt++ << ": " << segment->name() << "\n";
  }
  ss << "=================LoggerStackTrace End=================\n";
  LERROR("%s", ss.str().c_str());
}

}  // namespace planning
}  // namespace ad_byd
