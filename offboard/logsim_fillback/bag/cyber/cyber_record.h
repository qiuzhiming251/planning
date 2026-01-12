#pragma once
#include <atomic>
#include <limits>
#include <map>
#include <string>
#include <thread>

#include "bag/record.h"

namespace worldview {
class CyberSingleRecord {
 public:
  struct TopicInfoCyber {
    TopicInfoCyber(const std::string &n, const std::string &t)
        : name(n), type(t) {}
    std::string name;
    std::string type;
  };

  CyberSingleRecord(const std::string &name, const Filter &topic_filter,
                    const Filter &type_filter);
  const std::string &name() const { return name_; }
  int64_t min_time() const { return min_time_; }
  int64_t max_time() const { return max_time_; }
  void set_min_time(int64_t t) { min_time_ = t; }
  void set_max_time(int64_t t) { max_time_ = t; }

  bool operator<(const CyberSingleRecord &rhs) const {
    return this->min_time_ < rhs.min_time_;
  }

  static bool checkValid(const std::string &);
  const std::map<int64_t, TopicInfoCyber> &getTopicInfo() const {
    return topic_infos_;
  }

 private:
  std::string name_;
  int64_t min_time_ = std::numeric_limits<int64_t>::max();
  int64_t max_time_ = std::numeric_limits<int64_t>::min();
  std::map<int64_t, TopicInfoCyber> topic_infos_;

  Filter topic_filter_;
  Filter type_filter_;
};

class CyberRecord : public Record {
 public:
  CyberRecord(const std::vector<std::string> &files, const Filter &topic_filter,
              const Filter &type_filter, bool dynamic_transform,
              bool dynamic_load, int64_t start_offset, int64_t end_offset);

  void join() {
    if (producing_) {
      producing_ = false;
      produce_thread_.join();
    }
  }

  virtual ~CyberRecord() {
    producing_ = false;
    if (produce_thread_.joinable()) {
      produce_thread_.join();
    }
  }

  virtual int64_t minTime() const { return single_files_.front().min_time(); }
  virtual int64_t maxTime() const { return single_files_.back().max_time(); }
  // std::vector<KeyVal> meta() const override;
  virtual bool loading() const override { return producing_; }

  virtual void rewriteTopics(const std::vector<std::string> &topics,
                             const std::string &filename);

  static bool checkValid(const std::vector<std::string> &files);
  static RecordInfo info(const std::vector<std::string> &files, bool brief);

 private:
  std::vector<CyberSingleRecord> single_files_;
  void produceThreadFunc();

  bool dynamic_load_{false};
  std::atomic_bool producing_{false};
  std::thread produce_thread_;
};
}  // namespace worldview
