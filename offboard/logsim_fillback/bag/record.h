#pragma once
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "bag/msg_buffer.h"
#include "common/time_util.h"
#include "msg/dynamic_data.h"
#include "msg/behavior.h"

namespace worldview {

enum RecordType { RECORD_UNKNOWN = 0, RECORD_CYBER = 1 };

struct KeyVal {
  KeyVal(const std::string &k, const std::string &v) : key(k), val(v) {}
  std::string key, val;
};

struct Point2D {
  Point2D(double x_, double y_) : x(x_), y(y_) {}
  double x{0}, y{0};
};

struct RecordTag {
  int64_t stamp_ms;
  std::string name;
  std::string description;

  Json::Value toJson() const {
    Json::Value tag_j;
    tag_j["name"] = name;
    tag_j["stampMs"] = static_cast<Json::Int64>(stamp_ms);
    tag_j["description"] = description;
    return tag_j;
  }
};

struct RecordInfo {
  Json::Value toJson() const;

  std::vector<RecordTag> tags;
  RecordType type{RECORD_UNKNOWN};

  std::vector<std::string> filenames;
  std::map<std::string, std::string> extra;
  int64_t duration{0};
  int64_t min{std::numeric_limits<int64_t>::max()};
  int64_t max{std::numeric_limits<int64_t>::min()};
  int total_num{0};
  std::map<std::string, std::string> topics;
  std::map<std::string, int> topics_num;
  std::map<std::string, int64_t> topics_empty_front;
  std::map<std::string, int64_t> topics_empty_back;
  std::map<std::string, double> topics_freq;

  std::set<std::string> horizon_packs;
};

class Record {
 public:
  using Ptr = std::shared_ptr<Record>;
  using iterator = MsgBuffer::iterator;

  virtual ~Record() = default;

  virtual bool loading() const { return false; }

  // Record information
  virtual int64_t minTime() const = 0;
  virtual int64_t maxTime() const = 0;
  size_t size() const { return buffer_->size(); }
  double duration() const { return (maxTime() - minTime()) / 1000000000.0; }

  const std::vector<std::string> &getFiles() const { return files_; }
  std::map<std::string, std::string> getTopicsCyber() const {
    return types_cyber_;
  }
  const std::map<std::string, std::string> &getCyberTypes() const {
    return types_cyber_;
  }

  const std::vector<RecordTag> &getTags() const { return tags_; }
  RecordInfo info();

  void addTag(int64_t stamp_s, const std::string &name,
              const std::string &desc);
  bool checkTopicFields(const std::string &topic_name,
                        const std::string &fields) const;

  // iteration
  iterator begin() { return buffer_->begin(); }
  iterator end() { return buffer_->end(); }
  iterator floor(int64_t time) { return buffer_->floor(time); }

  // per topic iteration
  bool hasTopic(const std::string &topic) { return buffer_->hasTopic(topic); }
  size_t size(const std::string &topic) { return buffer_->size(topic); }
  iterator begin(const std::string &topic) { return buffer_->begin(topic); }
  iterator end(const std::string &topic) { return buffer_->end(topic); }
  iterator floor(const std::string &topic, int64_t time) {
    return buffer_->floor(topic, time);
  }

  // load process reporter
  virtual void onLoadProgress(int64_t time) const {}

  // rewrite
  virtual void rewriteTopics(const std::vector<std::string> &topics,
                             const std::string &output_dir) {}

  sim::FunctionId noaType();
  Json::Value getRawStringJsonCyber(const MsgInstance::Ptr &msg) const;
  std::string getRawDebugStringCyber(const MsgInstance::Ptr &msg) const;
  RecordType getRecodeType() const { return type_; }

 protected:
  Record(RecordType t, const std::vector<std::string> &files,
         const Filter &topic_filter, const Filter &type_filter,
         bool dynamic_transform)
      : type_(t),
        buffer_(new MsgBuffer()),
        files_(files),
        dynamic_transform_(dynamic_transform),
        topic_filter_(topic_filter),
        type_filter_(type_filter) {}

  void insertTopic(const std::string &topic_name, const std::string &type);
  void insertMsg(int64_t time, const std::string &topic_name,
                 const DynamicDataCyberPtr &sample);

  RecordType type_;
  MsgBuffer::Ptr buffer_;
  std::map<std::string, std::string> types_cyber_;
  std::map<std::string, std::string> original_types_cyber_;
  std::map<std::string, std::string> topics_;
  std::vector<RecordTag> tags_;
  std::set<std::string> should_adapt_types_;
  std::vector<std::string> files_;

  bool dynamic_transform_{true};

  Filter topic_filter_;
  Filter type_filter_;
};

}  // namespace worldview