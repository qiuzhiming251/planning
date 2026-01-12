#include "bag/cyber/cyber_record.h"
#include <algorithm>
#include "cyber/record/file/record_file_reader.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "common/filesystem.h"
#include "common/log.h"
#include "common/string_util.h"
#include "common/time_util.h"
#include "common/timer.h"

namespace worldview {

bool CyberRecord::checkValid(const std::vector<std::string> &files) {
  if (files.size() == 0) {
    LOG_ERROR << "no file provided";
    return false;
  }

  for (auto &f : files) {
    if (!CyberSingleRecord::checkValid(f)) {
      LOG_DEBUG << f << " is not a valid liviz record";
      return false;
    }
  }
  return true;
}

bool CyberSingleRecord::checkValid(const std::string &file_name) {
  fs::path p(file_name);
  if (!fs::exists(p)) {
    LOG_ERROR << file_name << " does not exist";
    return false;
  }
  if (!fs::is_regular_file(p)) {
    LOG_ERROR << file_name << " is not a regular file";
    return false;
  }
  apollo::cyber::record::RecordFileReader file_reader;
  if (!file_reader.Open(file_name)) {
    file_reader.Close();
    LOG_ERROR << "open record file error. file: " << file_name;
    return false;
  }
  file_reader.Close();
  return true;
}

CyberSingleRecord::CyberSingleRecord(const std::string &file_name,
                                     const Filter &topic_filter,
                                     const Filter &type_filter)
    : name_(file_name), topic_filter_(topic_filter), type_filter_(type_filter) {
  apollo::cyber::record::RecordFileReader file_reader;
  try {
    if (!file_reader.Open(file_name)) {
      throw std::runtime_error("open record file error");
    }
    if (!file_reader.ReadIndex()) {
      throw std::runtime_error("read index error");
    }
    const auto &header = file_reader.GetHeader();
    min_time_ = std::min(min_time_, static_cast<int64_t>(header.begin_time()));
    max_time_ = std::max(max_time_, static_cast<int64_t>(header.end_time()));
    apollo::cyber::proto::Index idx = file_reader.GetIndex();
    for (int i = 0; i < idx.indexes_size(); ++i) {
      apollo::cyber::proto::ChannelCache *cache =
          idx.mutable_indexes(i)->mutable_channel_cache();
      if (idx.mutable_indexes(i)->type() ==
          apollo::cyber::proto::SectionType::SECTION_CHANNEL) {
        if (!topic_filter_.allowed(cache->name())) {
          continue;
        }
        topic_infos_.emplace(
            i, TopicInfoCyber{cache->name(), cache->message_type()});
      }
    }
  } catch (std::exception &e) {
    LOG_ERROR << "cyber file format error: " << e.what();
  }
  file_reader.Close();
}

CyberRecord::CyberRecord(const std::vector<std::string> &files,
                         const Filter &topic_filter, const Filter &type_filter,
                         bool dynamic_transform, bool dynamic_load,
                         int64_t start_offset, int64_t end_offset)
    : Record(RECORD_CYBER, files, topic_filter, type_filter, dynamic_transform),
      dynamic_load_(dynamic_load) {
  int64_t min_time = std::numeric_limits<int64_t>::max();
  int64_t max_time = std::numeric_limits<int64_t>::min();

  for (auto &f : files) {
    single_files_.emplace_back(f, topic_filter, type_filter);
    min_time = std::min(min_time, single_files_.back().min_time());
    max_time = std::max(max_time, single_files_.back().max_time());
  }
  max_time = std::min(max_time, min_time + end_offset);
  min_time = min_time + start_offset;

  for (auto single_file = single_files_.begin();
       single_file != single_files_.end();) {
    if (single_file->min_time() > max_time ||
        single_file->max_time() < min_time) {
      single_file = single_files_.erase(single_file);
    } else {
      single_file->set_min_time(std::max(min_time, single_file->min_time()));
      single_file->set_max_time(std::min(max_time, single_file->max_time()));
      ++single_file;
    }
  }
  std::sort(single_files_.begin(), single_files_.end());

  for (auto &f : single_files_) {
    for (auto &kv : f.getTopicInfo()) {
      insertTopic(kv.second.name, kv.second.type);
    }
    // auto tags = tag(f.conn());
    // tags_.insert(tags_.end(), tags.begin(), tags.end());
  }

  producing_ = true;
  produce_thread_ = std::thread(&CyberRecord::produceThreadFunc, this);
  constexpr int32_t kPreloadSec = 5;
  int32_t preload_secs = kPreloadSec;
  while (producing_ && preload_secs) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    --preload_secs;
  }

  while (producing_ && buffer_->size() == 0) {
    LOG_WARN << "buffer empty after preload time. Give it some more time";
    preload_secs = kPreloadSec;
    while (producing_ && preload_secs && buffer_->size() == 0) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      --preload_secs;
    }
  }

  if (!dynamic_load_) {
    while (producing_) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

RecordInfo CyberRecord::info(const std::vector<std::string> &files,
                             bool brief) {
  RecordInfo info;
  info.type = RECORD_CYBER;
  int64_t max_time = std::numeric_limits<int64_t>::min();
  int64_t min_time = std::numeric_limits<int64_t>::max();
  int total_num = 0;
  std::map<std::string, std::string> topics;
  std::map<std::string, int64_t> topics_start;
  std::map<std::string, int64_t> topics_end;
  std::map<std::string, int64_t> topics_channel_num;

  for (const auto &file : files) {
    using apollo::cyber::record::RecordReader;
    RecordReader reader(file);
    const auto &channel_list = reader.GetChannelList();
    int64_t total = 0;
    for (const auto &channel : channel_list) {
      topics_channel_num[channel] = reader.GetMessageNumber(channel);
      total += topics_channel_num[channel];
      topics[channel] = reader.GetMessageType(channel);
    }
    total_num += total;
    for (int64_t i = 0; i < total; ++i) {
      using apollo::cyber::record::RecordMessage;
      RecordMessage message;
      if (reader.ReadMessage(&message)) {
        int64_t message_time = static_cast<int64_t>(message.time);
        if (topics_start.find(message.channel_name) == topics_start.end()) {
          topics_start[message.channel_name] = message_time;
        } else {
          topics_start[message.channel_name] =
              std::min(topics_start[message.channel_name], message_time);
        }
        if (topics_end.find(message.channel_name) == topics_end.end()) {
          topics_end[message.channel_name] = message_time;
        } else {
          topics_end[message.channel_name] =
              std::max(topics_end[message.channel_name], message_time);
        }
        min_time = std::min(min_time, message_time);
        max_time = std::max(max_time, message_time);
      }
    }
    if (brief) {
      for (auto &t : topics) {
        info.topics_num[t.first] = 0;
      }
    } else {
      for (auto &t : topics_channel_num) {
        info.topics_num[t.first] = t.second;
      }
    }
    // auto tags = tag(f_conn);
    // info.tags.insert(info.tags.end(), tags.begin(), tags.end());
    info.filenames.push_back(file);
  }
  for (const auto &topic : topics) {
    if (info.topics_num[topic.first] > 0) {
      info.topics_empty_front[topic.first] =
          topics_start[topic.first] - min_time;
      info.topics_empty_back[topic.first] = max_time - topics_end[topic.first];
    }
    if (info.topics_num[topic.first] > 1) {
      info.topics_freq[topic.first] =
          (info.topics_num[topic.first] - 1) /
          ((topics_end[topic.first] - topics_start[topic.first]) / 1e9);
    }
  }
  info.total_num = total_num;
  info.max = max_time;
  info.min = min_time;
  info.duration = max_time - min_time;
  LOG_INFO << "max_time:" << max_time << ",min_time:" << min_time
           << ",duration:" << info.duration;
  info.topics = topics;
  return info;
}

void CyberRecord::rewriteTopics(const std::vector<std::string> &topics,
                                const std::string &output_dir) {}

void CyberRecord::produceThreadFunc() {
  util::Timer total_timer;
  for (auto &f : single_files_) {
    LOG_DEBUG << "initializing " << f.name();
    try {
      auto record_reader = std::make_shared<DynamicDataReaderCyber>(f.name());
      if (!record_reader->IsValid()) {
        throw std::runtime_error("record reader is invalid");
      }
      if (!record_reader->GetHeader().is_complete()) {
        throw std::runtime_error("record file is not complete");
      }
      auto channel_list = record_reader->GetChannelList();
      auto record_viewer_ptr =
          std::make_shared<RecordViewerCyber>(record_reader);
      util::Timer timer;
      int count = 0;

      auto itr = record_viewer_ptr->begin();
      auto itr_end = record_viewer_ptr->end();
      for (; itr != itr_end; ++itr) {
        if (!topic_filter_.allowed(itr->channel_name)) {
          continue;
        }
        if (!types_cyber_.count(itr->channel_name) ||
            !type_filter_.allowed(types_cyber_[itr->channel_name])) {
          continue;
        }
        auto raw_msg = std::make_shared<DynamicDataCyber>(itr->content);
        insertMsg(itr->time, itr->channel_name, raw_msg);
        count++;
      }
      buffer_->merge();
      LOG_DEBUG << "one round read " << count << " msgs, took "
                << timer.time_ms() << " ms";
    } catch (std::exception &e) {
      LOG_ERROR << "cyber file format error: " << e.what();
    }
  }
  producing_ = false;
  LOG_DEBUG << "totally took " << total_timer.time_ms() << "ms";
}
}  // namespace worldview