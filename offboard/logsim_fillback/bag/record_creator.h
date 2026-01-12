#pragma once
#include "bag/record.h"

namespace worldview {

class RecordCreator {
 public:
  RecordCreator() = delete;
  static RecordInfo info(const std::string &file_or_dir, bool brief);
  static RecordInfo info(const std::vector<std::string> &files, bool brief);
  ;
};

}  // namespace worldview