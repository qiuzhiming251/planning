#include "bag/record_creator.h"

#include <set>

#include "bag/cyber/cyber_record.h"
#include "common/filesystem.h"
#include "common/log.h"

namespace worldview {

RecordInfo RecordCreator::info(const std::string &file_or_dir, bool brief) {
  return info(std::vector<std::string>{file_or_dir}, brief);
}

RecordInfo RecordCreator::info(const std::vector<std::string> &files,
                               bool brief) {
  if (files.size() == 0) {
    throw std::invalid_argument("no files provided");
  }
  if (files.size() == 1 && fs::is_directory(files[0])) {
    return info(files[0], brief);
  }
  if (CyberRecord::checkValid(files)) {
    return CyberRecord::info(files, brief);
  } else {
    std::string concat;
    for (auto &f : files) {
      concat += " " + f;
    }
    throw std::invalid_argument("unrecognized record files: " + concat);
  }
}

}  // namespace worldview