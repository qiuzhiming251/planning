
#include <fstream>

#include <fcntl.h>
#include <sys/time.h>
#include <unistd.h>

#include "plan_common/log.h"
#include "plan_common/util/utility.h"

namespace ad_byd {
namespace planning {

bool Utility::GetProtoFromBinaryFile(const std::string& filename,
                                     google::protobuf::Message* const message) {
  std::fstream input(filename, std::ios::in | std::ios::binary);
  if (!input.good()) {
    LWARN("Fail to open file: %s", filename.c_str());
    return false;
  }
  if (!message->ParseFromIstream(&input)) {
    LWARN("Fail to parse file: %s", filename.c_str());
    return false;
  }
  return true;
}

bool Utility::GetProtoFromAsciiFile(const std::string& filename,
                                    google::protobuf::Message* const message) {
  int file_descriptor = open(filename.c_str(), O_RDONLY);
  if (file_descriptor < 0) {
    LWARN("Fail to open file: %s, errno=%d", filename.c_str(), errno);
    return false;
  }
  google::protobuf::io::ZeroCopyInputStream* input =
      new google::protobuf::io::FileInputStream(file_descriptor);
  bool success = google::protobuf::TextFormat::Parse(input, message);
  if (!success) {
    LWARN("Fail to parse file: %s", filename.c_str());
  }
  delete input;
  close(file_descriptor);
  return success;
}

bool Utility::GetProtoFromFile(const std::string& filename,
                               google::protobuf::Message* const message) {
  std::string bin = ".bin";
  bool is_bin = filename.length() >= bin.length() &&
                filename.compare(filename.length() - bin.length(), bin.length(),
                                 bin) == 0;
  if (is_bin) {
    if (!GetProtoFromBinaryFile(filename, message) &&
        !GetProtoFromAsciiFile(filename, message)) {
      return false;
    }
  } else {
    if (!GetProtoFromAsciiFile(filename, message) &&
        !GetProtoFromBinaryFile(filename, message)) {
      return false;
    }
  }
  return true;
}

double Utility::GetCurrentTime() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  const double us2s = 1000000.0;  // microcond
  const double timestamp = tv.tv_sec * us2s + tv.tv_usec;
  return timestamp / us2s;
}

}  // namespace planning
}  // namespace ad_byd