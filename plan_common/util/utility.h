

#ifndef AD_BYD_PLANNING_UTILS_UTILITY_H
#define AD_BYD_PLANNING_UTILS_UTILITY_H
#include <algorithm>
#include <fstream>
#include <iomanip>

#include <fcntl.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include "google/protobuf/message.h"

namespace ad_byd {
namespace planning {
class Utility {
 public:
  Utility() = default;
  ~Utility() = default;
  static bool GetProtoFromBinaryFile(const std::string& filename,
                                     google::protobuf::Message* const message);
  static bool GetProtoFromAsciiFile(const std::string& filename,
                                    google::protobuf::Message* const message);
  static bool GetProtoFromFile(const std::string& filename,
                               google::protobuf::Message* const message);
  static double GetCurrentTime();
};
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_UTILS_UTILITY_H