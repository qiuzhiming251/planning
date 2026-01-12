

#include "plan_common/util/file_util.h"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <memory>
#include <system_error>

#include "absl/strings/str_cat.h"
#include "plan_common/log.h"
#include "google/protobuf/io/zero_copy_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/message.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/json_util.h"

//#include "global/buffered_logger.h"
//#include "lite/check.h"
#include "plan_common/util/filesystem.h"  // IWYU pragma: keep
// IWYU pragma: no_include <filesystem>

namespace st {
namespace file_util {

bool GetFileContent(const std::string& filename, std::string* content) {
  CHECK(content != nullptr);
  std::ifstream fin(filename.c_str(), std::ios::binary | std::ios::in);
  if (!fin.is_open() || !fin.good()) {
    return false;
  }
  *content = std::string((std::istreambuf_iterator<char>(fin)),
                         std::istreambuf_iterator<char>());
  return true;
}

bool GetFileContentByGetline(const std::string& filename,
                             std::string* content) {
  CHECK(content != nullptr);
  std::ifstream fin(filename.c_str(), std::ios::binary | std::ios::in);
  if (!fin.is_open() || !fin.good()) {
    return false;
  }
  std::string line;
  while (std::getline(fin, line)) {
    *content += line;
    *content += "\n";
    line.clear();
  }
  return true;
}

std::string GetFileContentOrDie(const std::string& filename) {
  std::string content;
  bool success = GetFileContent(filename, &content);
  CHECK(success);
  return content;
}

bool SetFileContent(const std::string& content, const std::string& filename) {
  std::ofstream fout(filename.c_str(), std::ios::binary | std::ios::out);
  if (!fout.good()) {
    return false;
  }
  fout.write(content.data(), content.size());
  return true;
}

void SetFileContentOrDie(const std::string& content,
                         const std::string& filename) {
  bool success = SetFileContent(content, filename);
  CHECK(success);
}

std::string GetFileExtension(const std::string& filename) {
  const auto pos = filename.rfind('.');
  if (pos == std::string::npos) {
    return std::string();
  } else {
    // E.g., 1) a.txt => txt 2) a.pb.txt => txt
    return filename.substr(pos + 1);
  }
}

absl::Status RemoveFile(std::string_view filepath) {
  std::error_code ec;
  const bool success = filesystem::remove(filepath, ec);
  // According to https://en.cppreference.com/w/cpp/filesystem/remove
  // returns true if the file was deleted, false if it did not exist.
  if (success || ec.value() == 0) {
    return absl::OkStatus();
  } else {
    return absl::UnknownError(ec.message());
  }
}

absl::Status CreateDirectory(std::string_view dir) {
  std::error_code ec;
  const bool success = filesystem::create_directories(dir, ec);
  if (success || ec.value() == 0) {
    return absl::OkStatus();
  } else {
    return absl::UnknownError(ec.message());
  }
}

absl::Status RemoveDirectory(std::string_view dir) {
  std::error_code ec;
  const auto count = filesystem::remove_all(dir, ec);
  if (count == static_cast<std::uintmax_t>(-1)) {  // on error
    return absl::UnknownError(ec.message());
  } else {
    return absl::OkStatus();
  }
}

absl::StatusOr<std::vector<std::string>> ListDirectory(std::string_view dir) {
  if (!filesystem::is_directory(dir)) {
    return absl::NotFoundError(
        absl::StrCat(std::string(dir), " is not a directory"));
  }
  std::vector<std::string> result;
  for (const auto& entry :
       filesystem::directory_iterator(filesystem::path(dir))) {
    result.push_back(entry.path().filename().string());
  }
  return result;
}

absl::StatusOr<std::vector<std::string>> ListDirectoryRecursive(
    std::string_view dir) {
  if (!filesystem::is_directory(dir)) {
    return absl::NotFoundError(
        absl::StrCat(std::string(dir), " is not a directory"));
  }
  std::vector<std::string> result;
  const auto base_path = filesystem::path(dir);
  for (const auto& entry :
       filesystem::recursive_directory_iterator(base_path)) {
    result.push_back(filesystem::relative(entry.path(), base_path));
  }
  return result;
}

bool StringToProto(const std::string& proto_string,
                   google::protobuf::Message* proto) {
  if (!google::protobuf::TextFormat::ParseFromString(proto_string, proto)) {
    return false;
  }
  return true;
}

bool TextFileToProto(const std::string& file_name,
                     google::protobuf::Message* message) {
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::ZeroCopyInputStream;
  int fd = open(file_name.c_str(), O_RDONLY);
  if (fd < 0) {
    LOG_ERROR << "Failed to open " << file_name
              << " in text mode: " << std::strerror(errno);
    return false;
  }

  std::unique_ptr<ZeroCopyInputStream> input =
      std::make_unique<FileInputStream>(fd);
  bool success = google::protobuf::TextFormat::Parse(input.get(), message);
  if (!success) {
    LOG_ERROR << "Failed to parse " << file_name << " as text proto.";
    ::close(fd);
    return false;
  }

  ::close(fd);
  return success;
}

bool BinaryFileToProto(const std::string& filename,
                       google::protobuf::Message* proto) {
  std::ifstream fin(filename.c_str(), std::ios::binary | std::ios::in);
  if (!fin.is_open()) {
    return false;
  }
  if (!proto->ParseFromIstream(&fin)) {
    return false;
  }
  return true;
}

bool FileToProto(const std::string& filename,
                 google::protobuf::Message* proto) {
  return BinaryFileToProto(filename, proto) || TextFileToProto(filename, proto);
}

bool ProtoToTextFile(const google::protobuf::Message& proto,
                     const std::string& filename) {
  std::string content;
  google::protobuf::TextFormat::PrintToString(proto, &content);
  if (!SetFileContent(content, filename)) {
    return false;
  }
  return true;
}

bool ProtoToBinaryFile(const google::protobuf::Message& proto,
                       const std::string& filename) {
  std::ofstream fout(filename.c_str(), std::ios::binary | std::ios::out);
  if (!fout.is_open()) {
    return false;
  }
  if (!proto.SerializeToOstream(&fout)) {
    return false;
  }
  return true;
}

bool ProtoToJsonFile(const google::protobuf::Message& proto,
                     const std::string& filename) {
  std::string content;

  google::protobuf::util::JsonPrintOptions options;
  options.add_whitespace = true;
  options.always_print_primitive_fields = true;
  options.preserve_proto_field_names = true;
  google::protobuf::util::MessageToJsonString(proto, &content, options);

  if (!SetFileContent(content, filename)) {
    return false;
  }
  return true;
}

}  // namespace file_util
}  // namespace st
