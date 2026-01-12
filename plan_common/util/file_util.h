

#ifndef ST_PLANNING_UTIL_FILE_UTIL
#define ST_PLANNING_UTIL_FILE_UTIL

#include <string>
#include <string_view>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "google/protobuf/message.h"

namespace st {
namespace file_util {

// Remove the file specified by fpath.
absl::Status RemoveFile(std::string_view fpath);

// @desc    Create the directory recursively.
// @param   dir   The directory to be created
absl::Status CreateDirectory(std::string_view dir);

// @desc    Remove all of the specified directory.
// @param   dir   The directory to be deleted
absl::Status RemoveDirectory(std::string_view dir);

// @desc    List all files and sub-directories, without . and ..
//          This is the same as the Linux command `ls`, except if 'dir' is not a
//          directory, error is returned. The results are NOT sorted.
// @param   dir   The directory to be listed
absl::StatusOr<std::vector<std::string>> ListDirectory(std::string_view dir);

// @desc    List all files and sub-directories, in a recursive manner.
// @returns vector of relative path w.r.t input 'dir'
absl::StatusOr<std::vector<std::string>> ListDirectoryRecursive(
    std::string_view dir);

// Get the entire file content as a string.
// WARNING! This function is not thread-safe; it cannot be called in parallel.
// For a thread-safe version, use MMapFile.
bool GetFileContent(const std::string& filename, std::string* content);

bool GetFileContentByGetline(const std::string& filename, std::string* content);

// WARNING! This function is not thread-safe; it cannot be called in parallel.
std::string GetFileContentOrDie(const std::string& filename);

// Save a file from the string content.
bool SetFileContent(const std::string& content, const std::string& filename);

void SetFileContentOrDie(const std::string& content,
                         const std::string& filename);

// Get file extension based on filename
std::string GetFileExtension(const std::string& filename);

// Parse string into a proto.
// TODO: This function should be moved to proto_util.h
bool StringToProto(const std::string& proto_string,
                   google::protobuf::Message* proto);

// Parse text (.pb.txt) proto file.
bool TextFileToProto(const std::string& filename,
                     google::protobuf::Message* proto);

// Parse binary (.pb.bin) proto file.
// WARNING! This function is not thread-safe; it cannot be called in parallel.
bool BinaryFileToProto(const std::string& filename,
                       google::protobuf::Message* proto);

// Parse proto file. It accepts text and binary encoding. It is best for the
// case that you are not sure the provided proto file is text or binary file.
bool FileToProto(const std::string& filename, google::protobuf::Message* proto);

// Save proto as text file.
bool ProtoToTextFile(const google::protobuf::Message& proto,
                     const std::string& filename);

// Save proto as binary file.
// WARNING! This function is not thread-safe; it cannot be called in parallel.
bool ProtoToBinaryFile(const google::protobuf::Message& proto,
                       const std::string& filename);

// Save proto as json file.
bool ProtoToJsonFile(const google::protobuf::Message& proto,
                     const std::string& filename);

}  // namespace file_util
}  // namespace st

#endif  // ST_PLANNING_UTIL_FILE_UTIL
