#pragma once

#include <string>
#include <vector>
namespace worldview {
namespace util {
enum FileType { FILE_REGULAR = 0, FILE_DIRECTORY = 1 };

std::string fileType(FileType t);

struct FileItem {
  FileItem(FileType type_, const std::string& name_,
           const std::string& full_name_)
      : type(type_), name(name_), full_name(full_name_) {}
  FileType type;
  std::string name;
  std::string full_name;
};

std::vector<FileItem> listDirectory(const std::string& dir);

std::string getRealPath(const std::string& path);
}  // namespace util
}  // namespace worldview