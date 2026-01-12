#include "common/file_util.h"
#include "common/filesystem.h"
#include "common/string_util.h"
namespace worldview {
namespace util {

const std::string kTypeRegular = "regular";
const std::string kTypeDirectory = "directory";

std::string fileType(FileType t) {
  switch (t) {
    case FILE_REGULAR:
      return kTypeRegular;
    case FILE_DIRECTORY:
      return kTypeDirectory;
    default:
      throw std::runtime_error(
          "unexpected type " +
          std::to_string(static_cast<std::underlying_type<FileType>::type>(t)));
  }
}

std::vector<FileItem> listDirectory(const std::string& dir) {
  std::vector<FileItem> items;
  fs::path p(dir);
  if (!fs::exists(p) || !fs::is_directory(p)) {
    throw std::runtime_error(p.string() + " is not a directory");
  }
  for (const auto& entry : fs::directory_iterator(p)) {
    const auto& entry_p = entry.path();
    items.emplace_back(
        fs::is_directory(entry_p) ? FILE_DIRECTORY : FILE_REGULAR,
        entry_p.filename(), fs::absolute(entry_p));
  }
  return items;
}

std::string getRealPath(const std::string& path) {
  std::string res = util::trim(path);
  if (res.length() && res.front() == '~') {
    res.replace(0, 1, getenv("HOME"));
  }
  return res;
}

}  // namespace util
}  // namespace worldview