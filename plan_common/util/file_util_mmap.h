

#ifndef ONBOARD_UTILS_FILE_UTIL_MMAP_H_
#define ONBOARD_UTILS_FILE_UTIL_MMAP_H_

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include <fcntl.h>  // IWYU pragma: keep
#include <sys/mman.h>
#include <sys/stat.h>   // IWYU pragma: keep
#include <sys/types.h>  // IWYU pragma: keep

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "plan_common/base/singleton.h"

namespace st::file_util {
using ScopedFileDescriptor = std::unique_ptr<int, void (*)(int*)>;
// A C++ wrapper around POSIX ::open(), added two features:
// - RAII, no need to worry about when to ::close()
// - More elaborate error message
absl::StatusOr<ScopedFileDescriptor> OpenFile(
    const std::string& filename, int flags,
    std::optional<mode_t> mode = std::nullopt);

// Can be used to get meta info of files, such as file type, size, atime, mtime,
// etc. c.f. `man 2 fstat`
// Basically an absl::StatusOr wrapped POSIX ::fstat()
absl::StatusOr<struct stat> GetFileStat(const std::string& filename);
absl::StatusOr<struct stat> GetFileStat(int fd);

class MMapFile {
 public:
  explicit MMapFile(const std::string& filename, bool writable = false);
  ~MMapFile() { Close(); }
  bool GetFileContent(std::string* content) const;
  std::string GetFileContentOrDie() const;
  bool GetFileContentView(std::string_view* content) const;
  std::string_view GetFileContentViewOrDie() const;
  void Close();
  absl::Status status() const { return status_; }
  size_t size() const { return file_size_; }
  void* data() { return mmap_ptr_; }
  // Pre-requisition: The destination file must exists, writable, having the
  // same size. To make the destination file meet the requirements, use
  // MMapFile::FileAllocate(filename, size(), /*maybe_truncate=*/true);
  absl::Status DumpTo(const std::string& filename);
  // Dump 'step' bytes each time and invoke the callback. This is useful for
  // copying large files
  absl::Status DumpTo(const std::string& filename, size_t step,
                      const std::function<void(size_t, size_t)>& callback);
  // Allocate disk storage for the file. Three possible actions:
  // - file not exists: create the file and allocate given size
  // - file exists, size less or equal to the size given: expand to the size
  // - file exists, size greater than the given size: shrink if 'maybe_truncate'
  // is true, elsewise do nothing.
  static absl::Status FileAllocate(const std::string& filename, size_t size,
                                   bool maybe_truncate = false);

 private:
  void* mmap_ptr_ = MAP_FAILED;
  char* char_ptr_ = nullptr;
  absl::Status status_;
  size_t file_size_ = 0;
  absl::Status CreateMMap(const std::string& filename, bool writable);
  DISALLOW_COPY_AND_ASSIGN(MMapFile);
};
}  // namespace st::file_util

#endif  //  ONBOARD_UTILS_FILE_UTIL_MMAP_H_
