

#include <algorithm>
#include <cerrno>
#include <memory>

#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "plan_common/util/file_util_mmap.h"

//#include "global/buffered_logger.h"
//#include "lite/check.h"
#include "plan_common/util/status_macros.h"

namespace st::file_util {
absl::StatusOr<ScopedFileDescriptor> OpenFile(const std::string& filename,
                                              int flags,
                                              std::optional<mode_t> mode) {
  int fd = (mode == std::nullopt) ? ::open(filename.c_str(), flags)
                                  : ::open(filename.c_str(), flags, mode);
  if (fd < 0) {
    return absl::PermissionDeniedError(
        absl::StrFormat("Failed to open %s: %s", filename, strerror(errno)));
  }
  ScopedFileDescriptor s_fd(new int(fd), [](int* fd) {
    ::close(*fd);
    delete fd;
  });
  return s_fd;
}

absl::StatusOr<struct stat> GetFileStat(const std::string& filename) {
  struct stat statbuf;
  if (fstatat(AT_FDCWD, filename.c_str(), &statbuf, 0) < 0) {
    return absl::InternalError(
        absl::StrFormat("Failed to stat: %s", strerror(errno)));
  }
  return statbuf;
}

absl::StatusOr<struct stat> GetFileStat(int fd) {
  struct stat statbuf;
  if (fstat(fd, &statbuf) < 0) {
    return absl::InternalError(
        absl::StrFormat("Failed to stat fd: %s", strerror(errno)));
  }
  return statbuf;
}

MMapFile::MMapFile(const std::string& filename, bool writable) {
  status_ = CreateMMap(filename, writable);
}

absl::Status MMapFile::CreateMMap(const std::string& filename, bool writable) {
  ASSIGN_OR_RETURN(const auto s_fd,
                   OpenFile(filename, writable ? O_RDWR : O_RDONLY));
  ASSIGN_OR_RETURN(struct stat statbuf, GetFileStat(*s_fd));
  file_size_ = statbuf.st_size;
  if (file_size_ == 0) {
    return absl::CancelledError(absl::StrCat("Empty file: ", filename));
  }
  const int mode = (writable ? (PROT_READ | PROT_WRITE) : PROT_READ);
  mmap_ptr_ = mmap(nullptr, statbuf.st_size, mode, MAP_SHARED, *s_fd, 0);
  char_ptr_ = static_cast<char*>(mmap_ptr_);
  if (mmap_ptr_ == MAP_FAILED) {  // NOLINT
    return absl::InternalError(
        absl::StrFormat("Failed to mmap %s: %s", filename, strerror(errno)));
  }
  return absl::OkStatus();
}

std::string_view MMapFile::GetFileContentViewOrDie() const {
  // CHECK_OK(status_);
  return std::string_view(char_ptr_, file_size_);
}

std::string MMapFile::GetFileContentOrDie() const {
  // CHECK_OK(status_);
  return std::string(char_ptr_, char_ptr_ + file_size_);
}

bool MMapFile::GetFileContentView(std::string_view* content) const {
  if (!status_.ok()) return false;
  // *CHECK_NOTNULL(content) = std::string_view(char_ptr_, file_size_);
  return true;
}

bool MMapFile::GetFileContent(std::string* content) const {
  if (!status_.ok()) return false;
  // *CHECK_NOTNULL(content) = std::string(char_ptr_, char_ptr_ + file_size_);
  return true;
}

void MMapFile::Close() {
  if (mmap_ptr_ != MAP_FAILED) {  // NOLINT
    munmap(mmap_ptr_, file_size_);
    status_ = absl::UnavailableError("MMap is closed");
    mmap_ptr_ = MAP_FAILED;  // NOLINT
    file_size_ = 0;
  }
}

absl::Status MMapFile::FileAllocate(const std::string& filename, size_t size,
                                    bool maybe_truncate) {
  ASSIGN_OR_RETURN(const auto s_fd,
                   OpenFile(filename, O_RDWR | O_APPEND | O_CREAT, 0644));
  ASSIGN_OR_RETURN(struct stat statbuf, GetFileStat(*s_fd));

  if (statbuf.st_size < size) {
    if (posix_fallocate(*s_fd, 0, size) != 0) {
      return absl::InternalError(absl::StrFormat(
          "Failed to posix_fallocate %s: %s", filename, strerror(errno)));
    }
  } else if (statbuf.st_size > size) {
    if (!maybe_truncate) {
      return absl::PermissionDeniedError(
          absl::StrFormat("%s exists and is larger than %d, Force shrink by "
                          "invoke FileAllocate(filename, size, true)",
                          filename, size));
    }
    if (ftruncate(*s_fd, size) != 0) {
      return absl::InternalError(
          absl::StrFormat("ftruncate %s: %s", filename, strerror(errno)));
    }
  }
  return absl::OkStatus();
}

absl::Status MMapFile::DumpTo(const std::string& filename) {
  RETURN_IF_ERROR(status_);
  MMapFile dest_file(filename, /*writable=*/true);
  RETURN_IF_ERROR(dest_file.status());
  if (size() != dest_file.size()) {
    return absl::InternalError(absl::StrFormat(
        "File size mismatch: src(%d) vs. dest(%d)", size(), dest_file.size()));
  }
  ::memcpy(dest_file.data(), data(), size());
  return absl::OkStatus();
}

absl::Status MMapFile::DumpTo(
    const std::string& filename, size_t step,
    const std::function<void(size_t, size_t)>& callback) {
  RETURN_IF_ERROR(status_);
  MMapFile dest_file(filename, /*writable=*/true);
  RETURN_IF_ERROR(dest_file.status());
  if (size() != dest_file.size()) {
    return absl::InternalError(absl::StrFormat(
        "File size mismatch: src(%d) vs. dest(%d)", size(), dest_file.size()));
  }
  for (size_t offset = 0; offset < size(); offset += step) {
    const size_t copy_end = std::min(size(), offset + step);
    const size_t copy_size = copy_end - offset;
    ::memcpy(dest_file.char_ptr_ + offset, char_ptr_ + offset, copy_size);
    callback(copy_end, size());
  }
  return absl::OkStatus();
}
}  // namespace st::file_util
