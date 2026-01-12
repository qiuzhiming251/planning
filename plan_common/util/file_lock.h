

#ifndef ONBOARD_UTILS_FILE_LOCK_H_
#define ONBOARD_UTILS_FILE_LOCK_H_

#include <fstream>
#include <optional>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "boost/interprocess/sync/file_lock.hpp"
#include "boost/interprocess/sync/scoped_lock.hpp"
#include "plan_common/util/file_util.h"
#include "plan_common/util/filesystem.h"
#include "plan_common/util/status_macros.h"

namespace st {
// create a scoped file lock to prevent data race between processes.
// Note: does not prevent data race between threads.
// Usage: ScopedFileLock lock("/tmp/abc.lock");
//        CHECK_OK(lock.Lock());
class ScopedFileLock {
 public:
  explicit ScopedFileLock(std::string lock_file)
      : lock_file_(std::move(lock_file)) {}
  absl::Status Lock() {
    // create an empty file
    filesystem::path parent_dir = filesystem::path(lock_file_).parent_path();
    RETURN_IF_ERROR(file_util::CreateDirectory(parent_dir.string()));
    std::ofstream ofs(lock_file_);
    if (!ofs) {
      return absl::PermissionDeniedError(lock_file_);
    }
    ofs.close();

    file_lock_.emplace(lock_file_.c_str());
    scoped_lock_.emplace(*file_lock_);
    return absl::OkStatus();
  }

 private:
  std::string lock_file_;
  using file_lock = boost::interprocess::file_lock;
  std::optional<file_lock> file_lock_;
  std::optional<boost::interprocess::scoped_lock<file_lock>> scoped_lock_;
};

// If you intend to delete an object via DestroyContainerAsync() and still want
// to lock a file before delete (because the delete operation is heavy). You can
// do the following: DestroyContainerAsync(LockFileAndDelete(LOCK_FILE_NAME,
// ptr));
template <typename T>
class LockFileAndDelete {
 public:
  LockFileAndDelete(std::string lock_filename, T* ptr)
      : lock_filename_(std::move(lock_filename)), ptr_(ptr) {}
  LockFileAndDelete(LockFileAndDelete&& other) {
    this->lock_filename_ = std::move(other.lock_filename_);
    this->ptr_ = other.ptr_;
    other.ptr_ = nullptr;
  }
  ~LockFileAndDelete() {
    if (ptr_ == nullptr) return;
    ScopedFileLock file_lock(lock_filename_);
    file_lock.Lock().IgnoreError();
    delete ptr_;
  }

 private:
  std::string lock_filename_;
  T* ptr_;
};
}  // namespace st

#endif  // ONBOARD_UTILS_FILE_LOCK_H_
