

#ifndef ONBOARD_UTILS_FILESYSTEM_H_
#define ONBOARD_UTILS_FILESYSTEM_H_

#if __has_include(<filesystem>)
#include <filesystem>  // IWYU pragma: export
namespace st {
namespace filesystem = std::filesystem;
}
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>  // // IWYU pragma: export
namespace st {
namespace filesystem = std::experimental::filesystem;
}
#else
error "Could not find system header <filesystem> or <experimental/filesystem>"
#endif

#endif
