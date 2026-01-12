#pragma once

#if __cplusplus >= 201703L && __has_include(<filesystem>)

#include <filesystem>
namespace fs = std::filesystem;

#elif __has_include(<experimental/filesystem>)

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

#else

static_assert(false,
              "cpp version too old, no filesystem or experimental::filesystem");

#endif
