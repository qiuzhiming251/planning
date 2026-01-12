#include <dlfcn.h>
#include <functional>
#include <memory>
#include <string>
namespace worldview {
namespace util {
class LibWrapper {
 public:
  using deleter_t = std::function<int(void*)>;
  LibWrapper(const std::string& name, int mode = RTLD_LAZY)
      : ptr_(dlopen(name.c_str(), mode),
             [](void* p) -> int { return dlclose(p); }) {
    if (!ptr_) {
      std::string load_err;
      auto err = dlerror();
      if (err) {
        load_err = std::string(err);
      }
      throw std::invalid_argument("failed to open so " + name + ": " +
                                  load_err);
    }
  }

  void* Get() const { return ptr_.get(); }

  operator void*() const { return Get(); }
  operator bool() const { return Get(); }

  template <typename FP>
  FP GetFunc(const std::string& name) {
    return reinterpret_cast<FP>(dlsym(Get(), name.c_str()));
  }

 private:
  std::unique_ptr<void, deleter_t> ptr_;
};
}  // namespace util
}  // namespace worldview