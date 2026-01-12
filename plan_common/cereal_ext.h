

#ifndef AD_BYD_PLANNING_COMMON_CEREAL_EXT_H
#define AD_BYD_PLANNING_COMMON_CEREAL_EXT_H

#include <memory>
#include <string>

#include "cereal/types/memory.hpp"
#include "cereal/types/polymorphic.hpp"

namespace cereal {
namespace detail {

// TODO: multiple compiler support
// We make use of gcc specific macro: __PRETTY_FUNCTION__, it only print
// function name. If T is int, then __PRETTY_FUNCTION__ returns something likes
// "void pretty_type_name<T>() [with T = int]". We extract the real type name
// 'int' and return.
template <typename T>
const std::string pretty_type_name() {
  const char *begin_ptr = __PRETTY_FUNCTION__;
  // skip after typename `T'
  bool found = false;
  bool preceeding_space = false;
  while (!found && '\0' != *begin_ptr) {
    switch (*begin_ptr) {
      case ' ':
        preceeding_space = true;
        break;
      case 'T':
        if (preceeding_space && (' ' == *(begin_ptr + 1))) {
          found = true;
        }
        // fall through
      default:
        preceeding_space = false;
    }
    ++begin_ptr;  // skip `T' itself if found
  }
  // skip whitespace after typename `T'
  while (' ' == *begin_ptr) {
    ++begin_ptr;
  }
  // skip `=' sign
  if ('=' == *begin_ptr) {
    ++begin_ptr;
  }
  // skip whitespace after `='
  while (' ' == *begin_ptr) {
    ++begin_ptr;
  }
  const char *end_ptr = begin_ptr;
  while (']' != *end_ptr && ';' != *end_ptr && '\0' != *end_ptr) {
    ++end_ptr;
  }
  return std::string(begin_ptr, end_ptr);
}

template <typename T>
const std::string unique_type_name() {
  // Usually pretty_type_name will return unique name for each type.
  // However, if two cc files all define types with same name inside
  // unnamed namespace, they will conflict each other.
  std::string type = pretty_type_name<T>();
  static const char unnamed_prefix[] = "_anoymous_";
  size_t start_index = type.find(unnamed_prefix);
  while (start_index != std::string::npos) {
    type = type.replace(start_index,
                        sizeof(unnamed_prefix) - 1,  // skip remain tailing "\0"
                        __BASE_FILE__);              // replace with filename
    start_index = type.find(unnamed_prefix);
  }
  return type;
}

}  // namespace detail
}  // namespace cereal

#define CEREAL_REGISTER_TEMPLATE(TEMPLATE_NAME)                      \
  namespace cereal {                                                 \
  namespace detail {                                                 \
                                                                     \
  template <typename T>                                              \
  struct binding_name<TEMPLATE_NAME<T>> {                            \
    static char const *name() {                                      \
      static std::string ret = unique_type_name<TEMPLATE_NAME<T>>(); \
      return ret.c_str();                                            \
    }                                                                \
  };                                                                 \
                                                                     \
  template <typename T>                                              \
  struct init_binding<TEMPLATE_NAME<T>> {                            \
    static bind_to_archives<TEMPLATE_NAME<T>> const &b;              \
    static void unused() { (void)b; }                                \
  };                                                                 \
  template <typename T>                                              \
  bind_to_archives<TEMPLATE_NAME<T>> const                           \
      &init_binding<TEMPLATE_NAME<T>>::b =                           \
          ::cereal::detail::StaticObject<                            \
              bind_to_archives<TEMPLATE_NAME<T>>>::getInstance()     \
              .bind();                                               \
  }                                                                  \
  }

#define CEREAL_REGISTER_TEMPLATE_CLASS_INSTANTIATION(T) \
  do {                                                  \
    if (0) {                                            \
      cereal::detail::init_binding<T>::unused();        \
    }                                                   \
  } while (0)

template <typename T, typename Archive, typename Enable = void>
struct serde_wrapper_impl {
  static void serialize(T &value, Archive &ar) {}
};

template <typename T, typename Archive>
struct serde_wrapper_impl<
    T, Archive,
    typename std::enable_if<std::enable_if<
        Archive::is_saving::value, cereal::traits::is_output_serializable<
                                       T, Archive>>::type::value>::type> {
  static void serialize(T &value, Archive &ar) { ar(value); }
};

template <typename T, typename Archive>
struct serde_wrapper_impl<
    T, Archive,
    typename std::enable_if<std::enable_if<
        Archive::is_loading::value, cereal::traits::is_input_serializable<
                                        T, Archive>>::type::value>::type> {
  static void serialize(T &value, Archive &ar) { ar(value); }
};

template <typename T, typename Archive, typename Enable = void>
struct serde_wrapper {
  static void serialize(T &value, Archive &ar) {
    serde_wrapper_impl<T, Archive>::serialize(value, ar);
  }
};

template <typename T, typename Archive>
struct serde_wrapper<std::shared_ptr<T>, Archive> {
  static void serialize(std::shared_ptr<T> &value, Archive &ar) {}
};

template <typename T, typename Archive>
void try_serialize(T &value, Archive &archive) {
  serde_wrapper<T, Archive>::serialize(value, archive);
}

#endif  // AD_BYD_PLANNING_COMMON_CEREAL_EXT_H
