

#ifndef ONBOARD_CONTAINER_STRONG_VECTOR_H_
#define ONBOARD_CONTAINER_STRONG_VECTOR_H_

#include <memory>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "plan_common/container/strong_int.h"
#include "plan_common/log.h"

// This macro defines a strong typed vector with uniquely defined index types.
// To use this macro, first define the classes by this macro (see the example in
// strong_vector_test.cc):
// ```
// DECLARE_STRONG_VECTOR(Foo);
// ```
//
// Then you can access a typed-vector `FooVector<Data>` and an index type
// `FooIndex`. The elements of the vector can be accessed by
// `FooVector[FooIndex]`.
//
// You can iterate through the vector with one of the methods:
//
// ```
// for (const auto& data : foo_vector) { ... }
//
// ```
// or
// ```
// for (const auto index : foo_vector.index_range()) { ...foo_vector[index]... }
// ```
// or
// ```
// for (const auto index : foo_vector.index_from(begin)) {
// ...foo_vector[index]...
// }
// ```
// or
// ```
// for (const auto index : foo_vector.index_to(end)) { ...foo_vector[index]... }
// ```
// or
// ```
// for (const auto index : foo_vector.index_range(begin, end)) {
//   ...foo_vector[index]...
// }
// ```
//

#define DECLARE_STRONG_VECTOR(Name)                                         \
  DEFINE_STRONG_INT_TYPE(Name##Index, int);                                 \
  class Name##IndexRange {                                                  \
   public:                                                                  \
    class Iterator : public Name##Index {                                   \
     public:                                                                \
      explicit constexpr Iterator(int i) : Name##Index(i) {}                \
      Name##Index operator*() { return *this; }                             \
    };                                                                      \
    Name##IndexRange(int end) : end_(end) {}                                \
    Name##IndexRange(int begin, int end) : begin_(begin), end_(end) {}      \
    Iterator begin() const { return Iterator(begin_); }                     \
    Iterator end() const { return Iterator(end_); }                         \
                                                                            \
   private:                                                                 \
    int begin_ = 0;                                                         \
    int end_ = 0;                                                           \
  };                                                                        \
  template <class T, class Alloc = std::allocator<T>>                       \
  class Name##Vector {                                                      \
   public:                                                                  \
    static constexpr Name##Index kInvalidIndex = Name##Index(-1);           \
    using value_type = T;                                                   \
    void reserve(int size) { v_.reserve(size); }                            \
    void resize(int size) { v_.resize(size); }                              \
    void shrink_to_fit() { v_.shrink_to_fit(); }                            \
    void push_back(T&& t) { v_.push_back(std::forward<T>(t)); }             \
    void push_back(const T& t) { v_.push_back(t); }                         \
    const T& front() const { return v_.front(); }                           \
    T& front() { return v_.front(); }                                       \
    const T& back() const { return v_.back(); }                             \
    T& back() { return v_.back(); }                                         \
    void clear() { v_.clear(); }                                            \
    template <class... Args>                                                \
    constexpr T& emplace_back(Args&&... args) {                             \
      return v_.emplace_back(std::forward<Args>(args)...);                  \
    }                                                                       \
    template <typename Iter>                                                \
    Iter erase(Iter iter) {                                                 \
      return v_.erase(iter);                                                \
    }                                                                       \
    template <typename Iter>                                                \
    Iter erase(Iter first, Iter last) {                                     \
      return v_.erase(first, last);                                         \
    }                                                                       \
    const T* data() const { return v_.data(); }                             \
    int size() const { return v_.size(); }                                  \
    T& operator[](const Name##Index index) { return v_[index.value()]; }    \
    const T& operator[](const Name##Index index) const {                    \
      return v_[index.value()];                                             \
    }                                                                       \
    Name##IndexRange index_range() const {                                  \
      return Name##IndexRange(v_.size());                                   \
    }                                                                       \
    Name##IndexRange index_from(int begin) const {                          \
      CHECK(begin >= 0) << "Negative begin position!";                      \
      return Name##IndexRange(begin, v_.size());                            \
    }                                                                       \
    Name##IndexRange index_to(int end) const {                              \
      CHECK(end <= v_.size()) << "End position exceeds vector length!";     \
      return Name##IndexRange(0, end);                                      \
    }                                                                       \
    Name##IndexRange index_range(int begin, int end) const {                \
      CHECK(begin >= 0) << "Negative begin position!";                      \
      CHECK(end <= v_.size()) << "End position exceeds vector length!";     \
      return Name##IndexRange(begin, end);                                  \
    }                                                                       \
    bool valid_index(Name##Index index) const {                             \
      return index.value() >= 0 && index.value() < v_.size();               \
    }                                                                       \
    typename std::vector<T, Alloc>::const_iterator begin() const {          \
      return v_.cbegin();                                                   \
    }                                                                       \
    typename std::vector<T, Alloc>::const_iterator end() const {            \
      return v_.cend();                                                     \
    }                                                                       \
    typename std::vector<T, Alloc>::iterator begin() { return v_.begin(); } \
    typename std::vector<T, Alloc>::iterator end() { return v_.end(); }     \
                                                                            \
   private:                                                                 \
    std::vector<T, Alloc> v_;                                               \
  };

#endif  // ONBOARD_CONTAINER_STRONG_VECTOR_H_
