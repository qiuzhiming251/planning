

#ifndef AD_BYD_PLANNING_COMMON_STATUS_BASE_H
#define AD_BYD_PLANNING_COMMON_STATUS_BASE_H

#include <glog/logging.h>

#include <map>
#include <mutex>

#include "cereal/archives/json.hpp"
#include "cereal/types/memory.hpp"
#include "cereal_ext.h"
#include "plan_common/log.h"

namespace ad_byd {
namespace planning {

class StatusBase {
 public:
  struct BaseInfo {
    virtual ~BaseInfo() {}
    virtual BaseInfo* clone() const { return new BaseInfo(); }
    template <typename Archive>
    void serialize(Archive& ar) {}
  };

  struct InfoMap {
    typedef std::map<std::type_index, std::shared_ptr<BaseInfo>> Map;

    template <typename Archive>
    void save(Archive& ar) const {
      ar(static_cast<uint32_t>(data.size()));
      for (auto kv : data) {
        ar(kv.second);
      }
    }

    template <typename Archive>
    void load(Archive& ar) {
      uint32_t size;
      ar(size);
      for (uint32_t i = 0; i != size; ++i) {
        std::shared_ptr<BaseInfo> p;
        ar(p);
        data[std::type_index(typeid(*p))] = p;
      }
    }

    InfoMap(const InfoMap& other);

    InfoMap(InfoMap&& other);

    InfoMap() = default;

    InfoMap& operator=(const InfoMap& other);

    InfoMap& operator=(InfoMap&& other);

    Map data;
  };

  template <typename T>
  struct InfoProxy : public BaseInfo {
    T info;
    virtual BaseInfo* clone() const {
      InfoProxy* proxy = new InfoProxy<T>();
      proxy->info = this->info;
      return proxy;
    }
    template <class Archive>
    void serialize(Archive& ar) {
      ar(cereal::base_class<BaseInfo>(this));
      try_serialize(info, ar);
    }
  };

  template <class Archive>
  void serialize(Archive& ar) {
    ar(m_infos);
  }

  std::string to_json_string() const;

  void from_json_string(const std::string& val);

  template <typename T>
  bool has() const;

  // add attribute T with default value
  template <typename T>
  void set();

  template <typename T>
  void set(const T& value);

  template <typename T>
  T& get();

  template <typename T>
  const T& get() const;

  template <typename T>
  T move();

  template <typename T>
  void clear();

  virtual ~StatusBase() {}

  template <typename T>
  void set_or_clear(bool set_flag);

 private:
  const InfoMap::Map& info_map() const { return m_infos.data; }

  InfoMap::Map& info_map() { return m_infos.data; }

  template <typename T>
  std::type_index id() const {
    return std::type_index(typeid(InfoProxy<T>));
  }

  InfoMap m_infos;
};

}  // namespace planning
}  // namespace ad_byd

CEREAL_REGISTER_TEMPLATE(::ad_byd::planning::StatusBase::InfoProxy)

namespace ad_byd {
namespace planning {

// a helper class for tag behaves like a pointer
// example:
//
//      struct Origin : public Pointer<Unit> {};
//
template <typename T>
class Pointer {
 public:
  Pointer() : m_ptr(nullptr) {}

  void Assign(T* arg) { this->m_ptr = arg; }

  T* get() const {
    CHECK_NOTNULL(m_ptr);
    return m_ptr;
  }

  operator T*() { return get(); }

  bool operator==(const Pointer& arg) const { return this->m_ptr == arg.m_ptr; }

  bool operator==(T* arg) const { return this->m_ptr == arg; }

  T& operator*() const { return *m_ptr; }

  T* operator->() const { return get(); }

  bool is_null() const { return m_ptr == NULL; }

 private:
  T* m_ptr;
};

// a helper class for tag behaves like a number
// example:
//
//      struct Count : public Value<int> {};
//
template <typename T>
class Value {
 public:
  Value() : m_value(T()) {}

  T& operator*() { return m_value; }

  T operator*() const { return m_value; }

  bool operator==(const T& other) { return m_value == other.m_value; }

  template <class Archive>
  void serialize(Archive& ar) {
    try_serialize(m_value, ar);
  }

  T* operator->() { return &m_value; }

  const T* operator->() const { return &m_value; }

 private:
  T m_value;
};

template <typename T>
bool StatusBase::has() const {
  return info_map().find(id<T>()) != info_map().end();
}

template <typename T>
void StatusBase::set() {
  if (!has<T>()) {
    CEREAL_REGISTER_TEMPLATE_CLASS_INSTANTIATION(InfoProxy<T>);
    info_map().insert(
        std::make_pair(id<T>(), std::make_shared<InfoProxy<T>>()));
  }
}

// Avoid using this function, cuz it will incur copy assignment and temporary
// object destruction
//   if you reclaim resources in destructor of T, it may reclaim resources
//   when `const T& value` destruct, so it will not work as expected
//
// Recommand: use get<T> , then set corresponding members in T
//            use set<T> only when copy assignment is "safe & trivial"
template <typename T>
void StatusBase::set(const T& value) {
  this->get<T>() = value;
}

template <typename T>
T& StatusBase::get() {
  set<T>();  // NOLINT
  const T& info = static_cast<const StatusBase*>(this)->get<T>();
  return const_cast<T&>(info);
}

template <typename T>
const T& StatusBase::get() const {
  auto it = info_map().find(id<T>());
  CHECK(it != info_map().end())
      << "call get<" << typeid(T).name()
      << ">() failed on a const Status object since the it\'s not set";
  const InfoProxy<T>* proxy =
      static_cast<const InfoProxy<T>*>(it->second.get());
  return proxy->info;
}

template <typename T>
void StatusBase::clear() {
  info_map().erase(id<T>());
}

template <typename T>
T StatusBase::move() {
  T result = get<T>();
  clear<T>();
  return result;
}

template <typename T>
void StatusBase::set_or_clear(bool set_flag) {
  if (set_flag) {
    set<T>();
  } else {
    clear<T>();
  }
}

class ThreadSafeStatusBase : public StatusBase {
 public:
  std::recursive_mutex& get_mutex() { return _mutex; }

  std::string to_json_string() const {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    return StatusBase::to_json_string();
  }

  void from_json_string(const std::string& val) {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    StatusBase::from_json_string(val);
  }

  template <typename T>
  bool has() const {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    return StatusBase::has<T>();
  }

  // add attribute T with default value
  template <typename T>
  void set() {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    StatusBase::set<T>();
  }

  template <typename T>
  void set(const T& value) {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    StatusBase::set<T>(value);
  }

  template <typename T>
  T& get() {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    return StatusBase::get<T>();
  }

  template <typename T>
  const T& get() const {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    return StatusBase::get<T>();
  }

  template <typename T>
  T move() {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    return StatusBase::move<T>();
  }

  template <typename T>
  void clear() {
    std::lock_guard<std::recursive_mutex> lock(_mutex);
    return StatusBase::clear<T>();
  }

 private:
  mutable std::recursive_mutex _mutex;
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_COMMON_STATUS_BASE_H
