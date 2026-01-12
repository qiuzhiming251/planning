

#ifndef AD_BYD_PLANNING_UTILS_LRU_CACHE_ASYNC_H
#define AD_BYD_PLANNING_UTILS_LRU_CACHE_ASYNC_H

#include <list>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "plan_common/log.h"
#include "plan_common/planning_error.h"
//#include "plan_common/util/thread_pool.h"
#include "plan_common/async/async_util.h"

namespace ad_byd {
namespace planning {

// template <typename T>
// struct AsyncDeleter {  // deleter
//   void operator()(T* p) const {
//     DELAY_CLEAN([p]() { delete p; });
//   };
// };

template <typename T>
using AsyncDeleter = st::AsyncDeleter<T>;

template <typename T, typename K = int32_t>
class LRUCacheAsync {
 public:
  explicit LRUCacheAsync(std::size_t capacity);
  ~LRUCacheAsync() = default;

  LRUCacheAsync(LRUCacheAsync&) = delete;
  LRUCacheAsync& operator=(LRUCacheAsync&) = delete;

  std::size_t capacity() const;
  std::size_t size() const;
  bool has_key(K key) const;
  bool touch_key(K key);
  bool erase(K key);
  void clear();
  ErrorCode put(K key, std::unique_ptr<T, AsyncDeleter<T>>& value);
  ErrorCode get(K key, T** const value);
  ErrorCode get_silent(K key, T** const value);
  std::list<K> lru_list();

 private:
  struct Item {
    std::unique_ptr<T, AsyncDeleter<T>> value;
    typename std::list<K>::iterator address;
  };

 private:
  std::size_t _capacity = 0;
  std::unordered_map<K, Item> _map;
  std::list<K> _list;
  mutable std::recursive_mutex _mutex;
};

template <typename T, typename K>
LRUCacheAsync<T, K>::LRUCacheAsync(std::size_t capacity)
    : _capacity(capacity) {}

template <typename T, typename K>
ErrorCode LRUCacheAsync<T, K>::put(K key,
                                   std::unique_ptr<T, AsyncDeleter<T>>& value) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  if (value.get() == nullptr) {
    return ErrorCode::PLANNING_ERROR_NULL_POINTER;
  }
  ErrorCode ret = ErrorCode::PLANNING_OK;
  auto it = _map.find(key);
  if (it == _map.end()) {
    _list.push_front(key);
    Item& item = _map[key];
    item.value = std::move(value);
    item.address = _list.begin();

    if (size() > capacity()) {
      K id = _list.back();
      LRUCacheAsync::Item& _lost_item = _map[id];
      _list.erase(_lost_item.address);
      _map.erase(id);
    }
  } else {
    if (it->second.address != _list.begin()) {
      K id = *(it->second.address);
      _list.erase(it->second.address);
      _list.push_front(id);
      it->second.address = _list.begin();
    }
    it->second.value = std::move(value);
  }
  return ret;
}

template <typename T, typename K>
ErrorCode LRUCacheAsync<T, K>::get(K key, T** const value) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  CHECK_NOTNULL(value);
  *value = nullptr;
  ErrorCode ret = ErrorCode::PLANNING_OK;
  auto it = _map.find(key);
  if (it != _map.end()) {
    if (it->second.address != _list.begin()) {
      K id = *(it->second.address);
      _list.erase(it->second.address);
      _list.push_front(id);
      it->second.address = _list.begin();
    }
    *value = it->second.value.get();
  } else {
    ret = ErrorCode::PLANNING_ERROR_NOT_FOUND;
  }
  return ret;
}

template <typename T, typename K>
ErrorCode LRUCacheAsync<T, K>::get_silent(K key, T** const value) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  CHECK_NOTNULL(value);
  *value = nullptr;
  ErrorCode ret = ErrorCode::PLANNING_OK;
  auto it = _map.find(key);
  if (it != _map.end()) {
    *value = it->second.value.get();
  } else {
    ret = ErrorCode::PLANNING_ERROR_NOT_FOUND;
  }
  return ret;
}

template <typename T, typename K>
std::size_t LRUCacheAsync<T, K>::capacity() const {
  return _capacity;
}

template <typename T, typename K>
std::size_t LRUCacheAsync<T, K>::size() const {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  return _map.size();
}

template <typename T, typename K>
bool LRUCacheAsync<T, K>::has_key(K key) const {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  auto it = _map.find(key);
  return it != _map.end();
}

template <typename T, typename K>
bool LRUCacheAsync<T, K>::touch_key(K key) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  auto it = _map.find(key);
  bool has_key = (it != _map.end());

  if (has_key) {
    if (it->second.address != _list.begin()) {
      K id = *(it->second.address);
      _list.erase(it->second.address);
      _list.push_front(id);
      it->second.address = _list.begin();
    }
  }

  return has_key;
}

template <typename T, typename K>
bool LRUCacheAsync<T, K>::erase(K key) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  auto it = _map.find(key);
  bool has_key = (it != _map.end());
  if (has_key) {
    if (it->second.address != _list.begin()) {
      K id = *(it->second.address);
      _list.erase(it->second.address);
      _map.erase(id);
    }
  }
  return true;
}

template <typename T, typename K>
void LRUCacheAsync<T, K>::clear() {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  _map.clear();
  _list.clear();
}

template <typename T, typename K>
std::list<K> LRUCacheAsync<T, K>::lru_list() {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  return _list;
}

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_UTILS_LRU_CACHE_ASYNC_H
