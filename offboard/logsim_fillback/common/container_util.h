#pragma once

#include <map>
namespace worldview {
namespace util {

template <typename Key, typename T, typename Compare = std::less<Key>,
          typename Allocator = std::allocator<std::pair<const Key, T>>>
typename std::map<Key, T, Compare, Allocator>::const_iterator
before_lower_bound(const std::map<Key, T, Compare, Allocator>& m,
                   const Key& k) {
  if (m.empty()) {
    return m.end();
  }
  auto it = m.lower_bound(k);
  if (it == m.begin()) {
    return m.end();
  }
  --it;
  return it;
}
}  // namespace util
}  // namespace worldview