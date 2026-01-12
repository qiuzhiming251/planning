

#ifndef ST_PLANNING_CONTAINER_FLAT_MAP
#define ST_PLANNING_CONTAINER_FLAT_MAP

#include <functional>

#include "boost/container/flat_map.hpp"

namespace st {

// much like flat_map.h, this is the set version
template <typename K, typename V, typename Compare = std::less<K>>
using FlatMap = boost::container::flat_map<K, V, Compare>;

}  // namespace st

#endif  // ST_PLANNING_CONTAINER_FLAT_MAP
