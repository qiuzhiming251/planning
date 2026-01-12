

#ifndef ONBOARD_CONTAINER_FLAT_SET_H_
#define ONBOARD_CONTAINER_FLAT_SET_H_

#include <functional>

#include "boost/container/flat_set.hpp"

namespace st {

// much like flat_map.h, this is the set version
template <typename T, typename Compare = std::less<T>>
using FlatSet = boost::container::flat_set<T, Compare>;

}  // namespace st

#endif  // ONBOARD_CONTAINER_FLAT_SET_H_
