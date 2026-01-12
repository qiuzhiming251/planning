

#ifndef ONBOARD_MAPS_TYPE_DEFS_H_
#define ONBOARD_MAPS_TYPE_DEFS_H_

#include <cstdint>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"

namespace st::mapping {
template <typename Key, typename T>
using FlatHashMap = absl::flat_hash_map<Key, T>;

template <typename T>
using FlatHashSet = absl::flat_hash_set<T>;

// unify the definition of `PatchId` and `LevelId` for either imagery or
// semantic
using PatchId = uint64_t;
using PixelId = uint64_t;

using PatchIds = FlatHashSet<PatchId>;

// TODO: extract all common type_defs (or using alias) into this header
}  // namespace st::mapping

#endif  // ONBOARD_MAPS_TYPE_DEFS_H_
