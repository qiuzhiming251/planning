#pragma once

#include <optional>
#include <string>

#include "absl/container/flat_hash_map.h"

namespace st::planning {

struct CipvObjectInfo {
  std::optional<std::string> nearest_object_id = std::nullopt;
  std::optional<std::string> nearest_stay_object_id = std::nullopt;
  std::optional<std::string> second_nearest_stay_object_id = std::nullopt;
  absl::flat_hash_map<std::string, double> object_distance_map;
};

}  // namespace st::planning
