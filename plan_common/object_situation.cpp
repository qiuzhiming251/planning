//
// Created by xxx on 8/2/21.
//

#include "object_situation.h"
namespace ad_byd::planning {
void ObjectSituation::Reset() {
  lon_distance_ = 0.0;  // m
  lat_distance_ = 0.0;  // m
  motion_ = {};
}

}  // namespace ad_byd::planning
