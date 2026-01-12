#ifndef ONBOARD_PLANNER_SELECTOR_DEFS_H_
#define ONBOARD_PLANNER_SELECTOR_DEFS_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "decider/selector/cost_feature_base.h"

namespace st::planning {

typedef std::vector<std::unique_ptr<CostFeatureBase>> CostFeatures;
typedef absl::flat_hash_map<std::string, CostFeatureBase::CostVec> WeightTable;

struct FeatureCostSum {
  double cost_common = 0.0;
  double cost_same_start = 0.0;
  double static_cost = 0.0;

  inline double cost_sum() const { return cost_common + cost_same_start; }
  bool operator<(const FeatureCostSum& other) const {
    return cost_sum() < other.cost_sum();
  }
  void add(double val, bool is_common) {
    (is_common ? cost_common : cost_same_start) += val;
  }
  void AddWithStaticCost(double val, bool is_common, bool is_static_cost) {
    (is_common ? cost_common : cost_same_start) += val;
    if (is_static_cost) {
      static_cost += val;
    }
  }
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SELECTOR_DEFS_H_
