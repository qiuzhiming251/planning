

#ifndef AD_BYD_PLANNING_INFERENCE_INFERENCE_CREATOR_H
#define AD_BYD_PLANNING_INFERENCE_INFERENCE_CREATOR_H

#include <memory>
#include <string>

#include "inference/ad_byd_inference/base_inference.h"

namespace ad_byd {
namespace planning {

class InferenceCreator {
 public:
  InferenceCreator() = delete;

  static std::unique_ptr<BaseInference> Create(const std::string& name);
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_INFERENCE_INFERENCE_CREATOR_H