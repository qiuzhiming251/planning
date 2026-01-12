

#include "inference/ad_byd_inference/inference_creator.h"
#if defined(OFFLINE) || defined(PLANNING_ONLY)
#include "inference/ad_byd_inference/dummy_inference.h"
#else
#include "inference/ad_byd_inference/trt_inference.h"
#endif

namespace ad_byd {
namespace planning {

std::unique_ptr<BaseInference> InferenceCreator::Create(
    const std::string& name) {
  std::unique_ptr<BaseInference> base_infer_ptr;
#if defined(OFFLINE) || defined(PLANNING_ONLY)
  base_infer_ptr.reset(new DummyInference(name));
#else
  base_infer_ptr.reset(new TrtInference(name));
#endif
  return base_infer_ptr;
}

}  // namespace planning
}  // namespace ad_byd