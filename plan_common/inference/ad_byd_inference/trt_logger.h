

#ifndef AD_BYD_PLANNING_INFERENCE_TRT_LOGGER_H
#define AD_BYD_PLANNING_INFERENCE_TRT_LOGGER_H

#include <string>

#include "NvInfer.h"
#include "plan_common/log.h"

namespace ad_byd {
namespace planning {

class TrtLogger : public ::nvinfer1::ILogger {
  void log(Severity severity, const char* msg) noexcept {
    if (severity != Severity::kVERBOSE && severity != Severity::kINFO) {
      LWARN("TensorRT logger [%d] : %s", static_cast<int32_t>(severity), msg);
    }
  }
};

static TrtLogger _s_GLOBAL_LOGGER;

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_INFERENCE_TRT_LOGGER_H
