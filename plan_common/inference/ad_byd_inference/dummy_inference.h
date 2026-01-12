

#ifndef AD_BYD_PLANNING_INFERENCE_DUMMY_INFERENCE_H
#define AD_BYD_PLANNING_INFERENCE_DUMMY_INFERENCE_H

#include <string>
#include <unordered_map>
#include <vector>

#include "inference/ad_byd_inference/base_inference.h"

namespace ad_byd {
namespace planning {

class DummyInference : public BaseInference {
 public:
  DummyInference(const std::string& name);
  ~DummyInference() = default;

  void Init(const TrtInferenceOption& config);
  void CreateEngineModel(const std::string& model_file_path,
                         const std::vector<std::string>& input_node_names,
                         const std::vector<std::string>& output_node_names,
                         bool is_pinned_memory);
  bool Infer(
      const int32_t batch_size,
      std::unordered_map<std::string, std::vector<float>*>* const input_datas,
      std::vector<std::vector<float>>* const output_datas);
  bool Infer();
  void RegisterPinnedMemory(
      std::unordered_map<std::string, std::vector<float>*>& data_map);
  void UnregisterPinnedMemory(
      std::unordered_map<std::string, std::vector<float>*>& data_map);
  void PrepareIoBuffer(
      const std::unordered_map<std::string, std::vector<float>*>& input_map,
      const std::unordered_map<std::string, std::vector<float>*>& output_map);
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_INFERENCE_DUMMY_INFERENCE_H