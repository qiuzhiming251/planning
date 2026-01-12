

#include "inference/ad_byd_inference/dummy_inference.h"

namespace ad_byd {
namespace planning {

DummyInference::DummyInference(const std::string& name) : BaseInference() {
  void(name.c_str());
}

void DummyInference::Init(const TrtInferenceOption& config) {
  void(config.device_id);
}

bool DummyInference::Infer(
    const int32_t batch_size,
    std::unordered_map<std::string, std::vector<float>*>* const input_datas,
    std::vector<std::vector<float>>* const output_datas) {
  // do nothing
  return true;
}

bool DummyInference::Infer() { return true; }

void DummyInference::CreateEngineModel(
    const std::string& model_file_path,
    const std::vector<std::string>& input_node_names,
    const std::vector<std::string>& output_node_names, bool is_pinned_memory) {
  // do nothing
}

void DummyInference::RegisterPinnedMemory(
    std::unordered_map<std::string, std::vector<float>*>& data_map) {
  // do nothing
}

void DummyInference::UnregisterPinnedMemory(
    std::unordered_map<std::string, std::vector<float>*>& data_map) {
  // do nothing
}

void DummyInference::PrepareIoBuffer(
    const std::unordered_map<std::string, std::vector<float>*>& input_map,
    const std::unordered_map<std::string, std::vector<float>*>& output_map) {
  // do nothing
}

}  // namespace planning
}  // namespace ad_byd