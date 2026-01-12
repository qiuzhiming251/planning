

#ifndef AD_BYD_PLANNING_INFERENCE_BASE_INFERENCE_H
#define AD_BYD_PLANNING_INFERENCE_BASE_INFERENCE_H

#include <string>
#include <unordered_map>
#include <vector>

namespace ad_byd {
namespace planning {

enum class TrtModelType {
  UNKNOWN = 0,
  UFF = 1,
  ONNX = 2,
  CAFFE = 3,
};

struct TrtInferenceOption {
  TrtModelType type = TrtModelType::UNKNOWN;
  int32_t device_id = 0;
  int32_t max_batch_size = 0;
  std::size_t max_workspace = 0;
  bool enable_fp16 = false;
  bool enable_int8 = false;
};

// 接口定义
class BaseInference {
 public:
  BaseInference() = default;
  virtual ~BaseInference() = default;

  // 推理器必须提供的接口
  virtual void Init(const TrtInferenceOption&) = 0;
  virtual bool Infer(
      const int32_t,
      std::unordered_map<std::string, std::vector<float>*>* const,
      std::vector<std::vector<float>>* const) = 0;
  virtual bool Infer() = 0;
  virtual void CreateEngineModel(
      const std::string& model_file_path,
      const std::vector<std::string>& input_node_names,
      const std::vector<std::string>& output_node_names,
      bool is_pinned_memory) = 0;
  virtual void RegisterPinnedMemory(
      std::unordered_map<std::string, std::vector<float>*>& data_map) = 0;
  virtual void UnregisterPinnedMemory(
      std::unordered_map<std::string, std::vector<float>*>& data_map) = 0;
  virtual void PrepareIoBuffer(
      const std::unordered_map<std::string, std::vector<float>*>& input_map,
      const std::unordered_map<std::string, std::vector<float>*>&
          output_map) = 0;
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_INFERENCE_BASE_INFERENCE_H