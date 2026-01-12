

#ifndef AD_BYD_PLANNING_INFERENCE_TRT_INFERENCE_H
#define AD_BYD_PLANNING_INFERENCE_TRT_INFERENCE_H

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "NvInfer.h"
#include "NvOnnxParser.h"
#include "NvUffParser.h"
#include "driver_types.h"
#include "inference/ad_byd_inference/base_inference.h"

// CUDA foo must use this macro to invoke
#define CUDA_SAFE_CALL(call)                                              \
  do {                                                                    \
    cudaError_t err = call;                                               \
    if (cudaSuccess != err) {                                             \
      LERROR("Cuda error in file %s in line %d : %s", __FILE__, __LINE__, \
             cudaGetErrorString(err));                                    \
      CHECK(false);                                                       \
    }                                                                     \
  } while (0)

namespace ad_byd {
namespace planning {

class TrtInference : public BaseInference {
 public:
  TrtInference(const std::string& name);
  ~TrtInference();

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

 private:
  void PremallocGpuMemory();
  void DeserializeFromBin();
  void SerializeToBin(::nvinfer1::ICudaEngine* tmp_engine);
  void* CreateInputBuffer(int64_t eltCount, ::nvinfer1::DataType dtype,
                          std::vector<float>* const input_data) const;
  ::nvinfer1::ICudaEngine* LoadModelAndCreateEngine(
      const std::string& model_file,
      const std::vector<std::string>& input_node_names,
      const std::vector<std::string>& output_node_names);
  void AssignInputBuffer(int64_t elt_count, ::nvinfer1::DataType dtype,
                         void* buffer,
                         std::vector<float>* const input_data) const;
  void FetchOutput(int64_t elt_count, ::nvinfer1::DataType dtype, void* buffer,
                   std::vector<std::vector<float>>* const output_data) const;

  inline uint64_t GetElementSize(::nvinfer1::DataType t) const;
  inline int64_t GetVolume(const ::nvinfer1::Dims& d) const;
  void* SafeCudaMalloc(std::size_t mem_size) const;
  std::vector<std::pair<int64_t, ::nvinfer1::DataType>>
  CalculateBindingBufferSizes(int32_t batch_size) const;
  bool CheckFileCompleteByLength(const std::string& file_path) const;
  void CaptureCudaGraph();

 private:
  std::string model_name_;
  std::string serialized_model_path_;
  std::string serialized_model_tmp_path_;

  TrtInferenceOption config_;
  std::vector<std::string> input_nodes_;
  std::vector<std::string> output_nodes_;
  ::nvinfer1::ICudaEngine* engine_ = nullptr;
  ::nvinfer1::IExecutionContext* context_ = nullptr;
  ::nvinfer1::IRuntime* runtime_ = nullptr;

  int32_t nb_bindings_ = 0;
  std::vector<void*> buffers_;

  bool is_pinned_memory_ = false;

  cudaStream_t stream_;
  std::shared_ptr<cudaGraphExec_t> graph_exec_;
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_INFERENCE_TRT_INFERENCE_H
