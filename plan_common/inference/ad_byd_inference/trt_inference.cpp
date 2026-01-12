

#include <sys/stat.h>

#include "NvInferPlugin.h"
#include "boost/filesystem.hpp"
#include "plan_common/gflags.h"
#include "plan_common/log.h"
#include "plan_common/planning_macros.h"
#include "cuda_runtime_api.h"
#include "glog/logging.h"
#include "inference/ad_byd_inference/trt_inference.h"
#include "inference/ad_byd_inference/trt_logger.h"
#include "nvtx3/nvToolsExt.h"
#include "plan_common/util/hr_timer.h"
#include "plan_common/util/time_logger.h"

namespace ad_byd {
namespace planning {

TrtInference::TrtInference(const std::string& name)
    : BaseInference(), model_name_(name) {
  const auto& postfix = "_" + FLAGS_ad_byd_planning_platform + ".bin";
  serialized_model_path_ =
      FLAGS_ad_byd_planning_cache_data + "/" + model_name_ + postfix;
  LINFO("serialized_model_path: %s", serialized_model_path_.c_str());
  serialized_model_tmp_path_ = serialized_model_path_ + ".tmp";
  boost::filesystem::path pnc_data_dir(FLAGS_ad_byd_planning_cache_data);
  if (!boost::filesystem::exists(pnc_data_dir)) {
    CHECK(boost::filesystem::create_directories(pnc_data_dir));
  }

  if (FLAGS_ad_byd_planning_set_stream_priority) {
    int least_priority, greatest_priority;
    CUDA_SAFE_CALL(
        cudaDeviceGetStreamPriorityRange(&least_priority, &greatest_priority));
    LINFO(
        "cuda stream priority range, least_priority: %d, greatest_priority: %d",
        least_priority, greatest_priority);
    CUDA_SAFE_CALL(cudaStreamCreateWithPriority(&stream_, cudaStreamNonBlocking,
                                                greatest_priority));
  } else {
    CUDA_SAFE_CALL(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking));
  }
}

TrtInference::~TrtInference() {
  if (!is_pinned_memory_) {
    for (int32_t binding_idx = 0; binding_idx < nb_bindings_; ++binding_idx) {
      CUDA_SAFE_CALL(cudaFree(buffers_[binding_idx]));
    }
  }
  context_->destroy();
  engine_->destroy();
  runtime_->destroy();
  CUDA_SAFE_CALL(cudaStreamDestroy(stream_));
}

void TrtInference::CaptureCudaGraph() {
  // TODO: infer once before capture, is it necessary?
  context_->enqueueV2(buffers_.data(), stream_, nullptr);
  cudaStreamSynchronize(stream_);
  cudaGraph_t graph;
  // TODO: do not use shared_ptr
  graph_exec_ = std::make_shared<cudaGraphExec_t>();
  cudaStreamBeginCapture(stream_, cudaStreamCaptureModeThreadLocal);
  context_->enqueueV2(buffers_.data(), stream_, nullptr);
  cudaStreamEndCapture(stream_, &graph);
  cudaGraphInstantiate(graph_exec_.get(), graph, nullptr, nullptr, 0);
  cudaGraphDestroy(graph);
}

void TrtInference::Init(const TrtInferenceOption& config) {
  config_ = config;
  CHECK(config_.type == TrtModelType::ONNX);
}

void TrtInference::RegisterPinnedMemory(
    std::unordered_map<std::string, std::vector<float>*>& data_map) {
  for (auto& data : data_map) {
    CUDA_SAFE_CALL(cudaHostRegister(data.second->data(),
                                    data.second->size() * sizeof(float),
                                    cudaHostRegisterDefault));
  }
}

void TrtInference::UnregisterPinnedMemory(
    std::unordered_map<std::string, std::vector<float>*>& data_map) {
  // TODO: 首先判断是否已经被反注册，若是，则跳过
  for (auto& data : data_map) {
    CUDA_SAFE_CALL(cudaHostUnregister(data.second->data()));
  }
}

bool TrtInference::CheckFileCompleteByLength(
    const std::string& file_path) const {
  std::ifstream load_file(file_path.c_str(), std::ofstream::binary);
  std::stringstream sstr;
  sstr << load_file.rdbuf();
  std::string buffer = sstr.str();
  load_file.close();

  std::size_t min_file_size = 1e4;
  LINFO("Serialized model length : %lu (min file size : %lu)", buffer.size(),
        min_file_size);
  return buffer.size() > min_file_size;
}

void TrtInference::CreateEngineModel(
    const std::string& model_file_path,
    const std::vector<std::string>& input_node_names,
    const std::vector<std::string>& output_node_names, bool is_pinned_memory) {
  int device_cnt = 0;
  CUDA_SAFE_CALL(cudaGetDeviceCount(&device_cnt));
  CHECK(device_cnt > 0) << "device cnt: " << device_cnt;
  LINFO("Set device id %d", config_.device_id);
  //   CUDA_SAFE_CALL(cudaSetDevice(config_.device_id));

  input_nodes_.assign(input_node_names.begin(), input_node_names.end());
  output_nodes_.assign(output_node_names.begin(), output_node_names.end());

  struct stat buffer;
  bool model_file_exists = (stat(serialized_model_path_.c_str(), &buffer) == 0);
  LINFO("-----model_file_exists: %d", model_file_exists);
  if (FLAGS_ad_byd_planning_platform == "orin") {
    CHECK(model_file_exists &&
          CheckFileCompleteByLength(serialized_model_path_));
  }

  if (model_file_exists && CheckFileCompleteByLength(serialized_model_path_)) {
    DeserializeFromBin();
  } else {
    LINFO("Not find serialized model path %s, so create new",
          serialized_model_path_.c_str());
    ::nvinfer1::ICudaEngine* tmp_engine = LoadModelAndCreateEngine(
        model_file_path, input_node_names, output_node_names);
    CHECK_NOTNULL(tmp_engine);
    SerializeToBin(tmp_engine);
    tmp_engine->destroy();
  }
  context_ = engine_->createExecutionContext();
  CHECK_NOTNULL(context_);

  nb_bindings_ = engine_->getNbIOTensors();  // engine_->getNbBindings();
  for (int i = 0; i < nb_bindings_; i++) {
    LINFO("IO Tensors name: %s", engine_->getIOTensorName(i));
  }

  is_pinned_memory_ = is_pinned_memory;
  LINFO("is_pinned_memory: %d", is_pinned_memory);
  QUIT_IF_VOID_QUIET(is_pinned_memory_);

  PremallocGpuMemory();
}

::nvinfer1::ICudaEngine* TrtInference::LoadModelAndCreateEngine(
    const std::string& model_file,
    const std::vector<std::string>& input_node_names,
    const std::vector<std::string>& output_node_names) {
  int32_t verbosity =
      static_cast<int32_t>(nvinfer1::ILogger::Severity::kWARNING);
  ::nvinfer1::IBuilder* builder =
      ::nvinfer1::createInferBuilder(_s_GLOBAL_LOGGER);
  CHECK_NOTNULL(builder);
  ::nvinfer1::INetworkDefinition* network = nullptr;
  ::nvonnxparser::IParser* onnx_parser = nullptr;

  if (config_.type == TrtModelType::ONNX) {
    const auto explicitBatch =
        1U << static_cast<uint32_t>(
            nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    config_.max_batch_size = 1;
    network = builder->createNetworkV2(explicitBatch);
    onnx_parser = nvonnxparser::createParser(*network, _s_GLOBAL_LOGGER);

    struct stat buffer;
    bool model_file_exists = (stat(model_file.c_str(), &buffer) == 0);
    CHECK_EQ(model_file_exists, true);

    if (!onnx_parser->parseFromFile(model_file.c_str(), verbosity)) {
      LERROR("Parse onnx file failed, onnx file : %s", model_file.c_str());
      return nullptr;
    }

    LINFO("create parser file %s finish", model_file.c_str());
  } else {
    LERROR("not support %d type models support onnx[2]",
           static_cast<int>(config_.type));
    CHECK(false);
  }

  builder->setMaxBatchSize(config_.max_batch_size);
  ::nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
  config->setMaxWorkspaceSize(config_.max_workspace);
  if (config_.enable_fp16) {
    config->setFlag(::nvinfer1::BuilderFlag::kFP16);
  }
  if (config_.enable_int8) {
    config->setFlag(::nvinfer1::BuilderFlag::kINT8);
  }

  ::nvinfer1::ICudaEngine* engine =
      builder->buildEngineWithConfig(*network, *config);
  if (!engine) {
    LERROR("Unable to create engine");
    return nullptr;
  }

  onnx_parser->destroy();
  network->destroy();
  builder->destroy();

  return engine;
}

// Mapping input/output memory to avoid data transfer, just assign address to
// trt raw buffer
void TrtInference::PrepareIoBuffer(
    const std::unordered_map<std::string, std::vector<float>*>& input_map,
    const std::unordered_map<std::string, std::vector<float>*>& output_map) {
  //   CUDA_SAFE_CALL(cudaSetDevice(config_.device_id));
  CHECK_NOTNULL(engine_);
  std::vector<std::pair<int64_t, ::nvinfer1::DataType>> buffers_sizes =
      CalculateBindingBufferSizes(1);
  // tensorrt engine actually operates on an array of io pointers bound with
  // feature index
  std::vector<void*> trt_buffer(input_map.size() + output_map.size());

  // reuse input buffer
  float* d_ptr = nullptr;
  for (const auto& input : input_map) {
    LINFO("-----input_name: %s", input.first.c_str());
    LINFO("-----input_value: %d", input.second->size());
    int32_t binding_idx = engine_->getBindingIndex(input.first.c_str());
    LINFO("-----binding_idx: %d", binding_idx);
    CHECK_GE(binding_idx, 0);
    auto size_type_pair = buffers_sizes[binding_idx];
    LINFO("[in]%s@%d size=%zu", input.first.c_str(), binding_idx,
          size_type_pair.first);
    CHECK_EQ(size_type_pair.first, input.second->size());
    // get corresponding virtual address on CUDA memory system
    CUDA_SAFE_CALL(
        cudaHostGetDevicePointer((void**)&d_ptr, input.second->data(), 0));
    trt_buffer[binding_idx] = d_ptr;
  }

  // reuse output buffer
  for (const auto& output : output_map) {
    int32_t binding_idx = engine_->getBindingIndex(output.first.c_str());
    CHECK_GE(binding_idx, 0);
    auto size_type_pair = buffers_sizes[binding_idx];
    LINFO("[out]%s@%d size=%zu", output.first.c_str(), binding_idx,
          size_type_pair.first);
    CHECK_EQ(size_type_pair.first, output.second->size());
    CUDA_SAFE_CALL(
        cudaHostGetDevicePointer((void**)&d_ptr, output.second->data(), 0));
    trt_buffer[binding_idx] = d_ptr;
  }
  buffers_ = std::move(trt_buffer);

  if (FLAGS_ad_byd_planning_set_cuda_graph) {
    CaptureCudaGraph();
    LWARN("capture cuda graph success");
  }
}

void TrtInference::PremallocGpuMemory() {
  // nb_bindings_ = engine_->getNbBindings();
  nb_bindings_ = engine_->getNbIOTensors();  // 18
  LINFO("nb_bindings_: %d", nb_bindings_);
  buffers_.resize(nb_bindings_);

  std::vector<std::pair<int64_t, ::nvinfer1::DataType>> buffers_sizes =
      CalculateBindingBufferSizes(config_.max_batch_size);

  for (int32_t i = 0; i < nb_bindings_; ++i) {
    std::pair<int64_t, ::nvinfer1::DataType> bufferSizesOutput =
        buffers_sizes[i];
    buffers_[i] = SafeCudaMalloc(bufferSizesOutput.first *
                                 GetElementSize(bufferSizesOutput.second));
    LINFO("in PremallocGpuMemory: element nums: %ld, dataType: %d",
          bufferSizesOutput.first, bufferSizesOutput.second);
  }
}

void TrtInference::DeserializeFromBin() {
  LINFO("Loading serialized model path %s", serialized_model_path_.c_str());
  std::ifstream load_file(serialized_model_path_.c_str(),
                          std::ofstream::binary);
  std::stringstream sstr;
  sstr << load_file.rdbuf();
  std::string buffer = sstr.str();
  load_file.close();

  LINFO("Deserialized model length %lu", buffer.size());

  runtime_ = ::nvinfer1::createInferRuntime(_s_GLOBAL_LOGGER);
  CHECK_NOTNULL(runtime_);
  engine_ =
      runtime_->deserializeCudaEngine(buffer.data(), buffer.size(), nullptr);
  CHECK_NOTNULL(engine_);
}

void TrtInference::SerializeToBin(::nvinfer1::ICudaEngine* tmp_engine) {
  boost::filesystem::path tmp_model_path(serialized_model_tmp_path_);
  boost::filesystem::path model_path(serialized_model_path_);
  if (boost::filesystem::exists(tmp_model_path)) {
    CHECK(boost::filesystem::remove(tmp_model_path));
  }
  ::nvinfer1::IHostMemory* serialized_model = tmp_engine->serialize();
  std::ofstream model_file(serialized_model_tmp_path_.c_str(),
                           std::ofstream::binary);
  model_file.write((const char*)serialized_model->data(),
                   serialized_model->size());
  model_file.close();

  LINFO("Serialized model length %lu", serialized_model->size());
  boost::filesystem::rename(tmp_model_path, model_path);
  CHECK(boost::filesystem::exists(model_path));

  runtime_ = ::nvinfer1::createInferRuntime(_s_GLOBAL_LOGGER);
  CHECK_NOTNULL(runtime_);
  engine_ = runtime_->deserializeCudaEngine(serialized_model->data(),
                                            serialized_model->size(), nullptr);
  CHECK_NOTNULL(engine_);

  serialized_model->destroy();
}

bool TrtInference::Infer(
    const int32_t batch_size,
    std::unordered_map<std::string, std::vector<float>*>* const input_datas,
    std::vector<std::vector<float>>* const output_datas) {
  TimeLogger logger("TrtInference::Infer");
  std::vector<std::pair<int64_t, ::nvinfer1::DataType>> buffers_sizes =
      CalculateBindingBufferSizes(batch_size);

  for (const auto& input : *input_datas) {
    int32_t binding_idx_input = engine_->getBindingIndex(input.first.c_str());
    CHECK(binding_idx_input >= 0);
    std::pair<int64_t, ::nvinfer1::DataType> buffer_sizes_input =
        buffers_sizes[binding_idx_input];
    CHECK(static_cast<int64_t>(input.second->size()) <=
          buffer_sizes_input.first);
    AssignInputBuffer(buffer_sizes_input.first, buffer_sizes_input.second,
                      buffers_[binding_idx_input], input.second);
  }
  logger.RegisterTime("memory_assign");

  bool result = false;
  if (config_.type == TrtModelType::ONNX) {
    result = context_->executeV2(buffers_.data());
  } else {
    LERROR("not support %d type models support onnx[2]",
           static_cast<int>(config_.type));
    CHECK(false);
  }

  cudaError_t err = ::cudaGetLastError();
  LINFO("TRT execute api result %s", ::cudaGetErrorString(err));
  QUIT_IF(!result, false, ERROR, "TRT context execute failed");
  logger.RegisterTime("engine_execute");

  for (const auto& output_name : output_nodes_) {
    int32_t binding_idx = engine_->getBindingIndex(output_name.c_str());
    const auto& buffer_sizes_output = buffers_sizes[binding_idx];
    FetchOutput(buffer_sizes_output.first, buffer_sizes_output.second,
                buffers_[binding_idx], output_datas);
  }
  logger.RegisterTimeAndPrint("FetchOutput");

  return true;
}

bool TrtInference::Infer() {
  LINFO("TRT BEGIN inference");
  HRTimer timer;

  // actual inference
  // bool result = context_->enqueueV2(buffers_.data(), stream_, nullptr);
  if (FLAGS_ad_byd_planning_set_cuda_graph) {
    CUDA_SAFE_CALL(cudaGraphLaunch(*graph_exec_, stream_));
  } else {
    bool result = context_->enqueueV2(buffers_.data(), stream_, nullptr);
    QUIT_IF(!result, false, ERROR, "TRT inference failed");
  }

  cudaError_t err = ::cudaGetLastError();
  LINFO("TRT enqueueV2 return %s", ::cudaGetErrorString(err));
  // QUIT_IF(!result, false, ERROR, "TRT inference failed");
  CUDA_SAFE_CALL(cudaStreamSynchronize(stream_));
  LINFO("TRT END inference, %s took %ldms", model_name_.c_str(),
        timer.ElapsedMs());
  return true;
}

void* TrtInference::CreateInputBuffer(
    int64_t eltCount, ::nvinfer1::DataType dtype,
    std::vector<float>* const input_data) const {
  CHECK_EQ(eltCount, static_cast<int64_t>(input_data->size()));
  CHECK_EQ(GetElementSize(dtype), sizeof(float));
  std::size_t memSize = eltCount * GetElementSize(dtype);
  void* deviceMem = SafeCudaMalloc(memSize);
  CUDA_SAFE_CALL(cudaMemcpy(deviceMem, input_data->data(), memSize,
                            cudaMemcpyHostToDevice));
  return deviceMem;
}

void TrtInference::FetchOutput(
    int64_t ele_count, ::nvinfer1::DataType dtype, void* buffer,
    std::vector<std::vector<float>>* const output_data) const {
  CHECK_EQ(GetElementSize(dtype), sizeof(float));

  std::size_t mem_size = ele_count * GetElementSize(dtype);
  std::vector<float> outputs(ele_count);
  CUDA_SAFE_CALL(
      cudaMemcpy(outputs.data(), buffer, mem_size, cudaMemcpyDeviceToHost));
  output_data->emplace_back(std::move(outputs));
}

void TrtInference::AssignInputBuffer(
    int64_t ele_count, ::nvinfer1::DataType dtype, void* buffer,
    std::vector<float>* const input_data) const {
  CHECK_EQ(ele_count, static_cast<int64_t>(input_data->size()));
  CHECK_EQ(GetElementSize(dtype), sizeof(float));

  std::size_t mem_size = ele_count * GetElementSize(dtype);
  CUDA_SAFE_CALL(
      cudaMemcpy(buffer, input_data->data(), mem_size, cudaMemcpyHostToDevice));
}

inline uint64_t TrtInference::GetElementSize(::nvinfer1::DataType t) const {
  switch (t) {
    case ::nvinfer1::DataType::kINT32:
      return 4;
    case ::nvinfer1::DataType::kFLOAT:
      return 4;
    case ::nvinfer1::DataType::kHALF:
      return 2;
    case ::nvinfer1::DataType::kINT8:
      return 1;
    default:
      break;
  }

  CHECK(false);
  return 0;
}

inline int64_t TrtInference::GetVolume(const ::nvinfer1::Dims& d) const {
  int64_t v = 1;
  for (int64_t i = 0; i < d.nbDims; i++) {
    LINFO("d.d[i] = %d", d.d[i]);
    v *= d.d[i];
  }
  return v;
}

void* TrtInference::SafeCudaMalloc(std::size_t mem_size) const {
  void* deviceMem = nullptr;
  CUDA_SAFE_CALL(cudaMalloc(&deviceMem, mem_size));
  if (deviceMem == nullptr) {
    LERROR("Cuda malloc out of memory");
    CHECK(false);
  }
  return deviceMem;
}

std::vector<std::pair<int64_t, ::nvinfer1::DataType>>
TrtInference::CalculateBindingBufferSizes(int32_t batch_size) const {
  std::vector<std::pair<int64_t, ::nvinfer1::DataType>> sizes;
  for (int32_t i = 0; i < nb_bindings_; ++i) {
    auto tmp_name =
        engine_->getIOTensorName(i);  // engine_->getBindingDimensions(i);
    ::nvinfer1::Dims dims = engine_->getTensorShape(tmp_name);
    ::nvinfer1::DataType dtype = engine_->getTensorDataType(
        tmp_name);  // engine_->getBindingDataType(i);

    int64_t elt_count = GetVolume(dims) * batch_size;
    LINFO(
        "TensorName: %s, TensorDataType: %d, batch_size: %d, elementCount: "
        "%ld, dims: %d",
        tmp_name, dtype, batch_size, elt_count, dims.nbDims);
    sizes.push_back(std::make_pair(elt_count, dtype));
  }

  return sizes;
}

}  // namespace planning
}  // namespace ad_byd
