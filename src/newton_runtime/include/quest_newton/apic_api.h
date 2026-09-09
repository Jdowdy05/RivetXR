#pragma once

#include <cstddef>

namespace quest_newton {

struct ApicGraph;

using WarpVersionFunction = const char* (*)();
using ApicLoadGraphFunction = ApicGraph* (*)(void* context, const char* path, int device_type);
using ApicDestroyGraphFunction = void (*)(ApicGraph* graph);
using ApicGetNumKernelsFunction = int (*)(ApicGraph* graph);
using ApicGetKernelStringFunction = const char* (*)(ApicGraph* graph, int index);
using ApicRegisterLoadedCpuKernelFunction =
    void (*)(ApicGraph* graph, const char* kernel_key, const char* module_hash, void* forward_fn, void* backward_fn);
using ApicGetNumParamsFunction = int (*)(ApicGraph* graph);
using ApicGetParamNameFunction = const char* (*)(ApicGraph* graph, int index);
using ApicGetParamSizeFunction = std::size_t (*)(ApicGraph* graph, const char* name);
using ApicSetParamFunction = bool (*)(ApicGraph* graph, const char* name, const void* data, std::size_t size);
using ApicGetParamFunction = bool (*)(ApicGraph* graph, const char* name, void* data, std::size_t size);
using ApicCpuReplayGraphFunction = bool (*)(ApicGraph* graph);

struct ApicApi {
  WarpVersionFunction version = nullptr;
  ApicLoadGraphFunction load_graph = nullptr;
  ApicDestroyGraphFunction destroy_graph = nullptr;
  ApicGetNumKernelsFunction get_num_kernels = nullptr;
  ApicGetKernelStringFunction get_kernel_key = nullptr;
  ApicGetKernelStringFunction get_kernel_module_hash = nullptr;
  ApicGetKernelStringFunction get_kernel_forward_name = nullptr;
  ApicGetKernelStringFunction get_kernel_backward_name = nullptr;
  ApicRegisterLoadedCpuKernelFunction register_loaded_cpu_kernel = nullptr;
  ApicGetNumParamsFunction get_num_params = nullptr;
  ApicGetParamNameFunction get_param_name = nullptr;
  ApicGetParamSizeFunction get_param_size = nullptr;
  ApicSetParamFunction set_param = nullptr;
  ApicGetParamFunction get_param = nullptr;
  ApicCpuReplayGraphFunction replay_graph = nullptr;
};

}  // namespace quest_newton
