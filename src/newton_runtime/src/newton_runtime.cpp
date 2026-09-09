#include "quest_newton/newton_runtime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <string_view>
#include <utility>

namespace quest_newton {
namespace {

constexpr int kApicDeviceCpu = 1;
constexpr std::uint32_t kMinimumSubsteps = 1;
constexpr std::uint32_t kMaximumSubsteps = 12;
constexpr std::string_view kSupportedWarpVersion = "1.18.0.dev2";

struct RequiredBuffer {
  const char* name;
  std::size_t multiplier;
  bool uses_body_count;
};

constexpr std::array<RequiredBuffer, 8> kRequiredBuffers = {{
    {"joint_q_in", 1, false},
    {"joint_qd_in", 1, false},
    {"joint_force", 1, false},
    {"joint_target_q", 1, false},
    {"joint_target_qd", 1, false},
    {"joint_q_out", 1, false},
    {"joint_qd_out", 1, false},
    {"body_q_out", 7, true},
}};

Status MissingSymbol(const std::string_view name) {
  return {StatusCode::symbol_missing, "required symbol is missing: " + std::string(name)};
}

template <typename Function>
Status ResolveFunction(DynamicLibrary& library, const std::string_view name, Function& output) {
  void* const symbol = library.Resolve(name);
  if (symbol == nullptr) {
    return MissingSymbol(name);
  }
  output = reinterpret_cast<Function>(symbol);
  return Status::Ok();
}

StatusOr<ApicApi> ResolveApi(DynamicLibrary& library) {
  ApicApi api;
  Status status = ResolveFunction(library, "wp_version", api.version);
  if (!status.IsOk()) return status;
  status = ResolveFunction(library, "wp_apic_load_graph", api.load_graph);
  if (!status.IsOk()) return status;
  status = ResolveFunction(library, "wp_apic_destroy_graph", api.destroy_graph);
  if (!status.IsOk()) return status;
  status = ResolveFunction(library, "wp_apic_get_num_kernels", api.get_num_kernels);
  if (!status.IsOk()) return status;
  status = ResolveFunction(library, "wp_apic_get_kernel_key", api.get_kernel_key);
  if (!status.IsOk()) return status;
  status = ResolveFunction(library, "wp_apic_get_kernel_module_hash", api.get_kernel_module_hash);
  if (!status.IsOk()) return status;
  status = ResolveFunction(library, "wp_apic_get_kernel_forward_name", api.get_kernel_forward_name);
  if (!status.IsOk()) return status;
  status = ResolveFunction(library, "wp_apic_get_kernel_backward_name", api.get_kernel_backward_name);
  if (!status.IsOk()) return status;
  status = ResolveFunction(library, "wp_apic_register_loaded_cpu_kernel", api.register_loaded_cpu_kernel);
  if (!status.IsOk()) return status;
  status = ResolveFunction(library, "wp_apic_get_num_params", api.get_num_params);
  if (!status.IsOk()) return status;
  status = ResolveFunction(library, "wp_apic_get_param_name", api.get_param_name);
  if (!status.IsOk()) return status;
  status = ResolveFunction(library, "wp_apic_get_param_size", api.get_param_size);
  if (!status.IsOk()) return status;
  status = ResolveFunction(library, "wp_apic_set_param", api.set_param);
  if (!status.IsOk()) return status;
  status = ResolveFunction(library, "wp_apic_get_param", api.get_param);
  if (!status.IsOk()) return status;
  status = ResolveFunction(library, "wp_apic_cpu_replay_graph", api.replay_graph);
  if (!status.IsOk()) return status;
  return api;
}

bool IsFinite(const std::span<const float> values) {
  return std::all_of(values.begin(), values.end(), [](const float value) { return std::isfinite(value); });
}

std::size_t ExpectedBufferBytes(const RequiredBuffer& buffer, const RuntimeConfig& config) {
  const std::size_t count = buffer.uses_body_count ? config.body_count : config.joint_count;
  return count * buffer.multiplier * sizeof(float);
}

Status ValidateConfig(const RuntimeConfig& config) {
  if (config.expected_warp_version.empty() || config.graph_path.empty() || config.joint_count == 0U ||
      config.body_count == 0U || config.warp_library == nullptr) {
    return {StatusCode::invalid_argument, "runtime configuration is incomplete"};
  }
  if (config.required_buffers.size() != kRequiredBuffers.size()) {
    return {StatusCode::invalid_argument, "runtime configuration must list eight graph buffers"};
  }
  for (const RequiredBuffer& required : kRequiredBuffers) {
    const auto configured = std::find_if(config.required_buffers.begin(), config.required_buffers.end(),
                                         [required](const GraphBufferConfig& buffer) {
                                           return buffer.name == required.name;
                                         });
    if (configured == config.required_buffers.end() || configured->bytes != ExpectedBufferBytes(required, config)) {
      return {StatusCode::invalid_argument, "runtime configuration has an invalid graph buffer"};
    }
  }
  return Status::Ok();
}

void* ResolveKernelSymbol(const std::vector<std::unique_ptr<DynamicLibrary>>& libraries, const char* symbol) {
  if (symbol == nullptr || symbol[0] == '\0') {
    return nullptr;
  }
  for (const auto& library : libraries) {
    if (library != nullptr) {
      if (void* const resolved = library->Resolve(symbol); resolved != nullptr) {
        return resolved;
      }
    }
  }
  return nullptr;
}

Status ValidateGraphParameters(const ApicApi& api, ApicGraph* graph, const RuntimeConfig& config) {
  const int parameter_count = api.get_num_params(graph);
  if (parameter_count != static_cast<int>(kRequiredBuffers.size())) {
    return {StatusCode::artifact_error, "graph does not expose exactly eight required parameters"};
  }
  for (const RequiredBuffer& required : kRequiredBuffers) {
    bool found = false;
    for (int index = 0; index < parameter_count; ++index) {
      const char* const name = api.get_param_name(graph, index);
      if (name != nullptr && std::string_view(name) == required.name) {
        found = api.get_param_size(graph, name) == ExpectedBufferBytes(required, config);
        break;
      }
    }
    if (!found) {
      return {StatusCode::artifact_error, "graph parameter is missing or has the wrong size"};
    }
  }
  return Status::Ok();
}

Status RegisterGraphKernels(const ApicApi& api, ApicGraph* graph,
                            const std::vector<std::unique_ptr<DynamicLibrary>>& libraries) {
  struct ResolvedKernel {
    const char* key;
    const char* module_hash;
    void* forward;
    void* backward;
  };

  const int kernel_count = api.get_num_kernels(graph);
  if (kernel_count < 0) {
    return {StatusCode::graph_error, "graph returned a negative kernel count"};
  }
  std::vector<ResolvedKernel> resolved_kernels;
  resolved_kernels.reserve(static_cast<std::size_t>(kernel_count));
  for (int index = 0; index < kernel_count; ++index) {
    const char* const key = api.get_kernel_key(graph, index);
    const char* const module_hash = api.get_kernel_module_hash(graph, index);
    const char* const forward_name = api.get_kernel_forward_name(graph, index);
    const char* const backward_name = api.get_kernel_backward_name(graph, index);
    if (key == nullptr || key[0] == '\0' || module_hash == nullptr || module_hash[0] == '\0' ||
        forward_name == nullptr || forward_name[0] == '\0') {
      return {StatusCode::artifact_error, "graph contains incomplete kernel metadata"};
    }
    void* const forward = ResolveKernelSymbol(libraries, forward_name);
    if (forward == nullptr) {
      return MissingSymbol(forward_name);
    }
    void* backward = nullptr;
    if (backward_name != nullptr && backward_name[0] != '\0') {
      backward = ResolveKernelSymbol(libraries, backward_name);
      if (backward == nullptr) {
        return MissingSymbol(backward_name);
      }
    }
    resolved_kernels.push_back({key, module_hash, forward, backward});
  }
  for (const ResolvedKernel& kernel : resolved_kernels) {
    api.register_loaded_cpu_kernel(graph, kernel.key, kernel.module_hash, kernel.forward, kernel.backward);
  }
  return Status::Ok();
}

}  // namespace

NewtonRuntime::NewtonRuntime(RuntimeConfig config, const ApicApi api, GraphHandle graph)
    : apic_(api),
      warp_library_(std::move(config.warp_library)),
      kernel_libraries_(std::move(config.kernel_libraries)),
      graph_(std::move(graph)),
      joint_positions_(config.joint_count),
      joint_velocities_(config.joint_count),
      body_transforms_(config.body_count * 7U),
      working_joint_positions_(config.joint_count),
      working_joint_velocities_(config.joint_count),
      working_body_transforms_(config.body_count * 7U),
      joint_targets_(config.joint_count),
      joint_target_velocities_(config.joint_count),
      joint_forces_(config.joint_count),
      joint_positions_out_(config.joint_count),
      joint_velocities_out_(config.joint_count),
      body_transforms_out_(config.body_count * 7U) {}

NewtonRuntime::~NewtonRuntime() = default;

StatusOr<std::unique_ptr<NewtonRuntime>> NewtonRuntime::Create(RuntimeConfig config) {
  const Status config_status = ValidateConfig(config);
  if (!config_status.IsOk()) {
    return config_status;
  }
  auto api_or = ResolveApi(*config.warp_library);
  if (!api_or.IsOk()) {
    return api_or.status();
  }
  const ApicApi api = api_or.value();
  const char* const actual_version = api.version();
  if (actual_version == nullptr || kSupportedWarpVersion != actual_version) {
    return Status{StatusCode::version_mismatch, "Warp version does not match the pinned artifact"};
  }
  if (config.expected_warp_version != kSupportedWarpVersion) {
    return Status{StatusCode::invalid_argument, "runtime configuration does not request the supported Warp version"};
  }
  GraphHandle graph(api.load_graph(nullptr, config.graph_path.c_str(), kApicDeviceCpu), GraphDeleter{api.destroy_graph});
  if (graph == nullptr) {
    return Status{StatusCode::artifact_error, "failed to load APIC graph"};
  }
  const Status parameter_status = ValidateGraphParameters(api, graph.get(), config);
  if (!parameter_status.IsOk()) {
    return parameter_status;
  }
  const Status kernel_status = RegisterGraphKernels(api, graph.get(), config.kernel_libraries);
  if (!kernel_status.IsOk()) {
    return kernel_status;
  }
  return std::unique_ptr<NewtonRuntime>(new NewtonRuntime(std::move(config), api, std::move(graph)));
}

Status NewtonRuntime::Reset(const std::span<const float> joint_positions,
                            const std::span<const float> joint_velocities) {
  if (joint_positions.size() != joint_positions_.size() || joint_velocities.size() != joint_velocities_.size()) {
    return {StatusCode::invalid_argument, "joint state count does not match graph configuration"};
  }
  if (!IsFinite(joint_positions) || !IsFinite(joint_velocities)) {
    return {StatusCode::non_finite_state, "joint reset state contains a non-finite value"};
  }
  std::copy(joint_positions.begin(), joint_positions.end(), joint_positions_.begin());
  std::copy(joint_velocities.begin(), joint_velocities.end(), joint_velocities_.begin());
  working_joint_positions_ = joint_positions_;
  working_joint_velocities_ = joint_velocities_;
  joint_targets_ = joint_positions_;
  joint_target_velocities_ = joint_velocities_;
  std::fill(joint_forces_.begin(), joint_forces_.end(), 0.0F);
  std::fill(joint_positions_out_.begin(), joint_positions_out_.end(), 0.0F);
  std::fill(joint_velocities_out_.begin(), joint_velocities_out_.end(), 0.0F);
  std::fill(body_transforms_.begin(), body_transforms_.end(), 0.0F);
  std::fill(working_body_transforms_.begin(), working_body_transforms_.end(), 0.0F);
  std::fill(body_transforms_out_.begin(), body_transforms_out_.end(), 0.0F);
  faulted_ = false;
  return Status::Ok();
}

Status NewtonRuntime::SetJointTargets(const std::span<const float> joint_positions,
                                      const std::span<const float> joint_velocities) {
  if (joint_positions.size() != joint_targets_.size() || joint_velocities.size() != joint_target_velocities_.size()) {
    return {StatusCode::invalid_argument, "joint target count does not match graph configuration"};
  }
  if (!IsFinite(joint_positions) || !IsFinite(joint_velocities)) {
    return {StatusCode::non_finite_state, "joint target contains a non-finite value"};
  }
  std::copy(joint_positions.begin(), joint_positions.end(), joint_targets_.begin());
  std::copy(joint_velocities.begin(), joint_velocities.end(), joint_target_velocities_.begin());
  return Status::Ok();
}

Status NewtonRuntime::SetGraphParameter(const char* const name, const std::span<const float> values) {
  if (!apic_.set_param(graph_.get(), name, values.data(), values.size_bytes())) {
    return {StatusCode::graph_error, "failed to set APIC graph parameter"};
  }
  return Status::Ok();
}

Status NewtonRuntime::GetGraphParameter(const char* const name, const std::span<float> values) {
  if (!apic_.get_param(graph_.get(), name, values.data(), values.size_bytes())) {
    return {StatusCode::graph_error, "failed to read APIC graph parameter"};
  }
  return Status::Ok();
}

Status NewtonRuntime::RecordFault(Status failure) {
  faulted_ = true;
  return failure;
}

Status NewtonRuntime::Step(const std::uint32_t substeps) {
  if (faulted_) {
    return {StatusCode::graph_error, "runtime is faulted; a successful reset is required"};
  }
  if (substeps < kMinimumSubsteps || substeps > kMaximumSubsteps) {
    return {StatusCode::invalid_argument, "substep count must be between one and twelve"};
  }
  working_joint_positions_ = joint_positions_;
  working_joint_velocities_ = joint_velocities_;
  working_body_transforms_ = body_transforms_;
  for (std::uint32_t substep = 0; substep < substeps; ++substep) {
    const std::array<std::pair<const char*, std::span<const float>>, 8> writes = {{
        {"joint_q_in", working_joint_positions_},
        {"joint_qd_in", working_joint_velocities_},
        {"joint_force", joint_forces_},
        {"joint_target_q", joint_targets_},
        {"joint_target_qd", joint_target_velocities_},
        {"joint_q_out", joint_positions_out_},
        {"joint_qd_out", joint_velocities_out_},
        {"body_q_out", working_body_transforms_},
    }};
    for (const auto& [name, values] : writes) {
      const Status write_status = SetGraphParameter(name, values);
      if (!write_status.IsOk()) {
        return RecordFault(write_status);
      }
    }
    if (!apic_.replay_graph(graph_.get())) {
      return RecordFault({StatusCode::graph_error, "APIC graph replay failed"});
    }
    const std::array<std::pair<const char*, std::span<float>>, 3> reads = {{
        {"joint_q_out", joint_positions_out_},
        {"joint_qd_out", joint_velocities_out_},
        {"body_q_out", body_transforms_out_},
    }};
    for (const auto& [name, values] : reads) {
      const Status read_status = GetGraphParameter(name, values);
      if (!read_status.IsOk()) {
        return RecordFault(read_status);
      }
    }
    if (!IsFinite(joint_positions_out_) || !IsFinite(joint_velocities_out_) || !IsFinite(body_transforms_out_)) {
      return RecordFault({StatusCode::non_finite_state, "APIC graph produced a non-finite state"});
    }
    working_joint_positions_ = joint_positions_out_;
    working_joint_velocities_ = joint_velocities_out_;
    working_body_transforms_ = body_transforms_out_;
  }
  joint_positions_ = working_joint_positions_;
  joint_velocities_ = working_joint_velocities_;
  body_transforms_ = working_body_transforms_;
  return Status::Ok();
}

}  // namespace quest_newton
