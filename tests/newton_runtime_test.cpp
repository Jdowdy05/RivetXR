#include "quest_newton/newton_runtime.h"
#include "quest_newton/dynamic_library.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using quest_newton::ApicGraph;
using quest_newton::DynamicLibrary;
using quest_newton::GraphBufferConfig;
using quest_newton::NewtonRuntime;
using quest_newton::NativeDynamicLibrary;
using quest_newton::RuntimeConfig;
using quest_newton::Status;
using quest_newton::StatusCode;

constexpr std::size_t kJointCount = 7;
constexpr std::size_t kBodyCount = 7;
constexpr std::size_t kJointBytes = kJointCount * sizeof(float);
constexpr std::size_t kBodyBytes = kBodyCount * 7U * sizeof(float);

struct Kernel {
  std::string key;
  std::string module_hash;
  std::string forward_symbol;
  std::string backward_symbol;
};

struct FakeApicState {
  std::string version = "1.18.0.dev2";
  bool fail_load = false;
  bool fail_set = false;
  bool fail_get = false;
  bool fail_replay = false;
  bool skip_graph_buffer_allocation = false;
  bool produce_non_finite_output = false;
  bool throw_on_register = false;
  int graph_destroys = 0;
  int library_destroys = 0;
  int kernel_registrations = 0;
  int replay_calls = 0;
  int write_calls = 0;
  int read_calls = 0;
  int fail_get_on_call = 0;
  int fail_replay_on_call = 0;
  std::vector<std::string> destruction_events;
  std::vector<Kernel> kernels = {
      {"first_kernel", "module_a", "kernel_a", ""},
      {"second_kernel", "module_b", "kernel_b", ""},
  };
  std::vector<GraphBufferConfig> parameters;
  std::unordered_map<std::string, std::vector<std::byte>> graph_buffers;
  std::vector<float> replay_input_first_joint;
};

FakeApicState* g_state = nullptr;

void Expect(const bool condition, const std::string_view message) {
  if (!condition) {
    std::fprintf(stderr, "FAILED: %.*s\n", static_cast<int>(message.size()), message.data());
    std::exit(EXIT_FAILURE);
  }
}

bool IsCode(const Status& status, const StatusCode code) {
  return status.code == code;
}

const char* FakeWarpVersion() {
  return g_state->version.c_str();
}

ApicGraph* FakeLoadGraph(void*, const char*, int) {
  if (g_state->fail_load) {
    return nullptr;
  }
  g_state->graph_buffers.clear();
  for (const auto& parameter : g_state->parameters) {
    const std::size_t bytes = g_state->skip_graph_buffer_allocation ? 0U : parameter.bytes;
    g_state->graph_buffers.emplace(parameter.name, std::vector<std::byte>(bytes));
  }
  return reinterpret_cast<ApicGraph*>(g_state);
}

void FakeDestroyGraph(ApicGraph*) {
  ++g_state->graph_destroys;
  g_state->destruction_events.emplace_back("graph");
}

int FakeGetNumKernels(ApicGraph*) {
  return static_cast<int>(g_state->kernels.size());
}

const char* FakeGetKernelKey(ApicGraph*, const int index) {
  return g_state->kernels.at(static_cast<std::size_t>(index)).key.c_str();
}

const char* FakeGetKernelModuleHash(ApicGraph*, const int index) {
  return g_state->kernels.at(static_cast<std::size_t>(index)).module_hash.c_str();
}

const char* FakeGetKernelForwardName(ApicGraph*, const int index) {
  return g_state->kernels.at(static_cast<std::size_t>(index)).forward_symbol.c_str();
}

const char* FakeGetKernelBackwardName(ApicGraph*, const int index) {
  return g_state->kernels.at(static_cast<std::size_t>(index)).backward_symbol.c_str();
}

void FakeRegisterLoadedCpuKernel(ApicGraph*, const char*, const char*, void*, void*) {
  ++g_state->kernel_registrations;
  if (g_state->throw_on_register) {
    throw std::runtime_error("fake kernel registration failed");
  }
}

int FakeGetNumParams(ApicGraph*) {
  return static_cast<int>(g_state->parameters.size());
}

const char* FakeGetParamName(ApicGraph*, const int index) {
  return g_state->parameters.at(static_cast<std::size_t>(index)).name.c_str();
}

std::size_t FakeGetParamSize(ApicGraph*, const char* name) {
  for (const auto& parameter : g_state->parameters) {
    if (parameter.name == name) {
      return parameter.bytes;
    }
  }
  return 0;
}

bool FakeSetParam(ApicGraph*, const char* name, const void* data, const std::size_t size) {
  ++g_state->write_calls;
  if (g_state->fail_set) {
    return false;
  }
  const auto buffer = g_state->graph_buffers.find(name);
  if (buffer == g_state->graph_buffers.end() || buffer->second.size() != size) {
    return false;
  }
  std::memcpy(buffer->second.data(), data, size);
  return true;
}

bool FakeGetParam(ApicGraph*, const char* name, void* data, const std::size_t size) {
  ++g_state->read_calls;
  if (g_state->fail_get ||
      (g_state->fail_get_on_call > 0 && g_state->read_calls == g_state->fail_get_on_call)) {
    return false;
  }
  const auto buffer = g_state->graph_buffers.find(name);
  if (buffer == g_state->graph_buffers.end() || buffer->second.size() != size) {
    return false;
  }
  std::memcpy(data, buffer->second.data(), size);
  return true;
}

float ReadGraphFloat(const std::string_view name, const std::size_t index) {
  float value = 0.0F;
  const auto& buffer = g_state->graph_buffers.at(std::string(name));
  std::memcpy(&value, buffer.data() + index * sizeof(float), sizeof(value));
  return value;
}

void WriteGraphFloat(const std::string_view name, const std::size_t index, const float value) {
  auto& buffer = g_state->graph_buffers.at(std::string(name));
  std::memcpy(buffer.data() + index * sizeof(float), &value, sizeof(value));
}

bool FakeReplayGraph(ApicGraph*) {
  ++g_state->replay_calls;
  if (g_state->fail_replay ||
      (g_state->fail_replay_on_call > 0 && g_state->replay_calls == g_state->fail_replay_on_call)) {
    return false;
  }
  g_state->replay_input_first_joint.push_back(ReadGraphFloat("joint_q_in", 0));
  for (std::size_t index = 0; index < kJointCount; ++index) {
    WriteGraphFloat("joint_q_out", index, ReadGraphFloat("joint_q_in", index) + 1.0F);
    WriteGraphFloat("joint_qd_out", index, ReadGraphFloat("joint_qd_in", index) + 2.0F);
  }
  for (std::size_t index = 0; index < kBodyCount * 7U; ++index) {
    WriteGraphFloat("body_q_out", index, static_cast<float>(index));
  }
  if (g_state->produce_non_finite_output) {
    WriteGraphFloat("joint_q_out", 0, std::numeric_limits<float>::quiet_NaN());
  }
  return true;
}

void KernelA() {}
void KernelB() {}

class FakeDynamicLibrary final : public DynamicLibrary {
 public:
  FakeDynamicLibrary(std::shared_ptr<FakeApicState> state, std::string destruction_name)
      : state_(std::move(state)), destruction_name_(std::move(destruction_name)) {}

  ~FakeDynamicLibrary() override {
    ++state_->library_destroys;
    state_->destruction_events.push_back(destruction_name_);
  }

  void Add(const std::string_view symbol, void* address) {
    symbols_.emplace(std::string(symbol), address);
  }

  [[nodiscard]] void* Resolve(const std::string_view symbol) noexcept override {
    const auto found = symbols_.find(std::string(symbol));
    return found == symbols_.end() ? nullptr : found->second;
  }

 private:
  std::shared_ptr<FakeApicState> state_;
  std::string destruction_name_;
  std::unordered_map<std::string, void*> symbols_;
};

template <typename Function>
void AddFunction(FakeDynamicLibrary& library, const std::string_view symbol, Function function) {
  library.Add(symbol, reinterpret_cast<void*>(function));
}

std::vector<GraphBufferConfig> RequiredBuffers(const std::size_t joint_count = kJointCount,
                                               const std::size_t body_count = kBodyCount) {
  const std::size_t joint_bytes = joint_count * sizeof(float);
  const std::size_t body_bytes = body_count * 7U * sizeof(float);
  return {
      {"joint_q_in", joint_bytes},       {"joint_qd_in", joint_bytes},
      {"joint_force", joint_bytes},      {"joint_target_q", joint_bytes},
      {"joint_target_qd", joint_bytes},  {"joint_q_out", joint_bytes},
      {"joint_qd_out", joint_bytes},     {"body_q_out", body_bytes},
  };
}

std::unique_ptr<FakeDynamicLibrary> MakeWarpLibrary(const std::shared_ptr<FakeApicState>& state,
                                                     const std::string_view omitted_symbol = "") {
  auto library = std::make_unique<FakeDynamicLibrary>(state, "warp");
  const auto add = [&library, omitted_symbol](const std::string_view name, auto function) {
    if (name != omitted_symbol) {
      AddFunction(*library, name, function);
    }
  };
  add("wp_version", &FakeWarpVersion);
  add("wp_apic_load_graph", &FakeLoadGraph);
  add("wp_apic_destroy_graph", &FakeDestroyGraph);
  add("wp_apic_get_num_kernels", &FakeGetNumKernels);
  add("wp_apic_get_kernel_key", &FakeGetKernelKey);
  add("wp_apic_get_kernel_module_hash", &FakeGetKernelModuleHash);
  add("wp_apic_get_kernel_forward_name", &FakeGetKernelForwardName);
  add("wp_apic_get_kernel_backward_name", &FakeGetKernelBackwardName);
  add("wp_apic_register_loaded_cpu_kernel", &FakeRegisterLoadedCpuKernel);
  add("wp_apic_get_num_params", &FakeGetNumParams);
  add("wp_apic_get_param_name", &FakeGetParamName);
  add("wp_apic_get_param_size", &FakeGetParamSize);
  add("wp_apic_set_param", &FakeSetParam);
  add("wp_apic_get_param", &FakeGetParam);
  add("wp_apic_cpu_replay_graph", &FakeReplayGraph);
  return library;
}

std::unique_ptr<FakeDynamicLibrary> MakeKernelLibrary(const std::shared_ptr<FakeApicState>& state,
                                                       const bool omit_second_kernel = false) {
  auto library = std::make_unique<FakeDynamicLibrary>(state, "kernel");
  AddFunction(*library, "kernel_a", &KernelA);
  if (!omit_second_kernel) {
    AddFunction(*library, "kernel_b", &KernelB);
  }
  return library;
}

RuntimeConfig MakeConfig(const std::shared_ptr<FakeApicState>& state) {
  state->parameters = RequiredBuffers();
  g_state = state.get();
  RuntimeConfig config;
  config.expected_warp_version = "1.18.0.dev2";
  config.graph_path = "fake.wrp";
  config.joint_count = kJointCount;
  config.body_count = kBodyCount;
  config.required_buffers = RequiredBuffers();
  config.warp_library = MakeWarpLibrary(state);
  config.kernel_libraries.push_back(MakeKernelLibrary(state));
  return config;
}

std::unique_ptr<NewtonRuntime> CreateRuntime(const std::shared_ptr<FakeApicState>& state) {
  auto created = NewtonRuntime::Create(MakeConfig(state));
  Expect(created.IsOk(), "runtime creation should succeed");
  return std::move(created).value();
}

void TestCreatesWithEightRequiredBuffers() {
  const auto state = std::make_shared<FakeApicState>();
  auto runtime = CreateRuntime(state);
  Expect(runtime->JointPositions().size() == kJointCount, "joint positions should be sized from config");
  Expect(runtime->JointVelocities().size() == kJointCount, "joint velocities should be sized from config");
  Expect(runtime->BodyTransforms().size() == kBodyCount * 7U, "body transforms should be sized from config");
}

void TestRejectsMissingApicFunction() {
  const auto state = std::make_shared<FakeApicState>();
  auto config = MakeConfig(state);
  config.warp_library = MakeWarpLibrary(state, "wp_apic_get_param");
  const auto created = NewtonRuntime::Create(std::move(config));
  Expect(!created.IsOk() && IsCode(created.status(), StatusCode::symbol_missing),
         "missing APIC function should be rejected");
}

void TestRejectsWrongWarpVersion() {
  const auto state = std::make_shared<FakeApicState>();
  state->version = "not-the-pinned-version";
  const auto created = NewtonRuntime::Create(MakeConfig(state));
  Expect(!created.IsOk() && IsCode(created.status(), StatusCode::version_mismatch),
         "wrong Warp version should be rejected");
}

void TestRejectsMatchingUnsupportedWarpVersion() {
  const auto state = std::make_shared<FakeApicState>();
  state->version = "1.17.0";
  auto config = MakeConfig(state);
  config.expected_warp_version = state->version;
  const auto created = NewtonRuntime::Create(std::move(config));
  Expect(!created.IsOk() && IsCode(created.status(), StatusCode::version_mismatch),
         "matching caller and library versions must still reject an unsupported Warp version");
}

void TestRejectsWrongGraphParameterBytes() {
  const auto state = std::make_shared<FakeApicState>();
  auto config = MakeConfig(state);
  state->parameters[0].bytes = kJointBytes - sizeof(float);
  const auto created = NewtonRuntime::Create(std::move(config));
  Expect(!created.IsOk() && IsCode(created.status(), StatusCode::artifact_error),
         "wrong graph parameter byte size should be rejected");
}

void TestRejectsMissingKernelSymbol() {
  const auto state = std::make_shared<FakeApicState>();
  auto config = MakeConfig(state);
  config.kernel_libraries.clear();
  config.kernel_libraries.push_back(MakeKernelLibrary(state, true));
  const auto created = NewtonRuntime::Create(std::move(config));
  Expect(!created.IsOk() && IsCode(created.status(), StatusCode::symbol_missing),
         "missing graph kernel symbol should be rejected");
  Expect(state->kernel_registrations == 0,
         "a missing later kernel symbol must not register an earlier kernel");
}

void TestRejectsNanTargets() {
  const auto state = std::make_shared<FakeApicState>();
  auto runtime = CreateRuntime(state);
  std::array<float, kJointCount> targets{};
  std::array<float, kJointCount> target_velocities{};
  targets[3] = std::numeric_limits<float>::quiet_NaN();
  const auto status = runtime->SetJointTargets(targets, target_velocities);
  Expect(IsCode(status, StatusCode::non_finite_state), "NaN targets should be rejected");
}

void TestRejectsWrongJointCounts() {
  const auto state = std::make_shared<FakeApicState>();
  auto runtime = CreateRuntime(state);
  std::array<float, kJointCount - 1U> positions{};
  std::array<float, kJointCount> velocities{};
  const auto status = runtime->Reset(positions, velocities);
  Expect(IsCode(status, StatusCode::invalid_argument), "wrong joint count should be rejected");
}

void TestRejectsOutOfRangeSubsteps() {
  const auto state = std::make_shared<FakeApicState>();
  auto runtime = CreateRuntime(state);
  Expect(IsCode(runtime->Step(0), StatusCode::invalid_argument), "zero substeps should be rejected");
  Expect(IsCode(runtime->Step(13), StatusCode::invalid_argument), "thirteen substeps should be rejected");
}

void TestResetRestoresInitialState() {
  const auto state = std::make_shared<FakeApicState>();
  auto runtime = CreateRuntime(state);
  const std::array<float, kJointCount> positions = {0.0F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F};
  const std::array<float, kJointCount> velocities = {7.0F, 6.0F, 5.0F, 4.0F, 3.0F, 2.0F, 1.0F};
  Expect(runtime->Reset(positions, velocities).IsOk(), "reset should accept finite initial state");
  Expect(runtime->Step(1).IsOk(), "step should advance the fake graph");
  Expect(runtime->Reset(positions, velocities).IsOk(), "second reset should succeed");
  Expect(runtime->JointPositions()[0] == 0.0F && runtime->JointPositions()[6] == 6.0F,
         "reset should restore joint positions");
  Expect(runtime->JointVelocities()[0] == 7.0F && runtime->JointVelocities()[6] == 1.0F,
         "reset should restore joint velocities");
}

void TestStepFeedsOutputsBackToNextInput() {
  const auto state = std::make_shared<FakeApicState>();
  auto runtime = CreateRuntime(state);
  const std::array<float, kJointCount> positions = {10.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  const std::array<float, kJointCount> velocities = {20.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  Expect(runtime->Reset(positions, velocities).IsOk(), "reset should prepare step state");
  Expect(runtime->Step(2).IsOk(), "two substeps should replay");
  Expect(state->replay_input_first_joint.size() == 2U, "fake graph should observe two inputs");
  Expect(state->replay_input_first_joint[0] == 10.0F && state->replay_input_first_joint[1] == 11.0F,
         "each replay should receive the prior output as its input");
  Expect(runtime->JointPositions()[0] == 12.0F && runtime->JointVelocities()[0] == 24.0F,
         "runtime should expose the final replay output");
}

void TestLaterReplayFailureDoesNotPublishPartialStep() {
  const auto state = std::make_shared<FakeApicState>();
  auto runtime = CreateRuntime(state);
  const std::array<float, kJointCount> positions = {11.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  const std::array<float, kJointCount> velocities = {12.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  Expect(runtime->Reset(positions, velocities).IsOk(), "reset should establish an atomic-step baseline");
  state->fail_replay_on_call = 2;
  Expect(IsCode(runtime->Step(2), StatusCode::graph_error), "second replay failure should fault the runtime");
  Expect(runtime->JointPositions()[0] == 11.0F && runtime->JointVelocities()[0] == 12.0F,
         "a later replay failure must not publish a successful earlier substep");
  Expect(runtime->BodyTransforms()[0] == 0.0F, "a later replay failure must not publish body output");
}

void TestLaterReadbackFailureDoesNotPublishPartialStep() {
  const auto state = std::make_shared<FakeApicState>();
  auto runtime = CreateRuntime(state);
  const std::array<float, kJointCount> positions = {13.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  const std::array<float, kJointCount> velocities = {14.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  Expect(runtime->Reset(positions, velocities).IsOk(), "reset should establish a readback baseline");
  state->fail_get_on_call = 4;
  Expect(IsCode(runtime->Step(2), StatusCode::graph_error), "second-substep readback failure should fault the runtime");
  Expect(runtime->JointPositions()[0] == 13.0F && runtime->JointVelocities()[0] == 14.0F,
         "a later readback failure must not publish a successful earlier substep");
  Expect(runtime->BodyTransforms()[0] == 0.0F, "a later readback failure must not publish body output");
}

void TestFailedWritesFaultUntilResetRecovery() {
  const auto state = std::make_shared<FakeApicState>();
  auto runtime = CreateRuntime(state);
  const std::array<float, kJointCount> positions = {3.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  const std::array<float, kJointCount> velocities = {4.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  Expect(runtime->Reset(positions, velocities).IsOk(), "reset should establish a recoverable state");
  state->fail_set = true;
  Expect(IsCode(runtime->Step(1), StatusCode::graph_error), "failed graph write should fault the runtime");
  Expect(state->replay_calls == 0, "failed graph write must not replay the graph");
  const int writes_after_fault = state->write_calls;
  Expect(IsCode(runtime->Step(1), StatusCode::graph_error), "faulted runtime must reject another step");
  Expect(state->write_calls == writes_after_fault, "faulted runtime must not write graph parameters");
  state->fail_set = false;
  Expect(runtime->Reset(positions, velocities).IsOk(), "successful reset should clear the fault");
  Expect(runtime->Step(1).IsOk(), "successful reset should permit replay recovery");
}

void TestReplayFailureFaultsWithoutPublishingState() {
  const auto state = std::make_shared<FakeApicState>();
  auto runtime = CreateRuntime(state);
  const std::array<float, kJointCount> positions = {5.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  const std::array<float, kJointCount> velocities = {6.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  Expect(runtime->Reset(positions, velocities).IsOk(), "reset should establish replay inputs");
  state->fail_replay = true;
  Expect(IsCode(runtime->Step(1), StatusCode::graph_error), "failed replay should fault the runtime");
  Expect(runtime->JointPositions()[0] == 5.0F && runtime->JointVelocities()[0] == 6.0F,
         "failed replay must preserve published joint state");
  const int replays_after_fault = state->replay_calls;
  Expect(IsCode(runtime->Step(1), StatusCode::graph_error), "faulted runtime must reject a replay");
  Expect(state->replay_calls == replays_after_fault, "faulted runtime must not replay again");
}

void TestReadbackFailureFaultsWithoutPublishingState() {
  const auto state = std::make_shared<FakeApicState>();
  auto runtime = CreateRuntime(state);
  const std::array<float, kJointCount> positions = {7.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  const std::array<float, kJointCount> velocities = {8.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  Expect(runtime->Reset(positions, velocities).IsOk(), "reset should establish readback inputs");
  state->fail_get = true;
  Expect(IsCode(runtime->Step(1), StatusCode::graph_error), "failed readback should fault the runtime");
  Expect(runtime->JointPositions()[0] == 7.0F && runtime->JointVelocities()[0] == 8.0F,
         "failed readback must preserve published joint state");
  const int reads_after_fault = state->read_calls;
  Expect(IsCode(runtime->Step(1), StatusCode::graph_error), "faulted runtime must reject another readback");
  Expect(state->read_calls == reads_after_fault, "faulted runtime must not read graph parameters");
}

void TestNonFiniteOutputFaultsWithoutPublishingState() {
  const auto state = std::make_shared<FakeApicState>();
  auto runtime = CreateRuntime(state);
  const std::array<float, kJointCount> positions = {9.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  const std::array<float, kJointCount> velocities = {10.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  Expect(runtime->Reset(positions, velocities).IsOk(), "reset should establish output inputs");
  state->produce_non_finite_output = true;
  Expect(IsCode(runtime->Step(1), StatusCode::non_finite_state), "non-finite output should fault the runtime");
  Expect(runtime->JointPositions()[0] == 9.0F && runtime->JointVelocities()[0] == 10.0F,
         "non-finite readback must not publish state");
  Expect(runtime->BodyTransforms()[0] == 0.0F, "non-finite readback must not publish body transforms");
  const int replays_after_fault = state->replay_calls;
  Expect(IsCode(runtime->Step(1), StatusCode::graph_error), "faulted runtime must reject another replay");
  Expect(state->replay_calls == replays_after_fault, "faulted runtime must not replay after non-finite output");
}

void TestCreationExceptionReleasesGraphBeforeLibraries() {
  const auto state = std::make_shared<FakeApicState>();
  state->throw_on_register = true;
  bool threw = false;
  try {
    static_cast<void>(NewtonRuntime::Create(MakeConfig(state)));
  } catch (const std::runtime_error&) {
    threw = true;
  }
  Expect(threw, "fake registration exception should escape runtime creation");
  const std::vector<std::string> expected = {"graph", "kernel", "warp"};
  Expect(state->destruction_events == expected,
         "creation exception must destroy graph before kernel and Warp libraries");
}

void TestConstructorAllocationFailureReleasesGraphBeforeLibraries() {
  const auto state = std::make_shared<FakeApicState>();
  auto config = MakeConfig(state);
  const std::size_t oversized_joint_count = std::numeric_limits<std::size_t>::max() / sizeof(float);
  config.joint_count = oversized_joint_count;
  config.body_count = 1;
  config.required_buffers = RequiredBuffers(oversized_joint_count, config.body_count);
  state->parameters = config.required_buffers;
  state->skip_graph_buffer_allocation = true;
  bool threw = false;
  try {
    static_cast<void>(NewtonRuntime::Create(std::move(config)));
  } catch (const std::exception&) {
    threw = true;
  }
  Expect(threw, "oversized runtime state should fail after graph ownership is adopted");
  const std::vector<std::string> expected = {"graph", "kernel", "warp"};
  Expect(state->destruction_events == expected,
         "constructor allocation failure must destroy graph before kernel and Warp libraries");
}

void TestDestructionReleasesGraphThenLibrariesOnce() {
  const auto state = std::make_shared<FakeApicState>();
  {
    auto runtime = CreateRuntime(state);
    Expect(runtime->Step(1).IsOk(), "runtime should have a live graph before destruction");
  }
  const std::vector<std::string> expected = {"graph", "kernel", "warp"};
  Expect(state->graph_destroys == 1 && state->library_destroys == 2,
         "graph and owned libraries should each be released exactly once");
  Expect(state->destruction_events == expected, "runtime should destroy graph before kernel and Warp libraries");
}

void TestNativeLoaderPreservesPlatformDiagnostic() {
  const auto result = NativeDynamicLibrary::Open(
      "quest_newton_missing_library_for_contract_test.so", NativeDynamicLibrary::Visibility::global);
  Expect(!result.IsOk() && IsCode(result.status(), StatusCode::artifact_error),
         "missing native library should return an artifact error");
#if defined(_WIN32)
  Expect(result.status().message.find("LoadLibrary") != std::string::npos,
         "Windows loader diagnostic should name LoadLibrary");
#else
  Expect(result.status().message.find("dlopen") != std::string::npos,
         "POSIX loader diagnostic should name dlopen");
#endif
}

}  // namespace

int main() {
  TestCreatesWithEightRequiredBuffers();
  TestRejectsMissingApicFunction();
  TestRejectsWrongWarpVersion();
  TestRejectsMatchingUnsupportedWarpVersion();
  TestRejectsWrongGraphParameterBytes();
  TestRejectsMissingKernelSymbol();
  TestRejectsNanTargets();
  TestRejectsWrongJointCounts();
  TestRejectsOutOfRangeSubsteps();
  TestResetRestoresInitialState();
  TestStepFeedsOutputsBackToNextInput();
  TestLaterReplayFailureDoesNotPublishPartialStep();
  TestLaterReadbackFailureDoesNotPublishPartialStep();
  TestFailedWritesFaultUntilResetRecovery();
  TestReplayFailureFaultsWithoutPublishingState();
  TestReadbackFailureFaultsWithoutPublishingState();
  TestNonFiniteOutputFaultsWithoutPublishingState();
  TestCreationExceptionReleasesGraphBeforeLibraries();
  TestConstructorAllocationFailureReleasesGraphBeforeLibraries();
  TestDestructionReleasesGraphThenLibrariesOnce();
  TestNativeLoaderPreservesPlatformDiagnostic();
  return EXIT_SUCCESS;
}
