#pragma once

#include "quest_newton/apic_api.h"
#include "quest_newton/dynamic_library.h"
#include "quest_newton/status.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace quest_newton {

struct GraphBufferConfig {
  std::string name;
  std::size_t bytes = 0;
};

struct RuntimeConfig {
  std::string expected_warp_version;
  std::string graph_path;
  std::size_t joint_count = 0;
  std::size_t body_count = 0;
  std::vector<GraphBufferConfig> required_buffers;
  std::unique_ptr<DynamicLibrary> warp_library;
  std::vector<std::unique_ptr<DynamicLibrary>> kernel_libraries;
};

class NewtonRuntime final {
 public:
  [[nodiscard]] static StatusOr<std::unique_ptr<NewtonRuntime>> Create(RuntimeConfig config);
  ~NewtonRuntime();

  NewtonRuntime(const NewtonRuntime&) = delete;
  NewtonRuntime& operator=(const NewtonRuntime&) = delete;

  [[nodiscard]] Status Reset(std::span<const float> joint_positions,
                             std::span<const float> joint_velocities);
  [[nodiscard]] Status SetJointTargets(std::span<const float> joint_positions,
                                       std::span<const float> joint_velocities);
  [[nodiscard]] Status Step(std::uint32_t substeps);

  [[nodiscard]] std::span<const float> JointPositions() const noexcept { return joint_positions_; }
  [[nodiscard]] std::span<const float> JointVelocities() const noexcept { return joint_velocities_; }
  [[nodiscard]] std::span<const float> BodyTransforms() const noexcept { return body_transforms_; }

 private:
  struct GraphDeleter {
    ApicDestroyGraphFunction destroy = nullptr;

    void operator()(ApicGraph* graph) const noexcept {
      if (graph != nullptr && destroy != nullptr) {
        destroy(graph);
      }
    }
  };

  using GraphHandle = std::unique_ptr<ApicGraph, GraphDeleter>;

  NewtonRuntime(RuntimeConfig config, ApicApi api, GraphHandle graph);

  [[nodiscard]] Status SetGraphParameter(const char* name, std::span<const float> values);
  [[nodiscard]] Status GetGraphParameter(const char* name, std::span<float> values);
  [[nodiscard]] Status RecordFault(Status failure);

  ApicApi apic_;
  std::unique_ptr<DynamicLibrary> warp_library_;
  std::vector<std::unique_ptr<DynamicLibrary>> kernel_libraries_;
  GraphHandle graph_{nullptr, GraphDeleter{}};
  std::vector<float> joint_positions_;
  std::vector<float> joint_velocities_;
  std::vector<float> body_transforms_;
  std::vector<float> working_joint_positions_;
  std::vector<float> working_joint_velocities_;
  std::vector<float> working_body_transforms_;
  std::vector<float> joint_targets_;
  std::vector<float> joint_target_velocities_;
  std::vector<float> joint_forces_;
  std::vector<float> joint_positions_out_;
  std::vector<float> joint_velocities_out_;
  std::vector<float> body_transforms_out_;
  bool faulted_ = false;
};

}  // namespace quest_newton
