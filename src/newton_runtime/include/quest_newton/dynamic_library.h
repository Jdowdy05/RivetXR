#pragma once

#include "quest_newton/status.h"

#include <memory>
#include <string_view>

namespace quest_newton {

class DynamicLibrary {
 public:
  virtual ~DynamicLibrary() = default;

  [[nodiscard]] virtual void* Resolve(std::string_view symbol) noexcept = 0;
};

class NativeDynamicLibrary final : public DynamicLibrary {
 public:
  enum class Visibility { local, global };

  ~NativeDynamicLibrary() override;

  NativeDynamicLibrary(const NativeDynamicLibrary&) = delete;
  NativeDynamicLibrary& operator=(const NativeDynamicLibrary&) = delete;

  [[nodiscard]] static StatusOr<std::unique_ptr<NativeDynamicLibrary>> Open(
      std::string_view path, Visibility visibility = Visibility::local);
  [[nodiscard]] void* Resolve(std::string_view symbol) noexcept override;

 private:
  explicit NativeDynamicLibrary(void* handle) noexcept : handle_(handle) {}

  void* handle_ = nullptr;
};

}  // namespace quest_newton
