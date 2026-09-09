#include "quest_newton/dynamic_library.h"

#include <memory>
#include <sstream>
#include <string>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace quest_newton {

NativeDynamicLibrary::~NativeDynamicLibrary() {
  if (handle_ == nullptr) {
    return;
  }
#if defined(_WIN32)
  static_cast<void>(FreeLibrary(static_cast<HMODULE>(handle_)));
#else
  static_cast<void>(dlclose(handle_));
#endif
  handle_ = nullptr;
}

StatusOr<std::unique_ptr<NativeDynamicLibrary>> NativeDynamicLibrary::Open(
    const std::string_view path, const Visibility visibility) {
  if (path.empty()) {
    return Status{StatusCode::invalid_argument, "dynamic library path is empty"};
  }
  const std::string owned_path(path);
#if defined(_WIN32)
  static_cast<void>(visibility);
  void* const handle = static_cast<void*>(LoadLibraryA(owned_path.c_str()));
#else
  static_cast<void>(dlerror());
  const int flags = RTLD_NOW | (visibility == Visibility::global ? RTLD_GLOBAL : RTLD_LOCAL);
  void* const handle = dlopen(owned_path.c_str(), flags);
#endif
  if (handle == nullptr) {
#if defined(_WIN32)
    const DWORD error = GetLastError();
    std::ostringstream message;
    message << "LoadLibrary failed for " << owned_path << " (win32 error " << error << ")";
    return Status{StatusCode::artifact_error, message.str()};
#else
    const char* const loader_error = dlerror();
    const std::string detail = loader_error == nullptr ? "unknown loader error" : loader_error;
    return Status{StatusCode::artifact_error, "dlopen failed for " + owned_path + ": " + detail};
#endif
  }
  return std::unique_ptr<NativeDynamicLibrary>(new NativeDynamicLibrary(handle));
}

void* NativeDynamicLibrary::Resolve(const std::string_view symbol) noexcept {
  if (handle_ == nullptr || symbol.empty()) {
    return nullptr;
  }
  const std::string owned_symbol(symbol);
#if defined(_WIN32)
  return reinterpret_cast<void*>(GetProcAddress(static_cast<HMODULE>(handle_), owned_symbol.c_str()));
#else
  return dlsym(handle_, owned_symbol.c_str());
#endif
}

}  // namespace quest_newton
