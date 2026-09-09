#pragma once
#include <cstdint>
#include <functional>
#include <string>

namespace quest_newton {
struct PassthroughDispatch {
    std::function<bool(std::uint64_t&)> create_feature;
    std::function<bool(std::uint64_t, std::uint64_t&)> create_layer;
    std::function<bool(std::uint64_t)> destroy_layer;
    std::function<bool(std::uint64_t)> destroy_feature;
};
class PassthroughLayer {
  public:
    bool Start(PassthroughDispatch dispatch);
    bool Stop(const std::function<void()>& destroy_renderer);
    bool Ready() const { return feature_ != 0 && layer_ != 0; }
    std::uint64_t Handle() const { return layer_; }
    int ReserveUnderlay(int capacity, int& layer_count) const;
    const std::string& Error() const { return error_; }
  private:
    PassthroughDispatch dispatch_;
    std::uint64_t feature_ = 0, layer_ = 0;
    std::string error_;
};
} // namespace quest_newton
