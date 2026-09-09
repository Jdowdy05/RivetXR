#include "passthrough_layer.h"
#include <utility>
namespace quest_newton {
bool PassthroughLayer::Start(PassthroughDispatch dispatch) {
    if (Ready()) return true;
    if (!dispatch.create_feature || !dispatch.create_layer || !dispatch.destroy_layer || !dispatch.destroy_feature) {
        error_ = "passthrough_dispatch"; return false;
    }
    dispatch_ = std::move(dispatch);
    if (!dispatch_.create_feature(feature_) || !feature_) {
        Stop({}); error_ = "passthrough_create"; return false;
    }
    if (!dispatch_.create_layer(feature_, layer_) || !layer_) {
        Stop({}); error_ = "passthrough_layer_create"; return false;
    }
    error_.clear(); return true;
}
bool PassthroughLayer::Stop(const std::function<void()>& destroy_renderer) {
    if (destroy_renderer) destroy_renderer();
    bool result = true;
    if (layer_) { result = dispatch_.destroy_layer(layer_) && result; layer_ = 0; }
    if (feature_) { result = dispatch_.destroy_feature(feature_) && result; feature_ = 0; }
    if (!result) error_ = "passthrough_destroy";
    return result;
}
int PassthroughLayer::ReserveUnderlay(int capacity, int& layer_count) const {
    if (!Ready() || layer_count < 0 || capacity < 2 || layer_count >= capacity-1) return -1;
    return layer_count++;
}
} // namespace quest_newton
