#pragma once
#include <array>
#include <cstdint>
#include <mutex>
#include <span>
#include <string_view>

namespace quest_newton {
inline constexpr std::size_t kArmBodies = 12;
using BodyValues = std::array<float, kArmBodies * 7>;
using Vec3 = std::array<float, 3>;
struct Pose { Vec3 position{}; std::array<float, 4> rotation{0, 0, 0, 1}; };
struct Mat4 {
    std::array<float, 16> m{}; // Row-major, column vectors; same layout as OVR::Matrix4f.
    static Mat4 Identity();
    bool operator==(const Mat4&) const = default;
};
Mat4 Multiply(const Mat4& a, const Mat4& b);
Mat4 PoseMatrix(const Pose& pose);
Vec3 TransformPoint(const Mat4& matrix, Vec3 point);
Mat4 ProvisionalStagePlacement();
const std::array<std::string_view, kArmBodies>& ArmBodyNames();
bool MakeVisualTransform(const Pose& pose, Vec3 scale, Mat4& body_from_mesh);
struct RenderState {
    std::array<float, 4> premultiplied_color{0, .35F, .35F, .35F};
    bool blend = true;
    std::uint32_t blend_equation = 0x8006, blend_source = 1, blend_destination = 0x0303;
    bool depth_test = true, depth_write = false;
    std::uint32_t depth_function = 0x0203;
};
RenderState ArmRenderState();
bool DecodeBodies(std::span<const float> values, std::array<Pose, kArmBodies>& poses);

class ArmScene {
  public:
    bool Update(std::span<const float> values, const Mat4& stage_from_base);
    const auto& Bodies() const { return bodies_; }
    const auto& Models() const { return models_; } // STAGE-from-body; mesh transforms remain separate.
    bool Valid() const { return valid_; }
    std::array<std::size_t, kArmBodies> BackToFront(
        const Mat4& center_view, const std::array<Vec3, kArmBodies>& body_local_centers = {}) const;
  private:
    std::array<Pose, kArmBodies> bodies_{};
    std::array<Mat4, kArmBodies> models_{};
    bool valid_ = false;
};

struct BodySnapshot {
    BodyValues values{};
    std::uint64_t generation = 0;
    bool valid = false;
    Mat4 stage_from_base = Mat4::Identity();
    bool placement_valid = false;
};
class SnapshotMailbox {
  public:
    bool Publish(std::span<const float> values, const Mat4* stage_from_base = nullptr);
    BodySnapshot Read() const;
  private:
    mutable std::mutex mutex_;
    BodySnapshot snapshot_;
};
} // namespace quest_newton

#if defined(__ANDROID__)
#include <android/asset_manager.h>
#include "Render/SurfaceRender.h"
namespace quest_newton {
class ArmRenderer {
  public:
    bool Init(AAssetManager* assets);
    void Shutdown();
    bool Update(const BodySnapshot& snapshot, const Mat4& stage_from_base);
    void Append(const OVR::Matrix4f& center_view, std::vector<OVRFW::ovrDrawSurface>& surfaces) const;
    bool Ready() const { return ready_; }
    void SetOpacity(float opacity) { color_ = {0, opacity, opacity, opacity}; }
    std::size_t VisualCount() const { return ready_ ? visual_count_ : 0; }
  private:
    ArmScene scene_;
    OVRFW::GlProgram program_;
    std::vector<OVRFW::ovrSurfaceDef> mesh_surfaces_;
    std::array<Mat4, kArmBodies> body_from_mesh_{};
    std::array<Vec3, kArmBodies> body_local_centers_{};
    std::array<int, kArmBodies> asset_indices_{};
    std::array<float, 4> color_ = ArmRenderState().premultiplied_color;
    std::size_t visual_count_ = 0;
    bool ready_ = false;
};
}
#endif
