#include "arm_renderer.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace quest_newton {
Mat4 Mat4::Identity() { Mat4 result; result.m[0] = result.m[5] = result.m[10] = result.m[15] = 1; return result; }
Mat4 Multiply(const Mat4& a, const Mat4& b) {
    Mat4 result;
    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t col = 0; col < 4; ++col)
            for (std::size_t k = 0; k < 4; ++k) result.m[row*4+col] += a.m[row*4+k] * b.m[k*4+col];
    return result;
}
Mat4 PoseMatrix(const Pose& pose) {
    const auto [x,y,z,w] = pose.rotation;
    Mat4 result = Mat4::Identity();
    result.m[0] = 1-2*(y*y+z*z); result.m[1] = 2*(x*y-z*w); result.m[2] = 2*(x*z+y*w);
    result.m[4] = 2*(x*y+z*w); result.m[5] = 1-2*(x*x+z*z); result.m[6] = 2*(y*z-x*w);
    result.m[8] = 2*(x*z-y*w); result.m[9] = 2*(y*z+x*w); result.m[10] = 1-2*(x*x+y*y);
    result.m[3] = pose.position[0]; result.m[7] = pose.position[1]; result.m[11] = pose.position[2];
    return result;
}
Vec3 TransformPoint(const Mat4& matrix, Vec3 point) {
    Vec3 result{};
    for (std::size_t row = 0; row < 3; ++row) {
        result[row] = matrix.m[row*4+3];
        for (std::size_t k = 0; k < 3; ++k) result[row] += matrix.m[row*4+k]*point[k];
    }
    return result;
}
Mat4 ProvisionalStagePlacement() { return PoseMatrix({{0,0,-1.2F},{-.5F,.5F,.5F,.5F}}); }
const std::array<std::string_view, kArmBodies>& ArmBodyNames() {
    static constexpr std::array<std::string_view, kArmBodies> names{
        "panda_link0", "panda_link1", "panda_link2", "panda_link3", "panda_link4", "panda_link5",
        "panda_link6", "panda_link7", "panda_link8", "panda_hand", "panda_leftfinger", "panda_rightfinger"};
    return names;
}
bool MakeVisualTransform(const Pose& pose, Vec3 scale, Mat4& body_from_mesh) {
    const auto finite = [](float value) { return std::isfinite(value); };
    if (!std::all_of(pose.position.begin(), pose.position.end(), finite) ||
        !std::all_of(pose.rotation.begin(), pose.rotation.end(), finite) ||
        !std::all_of(scale.begin(), scale.end(), [](float value) { return std::isfinite(value) && value > 0; })) return false;
    double norm = 0;
    for (const double value : pose.rotation) norm += value * value;
    if (std::abs(norm - 1) > 1e-5) return false;
    Mat4 candidate = PoseMatrix(pose);
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column) candidate.m[row*4+column] *= scale[column];
    if (!std::all_of(candidate.m.begin(), candidate.m.end(), finite)) return false;
    body_from_mesh = candidate;
    return true;
}
RenderState ArmRenderState() { return {}; }
bool DecodeBodies(std::span<const float> values, std::array<Pose, kArmBodies>& poses) {
    if (values.size() != 84 || !std::all_of(values.begin(), values.end(), [](float value) { return std::isfinite(value); })) return false;
    std::array<Pose, kArmBodies> candidate;
    for (std::size_t i = 0; i < kArmBodies; ++i) {
        std::copy_n(values.begin() + static_cast<std::ptrdiff_t>(i*7), 3, candidate[i].position.begin());
        double norm = 0;
        for (std::size_t j = 0; j < 4; ++j) { const double v = values[i*7+3+j]; norm += v*v; }
        if (norm < 1e-12) return false;
        for (std::size_t j = 0; j < 4; ++j) candidate[i].rotation[j] = static_cast<float>(values[i*7+3+j] / std::sqrt(norm));
    }
    poses = candidate; return true;
}
bool ArmScene::Update(std::span<const float> values, const Mat4& stage_from_base) {
    std::array<Pose, kArmBodies> poses;
    if (!DecodeBodies(values, poses)) return false;
    std::array<Mat4, kArmBodies> models;
    for (std::size_t i = 0; i < kArmBodies; ++i) {
        models[i] = Multiply(stage_from_base, PoseMatrix(poses[i]));
        if (!std::all_of(models[i].m.begin(), models[i].m.end(), [](float value){return std::isfinite(value);})) return false;
    }
    bodies_ = poses; models_ = models; valid_ = true; return true;
}
std::array<std::size_t, kArmBodies> ArmScene::BackToFront(
    const Mat4& center_view, const std::array<Vec3, kArmBodies>& body_local_centers) const {
    std::array<std::size_t, kArmBodies> order; std::iota(order.begin(), order.end(), 0U);
    std::array<float, kArmBodies> depth{};
    for (std::size_t i = 0; i < kArmBodies; ++i)
        depth[i] = TransformPoint(center_view, TransformPoint(models_[i], body_local_centers[i]))[2];
    if (std::all_of(depth.begin(), depth.end(), [](float value){return std::isfinite(value);})) {
        // Twelve entries: stable insertion sort avoids a per-frame temporary allocation.
        for (std::size_t i = 1; i < order.size(); ++i) {
            const auto entry = order[i];
            auto j = i;
            while (j > 0 && depth[entry] < depth[order[j-1]]) { order[j] = order[j-1]; --j; }
            order[j] = entry;
        }
    }
    return order;
}
bool SnapshotMailbox::Publish(std::span<const float> values, const Mat4* stage_from_base) {
    BodySnapshot candidate;
    if (values.size() != candidate.values.size()) return false;
    std::copy(values.begin(), values.end(), candidate.values.begin());
    std::array<Pose, kArmBodies> validated;
    if (!DecodeBodies(candidate.values, validated)) return false;
    if (stage_from_base != nullptr) {
        if (!std::all_of(stage_from_base->m.begin(), stage_from_base->m.end(), [](float v) { return std::isfinite(v); })) return false;
        candidate.stage_from_base = *stage_from_base;
        candidate.placement_valid = true;
    }
    candidate.valid = true;
    const std::lock_guard lock(mutex_);
    if (snapshot_.generation == std::numeric_limits<std::uint64_t>::max()) return false;
    candidate.generation = snapshot_.generation + 1;
    snapshot_ = candidate; return true;
}
BodySnapshot SnapshotMailbox::Read() const { const std::lock_guard lock(mutex_); return snapshot_; }
} // namespace quest_newton

#if defined(__ANDROID__)
#include "franka_meshes.h"
#include "mesh_geometry.h"
#include "Render/GlGeometry.h"
#include <GLES3/gl3.h>
#include <memory>
#include <stdexcept>

namespace quest_newton {
namespace {
constexpr std::array<std::string_view, 10> kAssetNames{
    "link0.qmsh", "link1.qmsh", "link2.qmsh", "link3.qmsh", "link4.qmsh",
    "link5.qmsh", "link6.qmsh", "link7.qmsh", "hand.qmsh", "finger.qmsh"};
constexpr std::array<int, kArmBodies> kBodyAssetIndices{0,1,2,3,4,5,6,7,-1,8,9,9};
constexpr std::uint64_t kGeometryByteLimit = 12 * 1024 * 1024;

bool MeshMetadataValid(const generated_meshes::MeshAsset& asset, std::size_t index) {
    if (!asset.name || std::string_view(asset.name) != kAssetNames[index] ||
        asset.vertex_count < 3 || asset.vertex_count >= 65536 ||
        asset.index_count == 0 || asset.index_count % 3 != 0 ||
        asset.index_count > OVRFW::GlGeometry::kMaxGeometryIndices ||
        asset.byte_count != 20ULL + 12ULL * asset.vertex_count + 2ULL * asset.index_count) return false;
    for (std::size_t axis = 0; axis < 3; ++axis)
        if (!std::isfinite(asset.bounds_min[axis]) || !std::isfinite(asset.bounds_max[axis]) ||
            asset.bounds_min[axis] > asset.bounds_max[axis]) return false;
    return true;
}

bool LoadMesh(AAssetManager* manager, const generated_meshes::MeshAsset& expected,
              MeshGeometryData& geometry) {
    const std::string path = std::string("franka/") + expected.name;
    const std::unique_ptr<AAsset, decltype(&AAsset_close)> asset(
        AAssetManager_open(manager, path.c_str(), AASSET_MODE_BUFFER), &AAsset_close);
    if (!asset) return false;
    const auto length = AAsset_getLength64(asset.get());
    if (length < 0 || static_cast<std::uint64_t>(length) != expected.byte_count) return false;
    const auto* bytes = static_cast<const std::byte*>(AAsset_getBuffer(asset.get()));
    if (!bytes) return false;
    std::string error;
    if (!DecodeQmsh({bytes, static_cast<std::size_t>(length)}, geometry, error) ||
        geometry.positions.size() != expected.vertex_count || geometry.indices.size() != expected.index_count) return false;
    auto minimum = geometry.positions.front();
    auto maximum = minimum;
    for (const auto& point : geometry.positions)
        for (std::size_t axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(minimum[axis], point[axis]);
            maximum[axis] = std::max(maximum[axis], point[axis]);
        }
    return minimum == expected.bounds_min && maximum == expected.bounds_max;
}
} // namespace

bool ArmRenderer::Init(AAssetManager* assets) {
    if (ready_) return true;
    const auto fail = [this] { Shutdown(); return false; };
    if (!assets || generated_meshes::kMeshBodies.size() != kArmBodies ||
        generated_meshes::kMeshAssets.size() != kAssetNames.size() ||
        generated_meshes::kVisualBodyCount != 11) return fail();
    try {
        std::uint64_t total_bytes = 0;
        for (std::size_t i = 0; i < generated_meshes::kMeshAssets.size(); ++i) {
            const auto& mesh = generated_meshes::kMeshAssets[i];
            if (!MeshMetadataValid(mesh, i) || mesh.byte_count >= kGeometryByteLimit - total_bytes) return fail();
            total_bytes += mesh.byte_count;
        }
        visual_count_ = 0;
        for (std::size_t i = 0; i < kArmBodies; ++i) {
            const auto& body = generated_meshes::kMeshBodies[i];
            if (!body.name || std::string_view(body.name) != ArmBodyNames()[i] ||
                body.asset_index != kBodyAssetIndices[i] ||
                !MakeVisualTransform({body.position, body.rotation}, body.scale, body_from_mesh_[i])) return fail();
            asset_indices_[i] = body.asset_index;
            if (body.asset_index < 0) continue; // Empty URDF panda_link8 still has a physics pose.
            const auto& mesh = generated_meshes::kMeshAssets[static_cast<std::size_t>(body.asset_index)];
            Vec3 center;
            for (std::size_t axis = 0; axis < 3; ++axis)
                center[axis] = .5F * mesh.bounds_min[axis] + .5F * mesh.bounds_max[axis];
            body_local_centers_[i] = TransformPoint(body_from_mesh_[i], center);
            if (!std::all_of(body_local_centers_[i].begin(), body_local_centers_[i].end(),
                             [](float value) { return std::isfinite(value); })) return fail();
            ++visual_count_;
        }
        if (visual_count_ != generated_meshes::kVisualBodyCount) return fail();
        const char* vertex = "attribute highp vec4 Position; void main(){ gl_Position = TransformVertex(Position); }";
        const char* fragment = "precision mediump float; uniform lowp vec4 Color; void main(){ gl_FragColor = Color; }";
        const OVRFW::ovrProgramParm uniforms[] = {{"Color", OVRFW::ovrProgramParmType::FLOAT_VECTOR4}};
        program_ = OVRFW::GlProgram::Build(vertex, fragment, uniforms, 1,
            OVRFW::GlProgram::GLSL_PROGRAM_VERSION, false);
        if (!program_.IsValid()) return fail();
        mesh_surfaces_.resize(generated_meshes::kMeshAssets.size());
        const auto state = ArmRenderState();
        for (std::size_t i = 0; i < mesh_surfaces_.size(); ++i) {
            const auto& mesh = generated_meshes::kMeshAssets[i];
            MeshGeometryData geometry;
            if (!LoadMesh(assets, mesh, geometry)) return fail();
            OVRFW::VertexAttribs attributes;
            attributes.position.reserve(geometry.positions.size());
            for (const auto& point : geometry.positions) attributes.position.emplace_back(point[0], point[1], point[2]);
            auto& surface = mesh_surfaces_[i];
            // DAE node transforms were baked by the host converter. Upload once,
            // without the framework's optional ambient geometry transform.
            const OVRFW::GlGeometry::TransformScope no_transform(OVR::Matrix4f::Identity(), false);
            surface.geo.Create(attributes, geometry.indices);
            if (glGetError() != GL_NO_ERROR || surface.geo.vertexBuffer == 0 || surface.geo.indexBuffer == 0 ||
                surface.geo.vertexArrayObject == 0 || surface.geo.vertexCount != static_cast<int>(mesh.vertex_count) ||
                surface.geo.indexCount != static_cast<int>(mesh.index_count)) return fail();
            surface.surfaceName = mesh.name;
            auto& command = surface.graphicsCommand;
            command.Program = program_; command.UniformData[0].Data = color_.data();
            command.GpuState.blendEnable = OVRFW::ovrGpuState::BLEND_ENABLE;
            command.GpuState.blendMode = state.blend_equation;
            command.GpuState.blendSrc = state.blend_source; command.GpuState.blendDst = state.blend_destination;
            command.GpuState.depthEnable = state.depth_test; command.GpuState.depthMaskEnable = state.depth_write;
            command.GpuState.depthFunc = state.depth_function;
        }
        ready_ = true;
        return true;
    } catch (const std::exception&) {
        return fail();
    }
}
void ArmRenderer::Shutdown() {
    // Fingers reference the same surface; only the ten owning assets free GL buffers.
    for (auto& surface : mesh_surfaces_) surface.geo.Free();
    mesh_surfaces_.clear();
    OVRFW::GlProgram::Free(program_); program_ = {};
    scene_ = {}; body_from_mesh_ = {}; body_local_centers_ = {}; asset_indices_.fill(-1);
    visual_count_ = 0; ready_ = false;
}
bool ArmRenderer::Update(const BodySnapshot& snapshot, const Mat4& stage_from_base) {
    return ready_ && snapshot.valid && scene_.Update(snapshot.values, stage_from_base);
}
void ArmRenderer::Append(const OVR::Matrix4f& center_view, std::vector<OVRFW::ovrDrawSurface>& surfaces) const {
    if (!ready_ || !scene_.Valid()) return;
    Mat4 view;
    for (std::size_t row=0; row<4; ++row) for (std::size_t col=0; col<4; ++col) view.m[row*4+col] = center_view.M[row][col];
    for (auto index : scene_.BackToFront(view, body_local_centers_)) {
        if (asset_indices_[index] < 0) continue;
        const auto stage_from_mesh = Multiply(scene_.Models()[index], body_from_mesh_[index]);
        OVR::Matrix4f model;
        for (std::size_t row=0; row<4; ++row) for (std::size_t col=0; col<4; ++col) model.M[row][col] = stage_from_mesh.m[row*4+col];
        surfaces.emplace_back(model, &mesh_surfaces_[static_cast<std::size_t>(asset_indices_[index])]);
    }
}
} // namespace quest_newton
#endif
