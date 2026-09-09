#include "arm_renderer.h"
#include "passthrough_layer.h"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <limits>
#include <source_location>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using namespace quest_newton;
void Check(bool condition, const std::source_location& location = std::source_location::current()) {
    if (!condition) throw std::runtime_error("check failed at line " + std::to_string(location.line()));
}
void Near(float a, float b) { Check(std::abs(a - b) < 1e-5F); }
BodyValues IdentityBodies() {
    BodyValues values{};
    for (std::size_t i = 0; i < 12; ++i) values[i * 7 + 6] = 1;
    return values;
}

void DecodeAndPreserve() {
    ArmScene scene;
    auto data = IdentityBodies();
    for (std::size_t i = 0; i < 12; ++i) data[7 * i] = static_cast<float>(i);
    data[3] = 0; data[4] = 0; data[5] = 2; data[6] = 2;
    Check(scene.Update(data, Mat4::Identity()));
    Near(scene.Bodies()[0].rotation[2], std::sqrt(.5F));
    Near(scene.Bodies()[0].rotation[3], std::sqrt(.5F));
    Near(scene.Bodies()[11].position[0], 11);
    const auto previous = scene.Models();
    for (std::size_t i = 0; i < 84; ++i) {
        auto bad = data; bad[i] = std::numeric_limits<float>::quiet_NaN();
        Check(!scene.Update(bad, Mat4::Identity()));
        Check(scene.Models() == previous);
    }
    auto bad = data;
    for (int j = 3; j < 7; ++j) bad[7 * 11 + static_cast<std::size_t>(j)] = 1e-12F;
    Check(!scene.Update(bad, Mat4::Identity()));
    Check(!scene.Update(std::span(data).first(83), Mat4::Identity()));
    auto invalid_base = Mat4::Identity(); invalid_base.m[0] = std::numeric_limits<float>::infinity();
    Check(!scene.Update(data, invalid_base));
    Check(scene.Models() == previous);
}

void BasisAndComposition() {
    const auto base = ProvisionalStagePlacement();
    const auto x = TransformPoint(base, {1, 0, 0});
    const auto y = TransformPoint(base, {0, 1, 0});
    const auto z = TransformPoint(base, {0, 0, 1});
    Near(x[0], 0); Near(x[1], 0); Near(x[2], -2.2F);
    Near(y[0], -1); Near(y[1], 0); Near(y[2], -1.2F);
    Near(z[0], 0); Near(z[1], 1); Near(z[2], -1.2F);
    auto bodies = IdentityBodies(); bodies[0] = 1; bodies[1] = 2; bodies[2] = 3;
    ArmScene scene; Check(scene.Update(bodies, base));
    const auto center = TransformPoint(scene.Models()[0], {0, 0, 0});
    // The mesh supplies its own vertices; a body origin must not inherit a
    // provisional cuboid's center or dimensions.
    Near(center[0], -2); Near(center[1], 3); Near(center[2], -2.2F);
}

void BodyAndRenderContract() {
    ArmScene scene; Check(scene.Update(IdentityBodies(), Mat4::Identity()));
    for (std::size_t i = 0; i < 12; ++i) {
        const std::string name = i < 9 ? "panda_link" + std::to_string(i) :
            i == 9 ? "panda_hand" : i == 10 ? "panda_leftfinger" : "panda_rightfinger";
        Check(ArmBodyNames()[i] == name);
        Check(scene.Models()[i] == Mat4::Identity());
    }
    const auto state = ArmRenderState();
    Check(state.premultiplied_color == (std::array<float, 4>{0,.35F,.35F,.35F}));
    Check(state.blend && state.blend_equation == 0x8006 && state.blend_source == 1 && state.blend_destination == 0x0303);
    Check(state.depth_test && state.depth_function == 0x0203 && !state.depth_write);
    auto data = IdentityBodies(); data[2] = -5; data[9] = -1;
    Check(scene.Update(data, Mat4::Identity()));
    const auto sorted = scene.BackToFront(Mat4::Identity());
    Check(sorted.front() == 0);
    Check(sorted.back() != 0);
}

void DecodeEveryBodyComponent() {
    BodyValues values{};
    for (std::size_t body = 0; body < kArmBodies; ++body) {
        const float ordinal = static_cast<float>(body + 1);
        values[body*7] = ordinal;
        values[body*7+1] = -2*ordinal;
        values[body*7+2] = 3*ordinal;
        values[body*7+3] = ordinal;
        values[body*7+4] = 2*ordinal;
        values[body*7+5] = 3*ordinal;
        values[body*7+6] = 4*ordinal;
    }
    ArmScene scene; Check(scene.Update(values, Mat4::Identity()));
    for (std::size_t body = 0; body < kArmBodies; ++body) {
        const float ordinal = static_cast<float>(body + 1);
        const auto& pose = scene.Bodies()[body];
        Near(pose.position[0], ordinal); Near(pose.position[1], -2*ordinal); Near(pose.position[2], 3*ordinal);
        Near(pose.rotation[0], 1/std::sqrt(30.F)); Near(pose.rotation[1], 2/std::sqrt(30.F));
        Near(pose.rotation[2], 3/std::sqrt(30.F)); Near(pose.rotation[3], 4/std::sqrt(30.F));
        const auto origin = TransformPoint(scene.Models()[body], {0,0,0});
        Near(origin[0], ordinal); Near(origin[1], -2*ordinal); Near(origin[2], 3*ordinal);
    }
}

void VisualPlacementAndValidation() {
    // A shared finger vertex must acquire the right finger's local Rz(pi),
    // then its physics body pose, then STAGE placement, exactly once.
    auto data = IdentityBodies();
    data[77] = 1; data[78] = 2; data[79] = 3;
    data[82] = std::sqrt(.5F); data[83] = std::sqrt(.5F); // Body Rz(pi/2).
    ArmScene scene; Check(scene.Update(data, ProvisionalStagePlacement()));
    Mat4 visual;
    Check(MakeVisualTransform({{.1F,.2F,.3F},{0,0,1,0}}, {2,3,4}, visual));
    const auto point = TransformPoint(Multiply(scene.Models()[11], visual), {.01F,.02F,.03F});
    // scale=(.02,.06,.12), visual=(.08,.14,.42), body=(.86,2.08,3.42).
    Near(point[0], -2.08F); Near(point[1], 3.42F); Near(point[2], -2.06F);
    const auto previous = visual;
    const auto nan = std::numeric_limits<float>::quiet_NaN();
    const auto inf = std::numeric_limits<float>::infinity();
    Check(!MakeVisualTransform({{nan,0,0}}, {1,1,1}, visual));
    Check(!MakeVisualTransform({{}, {0,0,inf,1}}, {1,1,1}, visual));
    Check(!MakeVisualTransform({{}, {0,0,0,0}}, {1,1,1}, visual));
    Check(!MakeVisualTransform({{}, {0,0,0,2}}, {1,1,1}, visual));
    Check(!MakeVisualTransform({}, {1,nan,1}, visual));
    Check(!MakeVisualTransform({}, {1,0,1}, visual));
    Check(!MakeVisualTransform({}, {-1,1,1}, visual));
    Check(visual == previous);
}

void SortByTransformedMeshCenters() {
    auto data = IdentityBodies();
    data[2] = -2; data[9] = -3;
    ArmScene scene; Check(scene.Update(data, Mat4::Identity()));
    Check(scene.BackToFront(Mat4::Identity()).front() == 1);
    std::array<Vec3, kArmBodies> centers{};
    centers[0] = {0,0,-2}; // Mesh center z=-4 although this body's origin is nearer.
    Check(scene.BackToFront(Mat4::Identity(), centers).front() == 0);
    // CenterView, including head orientation, must affect ordering.
    const auto reversed = scene.BackToFront(PoseMatrix({{}, {0,1,0,0}}), centers);
    Check(reversed[10] == 1 && reversed[11] == 0);
    // Rotate the local visual center into the body's frame before sorting.
    data[2] = -2; data[4] = -std::sqrt(.5F); data[6] = std::sqrt(.5F);
    Check(scene.Update(data, Mat4::Identity()));
    centers[0] = {-2,0,0}; // Body Ry(-pi/2) maps local -X onto camera -Z.
    Check(scene.BackToFront(Mat4::Identity(), centers).front() == 0);
    centers[0] = {2,0,0};
    Check(scene.BackToFront(Mat4::Identity(), centers).front() == 1);
}

void CompleteMailbox() {
    SnapshotMailbox mailbox;
    Check(!mailbox.Read().valid);
    auto values = IdentityBodies(); Check(mailbox.Publish(values));
    const auto good = mailbox.Read(); Check(good.valid && good.generation == 1);
    values[0] = std::numeric_limits<float>::infinity(); Check(!mailbox.Publish(values));
    Check(mailbox.Read().generation == 1 && mailbox.Read().values == good.values);
    std::atomic<bool> done{false};
    std::thread writer([&] {
        for (int generation = 1; generation <= 10000; ++generation) {
            auto frame = IdentityBodies();
            for (std::size_t i = 0; i < 12; ++i) frame[7 * i] = static_cast<float>(generation);
            Check(mailbox.Publish(frame));
        }
        done.store(true);
    });
    bool coherent = true;
    do {
        const auto frame = mailbox.Read();
        for (std::size_t i = 1; i < 12; ++i) coherent &= frame.values[7 * i] == frame.values[0];
    } while (!done.load());
    writer.join(); Check(coherent); Check(mailbox.Read().generation == 10001);
}

void RecordedPlacementIsOneSnapshot() {
    SnapshotMailbox mailbox;
    auto data = IdentityBodies();
    const auto placement = ProvisionalStagePlacement();
    Check(mailbox.Publish(data, &placement));
    const auto snapshot = mailbox.Read();
    Check(snapshot.placement_valid && snapshot.stage_from_base == placement);
    auto bad = placement;
    bad.m[3] = std::numeric_limits<float>::infinity();
    Check(!mailbox.Publish(data, &bad));
    Check(mailbox.Read().generation == snapshot.generation);
    Check(mailbox.Read().stage_from_base == placement);
    Check(mailbox.Publish(data));
    Check(!mailbox.Read().placement_valid);
}

void PassthroughFailureAndOrdering() {
    std::vector<std::string> events;
    bool fail_layer = true;
    PassthroughDispatch api{
        [&](std::uint64_t& handle) { events.push_back("feature+"); handle = 1; return true; },
        [&](std::uint64_t feature, std::uint64_t& handle) {
            Check(feature == 1); events.push_back("layer+"); if (fail_layer) return false; handle = 2; return true;
        },
        [&](std::uint64_t handle) { Check(handle == 2); events.push_back("layer-"); return true; },
        [&](std::uint64_t handle) { Check(handle == 1); events.push_back("feature-"); return true; }};
    PassthroughLayer layer;
    Check(!layer.Start(api)); Check(!layer.Ready());
    Check(events == (std::vector<std::string>{"feature+","layer+","feature-"}));
    events.clear(); fail_layer = false; Check(layer.Start(api)); Check(layer.Ready());
    int count = 0; Check(layer.ReserveUnderlay(16, count) == 0); Check(count == 1);
    const int projection_index = count++; Check(projection_index == 1);
    int full = 15; Check(layer.ReserveUnderlay(16, full) == -1 && full == 15);
    int invalid = -1; Check(layer.ReserveUnderlay(16, invalid) == -1);
    Check(layer.Stop([&] { events.push_back("renderer-"); }));
    Check(events == (std::vector<std::string>{"feature+","layer+","renderer-","layer-","feature-"}));
    events.clear(); Check(layer.Stop({})); Check(events.empty());
    count = 0; Check(layer.ReserveUnderlay(16, count) == -1);
    Check(!layer.Start({}));
}
}

int main() {
    try {
        DecodeAndPreserve(); DecodeEveryBodyComponent(); BasisAndComposition(); BodyAndRenderContract();
        VisualPlacementAndValidation(); SortByTransformedMeshCenters();
        CompleteMailbox(); RecordedPlacementIsOneSnapshot(); PassthroughFailureAndOrdering();
        std::puts("arm renderer, snapshot mailbox and passthrough tests passed");
    } catch (const std::exception& error) { std::fprintf(stderr, "%s\n", error.what()); return 1; }
}
