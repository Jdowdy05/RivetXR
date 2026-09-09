#pragma once
#include "quest_newton/franka_kinematics.h"
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <memory>
#include <span>
#include <string>
#include <vector>
#include <chrono>

namespace quest_newton {
struct RoomEnvironment;
struct SceneObject {std::uint32_t body_index=0;std::uint32_t kind=1;kinematics::Vec3 half_extents{};};
struct SceneSnapshot {
    std::vector<kinematics::Pose> bodies;
    std::vector<SceneObject> objects;
    std::uint32_t model_generation=0,contact_count=0;
    std::uint64_t step_index=0,publication=0;
    double simulation_time=0,step_cpu_ms=0;
    std::chrono::steady_clock::time_point published_at{};
    // Attached only after the worker applies this complete collider batch.
    std::shared_ptr<const RoomEnvironment> room;
};
bool DecodeSceneSnapshot(std::span<const std::byte> bytes,SceneSnapshot& snapshot,std::string& error);
class SceneMailbox {
public:
    void Publish(SceneSnapshot snapshot){const std::lock_guard lock(mutex_);snapshot.publication=++publication_;snapshot.published_at=std::chrono::steady_clock::now();snapshot_=std::move(snapshot);}
    SceneSnapshot Read() const {const std::lock_guard lock(mutex_);return snapshot_;}
private:
    mutable std::mutex mutex_;
    SceneSnapshot snapshot_;
    std::uint64_t publication_=0;
};
} // namespace quest_newton
