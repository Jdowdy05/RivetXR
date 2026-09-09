#pragma once
#include "quest_newton/franka_kinematics.h"
#include <cstdint>
#include <string>
#include <vector>

namespace quest_newton {
enum class RoomSurfaceKind { Floor, Wall, Table };
struct RoomCollider {
    RoomSurfaceKind kind = RoomSurfaceKind::Floor;
    kinematics::Pose world_from_collider;
    kinematics::Vec3 half_extents{};
};
struct RoomEnvironment {
    std::uint64_t revision = 0;
    bool enabled = false;
    std::vector<RoomCollider> colliders;
};
inline constexpr std::size_t kMaxRoomColliders = 64;
bool ValidateRoomEnvironment(const RoomEnvironment& room, std::string& error);
std::string RoomEnvironmentJson(const RoomEnvironment& room);
bool RoomPhysicsBlocked(bool requested, bool ready, bool frame_current,
                        const RoomEnvironment& desired, const RoomEnvironment* applied);
} // namespace quest_newton
