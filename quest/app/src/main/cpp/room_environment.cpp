#include "room_environment.h"
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace quest_newton {
namespace {
const char* KindName(RoomSurfaceKind kind) {
    switch (kind) {
        case RoomSurfaceKind::Floor: return "floor";
        case RoomSurfaceKind::Wall: return "wall";
        case RoomSurfaceKind::Table: return "table";
    }
    return nullptr;
}
}
bool ValidateRoomEnvironment(const RoomEnvironment& room, std::string& error) {
    const auto reject=[&](const char* text){error=text;return false;};
    if (room.colliders.size()>kMaxRoomColliders) return reject("Room has too many surfaces");
    if (!room.enabled && !room.colliders.empty()) return reject("Disabled room must have no surfaces");
    bool floor=false;
    for (const auto& collider:room.colliders) {
        if (!KindName(collider.kind)) return reject("Unknown room surface kind");
        floor |= collider.kind==RoomSurfaceKind::Floor;
        for (const float value:collider.world_from_collider.position)
            if (!std::isfinite(value) || std::abs(value)>100.F) return reject("Invalid room position");
        double length2=0;
        for (const float value:collider.world_from_collider.rotation) {
            if (!std::isfinite(value)) return reject("Invalid room rotation");
            length2+=static_cast<double>(value)*value;
        }
        if (std::abs(std::sqrt(length2)-1.)>1e-3) return reject("Room rotation must be unit length");
        for (const float value:collider.half_extents)
            if (!std::isfinite(value) || value<.005F || value>20.F) return reject("Invalid room surface size");
    }
    if (room.enabled && (!floor || room.revision==0)) return reject("Room requires a floor and valid revision");
    error.clear();return true;
}
std::string RoomEnvironmentJson(const RoomEnvironment& room) {
    std::string error;
    if (!ValidateRoomEnvironment(room,error)) throw std::invalid_argument(error);
    std::ostringstream out;out.imbue(std::locale::classic());out<<std::setprecision(17)<<std::boolalpha;
    out<<"{\"version\":1,\"revision\":"<<room.revision<<",\"enabled\":"<<room.enabled<<",\"colliders\":[";
    bool first=true;
    for (const auto& collider:room.colliders) {
        if (!first) out<<',';
        first=false;
        out<<"{\"kind\":\""<<KindName(collider.kind)<<"\",\"pose\":[";
        for (std::size_t i=0;i<3;++i) out<<(i?",":"")<<collider.world_from_collider.position[i];
        for (const auto value:collider.world_from_collider.rotation) out<<','<<value;
        out<<"],\"half_extents\":[";
        for (std::size_t i=0;i<3;++i) out<<(i?",":"")<<collider.half_extents[i];
        out<<"]}";
    }
    out<<"]}";return out.str();
}
bool RoomPhysicsBlocked(bool requested, bool ready, bool frame_current,
                        const RoomEnvironment& desired, const RoomEnvironment* applied) {
    if (!requested) return applied && applied->enabled;
    return !ready || !frame_current || !desired.enabled || !applied || !applied->enabled ||
        applied->revision!=desired.revision;
}
} // namespace quest_newton
