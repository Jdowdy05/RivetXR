#include "scene_snapshot.h"
#include <array>
#include <bit>
#include <cmath>
#include <cstring>

namespace quest_newton {
namespace {
template<class T>T Read(std::span<const std::byte> bytes,std::size_t offset) {
    static_assert(std::endian::native==std::endian::little);
    T value;std::memcpy(&value,bytes.data()+offset,sizeof(T));return value;
}
}
bool DecodeSceneSnapshot(std::span<const std::byte> bytes,SceneSnapshot& snapshot,std::string& error) {
    const auto reject=[&](const char* why){error=why;return false;};
    if(bytes.size()<48 || std::memcmp(bytes.data(),"QSIM",4)!=0 || Read<std::uint16_t>(bytes,4)!=1 ||
       Read<std::uint16_t>(bytes,6)!=48) return reject("Invalid scene snapshot header");
    const auto bodies=Read<std::uint32_t>(bytes,8),objects=Read<std::uint32_t>(bytes,12);
    if(bodies<12 || bodies>256 || objects>64 || objects>bodies-12 || bytes.size()!=48+bodies*28+objects*20)
        return reject("Invalid scene snapshot counts or length");
    SceneSnapshot candidate;
    candidate.model_generation=Read<std::uint32_t>(bytes,16);candidate.contact_count=Read<std::uint32_t>(bytes,20);
    candidate.step_index=Read<std::uint64_t>(bytes,24);candidate.simulation_time=Read<double>(bytes,32);
    candidate.step_cpu_ms=Read<double>(bytes,40);
    if(!std::isfinite(candidate.simulation_time) || candidate.simulation_time<0 ||
       !std::isfinite(candidate.step_cpu_ms) || candidate.step_cpu_ms<0) return reject("Invalid scene timing");
    candidate.bodies.reserve(bodies);
    for(std::size_t i=0;i<bodies;++i) {
        kinematics::Pose pose,normalized;
        for(std::size_t axis=0;axis<3;++axis)pose.position[axis]=Read<float>(bytes,48+i*28+axis*4);
        for(std::size_t axis=0;axis<4;++axis)pose.rotation[axis]=Read<float>(bytes,48+i*28+12+axis*4);
        if(!kinematics::NormalizePose(pose,normalized)) return reject("Invalid scene body pose");
        candidate.bodies.push_back(normalized);
    }
    std::array<bool,256> assigned{};
    for(std::size_t i=0;i<objects;++i) {
        const auto at=48+bodies*28+i*20;
        SceneObject object;object.body_index=Read<std::uint32_t>(bytes,at);object.kind=Read<std::uint32_t>(bytes,at+4);
        if(object.body_index<12 || object.body_index>=bodies || assigned[object.body_index] || object.kind!=1)
            return reject("Invalid scene object binding");
        assigned[object.body_index]=true;
        for(std::size_t axis=0;axis<3;++axis) {
            object.half_extents[axis]=Read<float>(bytes,at+8+axis*4);
            if(!std::isfinite(object.half_extents[axis]) || object.half_extents[axis]<=0 || object.half_extents[axis]>10)
                return reject("Invalid scene object size");
        }
        candidate.objects.push_back(object);
    }
    snapshot=std::move(candidate);error.clear();return true;
}
} // namespace quest_newton
