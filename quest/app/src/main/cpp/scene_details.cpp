#include "scene_details.h"
#include <array>
#include <bit>
#include <cmath>
#include <cstring>
#include <set>

namespace quest_newton {
namespace {
template<class T>T Read(std::span<const std::byte> bytes,std::size_t at){
    static_assert(std::endian::native==std::endian::little);
    T result;std::memcpy(&result,bytes.data()+at,sizeof(T));return result;
}
}
bool DecodeSceneDetails(std::span<const std::byte> bytes,const SceneSnapshot& scene,SceneDetails& output,std::string& error){
    const auto reject=[&](const char* why){error=why;return false;};
    if(bytes.size()<48 || std::memcmp(bytes.data(),"QDIA",4)!=0 || Read<std::uint16_t>(bytes,4)!=1 ||
       Read<std::uint16_t>(bytes,6)!=48)return reject("Invalid QDIA header");
    SceneDetails next;next.model_generation=Read<std::uint32_t>(bytes,8);next.body_count=Read<std::uint32_t>(bytes,12);
    next.step_index=Read<std::uint64_t>(bytes,16);next.simulation_time=Read<double>(bytes,24);
    const auto objects=Read<std::uint32_t>(bytes,32),contacts=Read<std::uint32_t>(bytes,36);
    next.total_contacts=Read<std::uint32_t>(bytes,40);const auto flags=Read<std::uint32_t>(bytes,44);
    next.truncated=(flags&1)!=0;
    next.contacts_current=(flags&2)==0;
    if(objects>9 || objects!=scene.objects.size() || contacts>32 || contacts>next.total_contacts ||
       next.total_contacts>scene.contact_count || (flags&~3U)!=0 || next.truncated!=(contacts<next.total_contacts) ||
       (!next.contacts_current && (contacts || next.total_contacts || next.truncated)) ||
       bytes.size()!=48+objects*24+contacts*36)return reject("Invalid QDIA counts or flags");
    if(next.model_generation!=scene.model_generation || next.body_count!=scene.bodies.size() ||
       next.step_index!=scene.step_index || !std::isfinite(next.simulation_time) || next.simulation_time!=scene.simulation_time)
        return reject("QDIA does not match scene generation and step");
    std::set<std::uint32_t> ids,bodies;
    for(std::size_t i=0;i<objects;++i){
        const auto at=48+i*24;DetailedObject object;
        object.id=Read<std::uint32_t>(bytes,at);object.body_index=Read<std::uint32_t>(bytes,at+4);
        if(!ids.insert(object.id).second || !bodies.insert(object.body_index).second || Read<std::uint32_t>(bytes,at+8)!=1)
            return reject("Duplicate QDIA object or invalid kind");
        const SceneObject* shape=nullptr;
        for(const auto& candidate:scene.objects)if(candidate.body_index==object.body_index)shape=&candidate;
        if(!shape)return reject("QDIA object body is not a scene object");
        for(std::size_t axis=0;axis<3;++axis){
            object.half_extents[axis]=Read<float>(bytes,at+12+axis*4);
            if(!std::isfinite(object.half_extents[axis]) || object.half_extents[axis]!=shape->half_extents[axis])
                return reject("QDIA geometry differs from scene");
        }
        next.objects.push_back(object);
    }
    for(std::size_t i=0;i<contacts;++i){
        const auto at=48+objects*24+i*36;ContactPoint contact;
        contact.body_a=Read<std::int32_t>(bytes,at);contact.body_b=Read<std::int32_t>(bytes,at+4);
        for(const auto body:{contact.body_a,contact.body_b})
            if(body<-1 || body>=static_cast<std::int32_t>(next.body_count))return reject("Invalid QDIA contact body");
        if(!bodies.contains(static_cast<std::uint32_t>(contact.body_a)) && !bodies.contains(static_cast<std::uint32_t>(contact.body_b)))
            return reject("QDIA contact does not involve an object");
        float normal_length=0;
        for(std::size_t axis=0;axis<3;++axis){
            contact.position[axis]=Read<float>(bytes,at+8+axis*4);contact.normal[axis]=Read<float>(bytes,at+20+axis*4);
            if(!std::isfinite(contact.position[axis]) || !std::isfinite(contact.normal[axis]))return reject("Invalid QDIA contact coordinates");
            normal_length+=contact.normal[axis]*contact.normal[axis];
        }
        contact.normal_force=Read<float>(bytes,at+32);
        if(std::abs(normal_length-1.F)>.01F || !std::isfinite(contact.normal_force))return reject("Invalid QDIA normal or force");
        next.contacts.push_back(contact);
    }
    output=std::move(next);error.clear();return true;
}
} // namespace quest_newton
