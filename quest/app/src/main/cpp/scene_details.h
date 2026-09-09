#pragma once
#include "scene_snapshot.h"
#include <cmath>
#include <optional>

namespace quest_newton {
struct DetailedObject {
    std::uint32_t id=0,body_index=0;
    kinematics::Vec3 half_extents{};
};
struct ContactPoint {
    std::int32_t body_a=-1,body_b=-1;
    kinematics::Vec3 position{},normal{};
    float normal_force=0;
};
struct SceneDetails {
    std::uint32_t model_generation=0,body_count=0,total_contacts=0;
    std::uint64_t step_index=0;
    double simulation_time=0;
    bool truncated=false,contacts_current=true;
    std::vector<DetailedObject> objects;
    std::vector<ContactPoint> contacts;
};
// QDIA is optional diagnostics. Reject a mismatched sidecar without faulting physics.
bool DecodeSceneDetails(std::span<const std::byte> bytes,const SceneSnapshot& scene,
                        SceneDetails& details,std::string& error);
class SceneDetailsPoll {
public:
    bool Due(std::uint32_t generation,bool contacts,double now) const {
        if(!std::isfinite(now) || (last_attempt_>=0 && now<last_attempt_))return false;
        return ((!generation_ || *generation_!=generation) && (!failed_ || now-last_attempt_>=.5)) ||
               ContactsDue(contacts,now);
    }
    bool ContactsDue(bool requested,double now) const{return requested && std::isfinite(now) &&
        (last_contacts_<0 || now-last_contacts_>=.1);}
    void Attempt(double now,bool contacts){last_attempt_=now;if(contacts)last_contacts_=now;}
    void Succeeded(std::uint32_t generation,double now){generation_=generation;last_attempt_=now;failed_=false;}
    void Failed(double now){last_attempt_=now;failed_=true;}
    void Invalidate(){generation_.reset();}
    // Python model generations restart in a new Session. Keep the global
    // contact sampling deadline, but no mapping/failure state may cross it.
    void BeginSession(){generation_.reset();failed_=false;}
private:
    std::optional<std::uint32_t> generation_;
    double last_attempt_=-1;
    double last_contacts_=-1;
    bool failed_=false;
};
} // namespace quest_newton
