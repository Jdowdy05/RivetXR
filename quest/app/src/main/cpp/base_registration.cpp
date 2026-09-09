#include "base_registration.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace quest_newton {
namespace kin=kinematics;
kin::Pose HandOffsetPose(const std::array<float,3>& degrees) {
    const auto axis=[&](std::size_t i) {
        kin::Pose pose;
        const float half=degrees[i]*std::numbers::pi_v<float>/360.F;
        pose.rotation={0,0,0,std::cos(half)};pose.rotation[i]=std::sin(half);return pose;
    };
    return kin::Compose(axis(2),kin::Compose(axis(1),axis(0)));
}
bool BaseRegistration::Update(const kin::Pose& head,bool tracking,bool clutch,const SimSettings& settings) {
    kin::Pose normalized;
    if(!tracking || !kin::NormalizePose(head,normalized)) {tracked_=false;return false;}
    const float height=normalized.position[1];
    if(!valid_) {
        const auto forward=kin::Compose(kin::Pose{{0,0,0},normalized.rotation},kin::Pose{{0,0,-1}}).position;
        const float yaw=std::atan2(-forward[0],-forward[2]);
        kin::Pose heading;heading.rotation={0,std::sin(yaw*.5F),0,std::cos(yaw*.5F)};
        kin::Pose axes;axes.rotation={-.5F,.5F,.5F,.5F};
        stage_from_world_=kin::Compose(heading,axes);
        stage_from_world_.position={normalized.position[0],0,normalized.position[2]};
        world_from_base_.position={.35F,0,std::max(.05F,height-settings.shoulder_offset_m)};
        world_from_base_.rotation={0,0,0,1};valid_=true;
    } else if(!settings.floating_base) {
        world_from_base_.position[2]+=last_shoulder_offset_-settings.shoulder_offset_m;
        const bool transition=!tracked_ || clutch!=last_clutch_ || settings.follow_height!=last_follow_ || last_floating_;
        const bool move=settings.follow_height?!clutch:clutch;
        if(move && !transition) world_from_base_.position[2]+=height-last_head_height_;
    }
    tracked_=true;last_head_height_=height;last_clutch_=clutch;
    last_follow_=settings.follow_height;last_floating_=settings.floating_base;
    last_shoulder_offset_=settings.shoulder_offset_m;return true;
}
} // namespace quest_newton
