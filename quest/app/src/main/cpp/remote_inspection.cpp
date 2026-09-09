#include "remote_inspection.h"
#include <cmath>

namespace quest_newton {
namespace kin=kinematics;
namespace {
bool Bounded(const kin::Vec3& position){for(float value:position)if(!std::isfinite(value) || std::abs(value)>100.F)return false;return true;}
kin::Vec3 Rotate(const kin::Pose& rotation,const kin::Vec3& value){return kin::Compose(rotation,{value,{0,0,0,1}}).position;}
}
bool RemoteInspection::Align(InspectionIdentity identity,const kin::Pose& observer,const kin::Pose& head,std::uint64_t reference){
    kin::Pose normalized_head,normalized_observer;
    if(!identity[0] || !identity[1] || !identity[2] || identity[3]>2)return false;
    if(!kin::NormalizePose(head,normalized_head) || !kin::NormalizePose(observer,normalized_observer) ||
       !Bounded(normalized_head.position) || !Bounded(normalized_observer.position))return false;
    const auto head_forward=Rotate({{},normalized_head.rotation},{0,0,-1});
    if(std::hypot(head_forward[0],head_forward[2])<1e-5F)return false;
    const float stage_yaw=std::atan2(-head_forward[0],-head_forward[2]);
    const auto map_forward=Rotate({{},normalized_observer.rotation},{0,0,-1});
    const float map_yaw=std::hypot(map_forward[0],map_forward[1])<1e-5F?0.F:std::atan2(map_forward[1],map_forward[0]);
    const kin::Pose stage_heading{{},{0,std::sin(stage_yaw*.5F),0,std::cos(stage_yaw*.5F)}};
    const kin::Pose map_heading{{},{0,0,std::sin(map_yaw*.5F),std::cos(map_yaw*.5F)}};
    // OpenXR observer axes in a Z-up map when looking along map +X.
    const kin::Pose map_from_view=kin::Compose(map_heading,{{},{.5F,-.5F,-.5F,.5F}});
    stage_rotation_=kin::Compose(stage_heading,kin::Inverse(map_from_view));
    map_anchor_=normalized_observer.position;stage_anchor_=normalized_head.position;
    identity_=identity;reference_=reference;valid_=true;return true;
}
bool RemoteInspection::Matches(InspectionIdentity identity,std::uint64_t reference) const{return valid_ && identity==identity_ && reference==reference_;}
bool RemoteInspection::Step(const kin::Pose& head,float metres){
    kin::Pose normalized;if(!valid_ || !kin::NormalizePose(head,normalized) || !std::isfinite(metres) || std::abs(metres)>1.F)return false;
    auto direction=Rotate({{},normalized.rotation},{0,0,-1});direction[1]=0;
    const float length=std::hypot(direction[0],direction[2]);if(length<1e-5F)return false;
    for(auto& value:direction)value/=length;
    const auto in_map=Rotate(kin::Inverse(stage_rotation_),direction);auto next=map_anchor_;
    for(std::size_t i=0;i<3;++i)next[i]+=in_map[i]*metres;
    if(!Bounded(next))return false;map_anchor_=next;return true;
}
bool RemoteInspection::SetScale(float scale,const kin::Pose& head){
    kin::Pose normalized;if(!valid_ || !std::isfinite(scale) || scale<.25F || scale>2.F ||
        !kin::NormalizePose(head,normalized) || !Bounded(normalized.position))return false;
    kin::Vec3 displacement{};
    for(std::size_t i=0;i<3;++i)displacement[i]=(normalized.position[i]-stage_anchor_[i])/scale_;
    const auto in_map=Rotate(kin::Inverse(stage_rotation_),displacement);auto next=map_anchor_;
    for(std::size_t i=0;i<3;++i)next[i]+=in_map[i];
    if(!Bounded(next))return false;
    map_anchor_=next;stage_anchor_=normalized.position;scale_=scale;return true;
}
Mat4 RemoteInspection::StageFromMap() const {
    if(!valid_)return Mat4::Identity();
    auto matrix=PoseMatrix({{},stage_rotation_.rotation});
    const auto rotated=Rotate(stage_rotation_,map_anchor_);
    for(std::size_t r=0;r<3;++r){
        for(std::size_t c=0;c<3;++c)matrix.m[r*4+c]*=scale_;
        matrix.m[r*4+3]=stage_anchor_[r]-scale_*rotated[r];
    }
    return matrix;
}
} // namespace quest_newton
