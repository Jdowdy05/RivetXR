#include "gimbal_control.h"
#include <algorithm>
#include <bit>
#include <cmath>
#include <stdexcept>

namespace quest_newton {namespace {
std::uint64_t U(std::span<const std::byte> b,std::size_t at,std::size_t count){
    std::uint64_t n=0;for(std::size_t i=0;i<count;++i)n|=std::uint64_t(std::to_integer<unsigned char>(b[at+i]))<<(8*i);return n;
}
float F(std::span<const std::byte>b,std::size_t at){return std::bit_cast<float>(static_cast<std::uint32_t>(U(b,at,4)));}
void Put(std::span<std::byte>b,std::size_t at,std::uint64_t n,std::size_t count){for(std::size_t i=0;i<count;++i)b[at+i]=std::byte((n>>(8*i))&255);}
void Float(std::span<std::byte>b,std::size_t at,float f){Put(b,at,std::bit_cast<std::uint32_t>(f),4);}
bool Angles(const kinematics::Pose& pose,float& yaw,float& pitch){
    kinematics::Pose p;if(!kinematics::NormalizePose(pose,p))return false;
    auto f=kinematics::Compose(p,{{0,0,-1},{0,0,0,1}}).position;
    for(std::size_t i=0;i<3;++i)f[i]-=p.position[i];
    const auto horizontal=std::hypot(f[0],f[2]);if(horizontal<1e-4F)return false;
    yaw=std::atan2(-f[0],-f[2]);pitch=std::atan2(f[1],horizontal);return true;
}
}
bool DecodeGimbalFeedback(std::span<const std::byte>b,GimbalFeedback& output,std::string& error){
    const auto reject=[&](const char* text){error=text;return false;};
    if(b.size()!=96||U(b,0,4)!=0x46434751||U(b,4,2)!=1||U(b,6,2)!=96||U(b,92,4))return reject("Invalid camera-control feedback header");
    GimbalFeedback next;next.session=U(b,8,8);next.challenge=U(b,16,8);next.ack=U(b,24,8);next.server_ns=U(b,32,8);
    next.pan=F(b,40);next.tilt=F(b,44);next.source_id=U(b,48,8);next.flags=static_cast<std::uint32_t>(U(b,56,4));
    next.lease_ms=static_cast<std::uint32_t>(U(b,60,4));next.map_generation=U(b,64,8);
    next.pan_min=F(b,72);next.pan_max=F(b,76);next.tilt_min=F(b,80);next.tilt_max=F(b,84);next.max_rate=F(b,88);
    if(!next.session||!next.challenge||!next.server_ns||!next.source_id||!next.map_generation||!(next.flags&1)||next.flags&~15u||next.lease_ms>200)
        return reject("Only valid simulated camera control is supported");
    for(float f:{next.pan,next.tilt,next.pan_min,next.pan_max,next.tilt_min,next.tilt_max,next.max_rate})if(!std::isfinite(f))return reject("Nonfinite camera-control feedback");
    if(next.pan_min>=next.pan_max||next.tilt_min>=next.tilt_max||next.max_rate<=0||next.max_rate>20||
       std::abs(next.pan_min)>1000||std::abs(next.pan_max)>1000||std::abs(next.tilt_min)>1000||std::abs(next.tilt_max)>1000)
        return reject("Invalid simulated camera limits");
    output=next;error.clear();return true;
}
std::array<std::byte,64> EncodeGimbalCommand(const GimbalFeedback& f,const GimbalIntent& intent,std::uint64_t sequence,std::uint64_t stamp){
    if(!f.session||!f.challenge||!sequence||!stamp||!std::isfinite(intent.pan)||!std::isfinite(intent.tilt)||static_cast<unsigned>(intent.operation)>2)
        throw std::invalid_argument("Invalid camera-control command");
    std::array<std::byte,64>b{};Put(b,0,0x4d434751,4);Put(b,4,1,2);Put(b,6,64,2);Put(b,8,f.session,8);Put(b,16,f.challenge,8);
    Put(b,24,sequence,8);Put(b,32,stamp,8);Put(b,40,static_cast<unsigned>(intent.operation),4);Put(b,44,intent.clutch?1:0,4);
    Float(b,48,intent.pan);Float(b,52,intent.tilt);Put(b,56,100,4);return b;
}
void GimbalAim::Reset(){needs_release_=true;clutched_=false;accumulated_yaw_=0;}
GimbalIntent GimbalAim::Update(const GimbalAimInput& i){
    GimbalIntent out;out.sampled=i.now;
    if(i.epoch!=epoch_||(last_!=GimbalTime{}&&(i.now<last_||i.now-last_>std::chrono::milliseconds(150))))Reset();
    epoch_=i.epoch;last_=i.now;
    float yaw=0,pitch=0;
    if(!i.eligible||!i.clutch_active||!std::isfinite(i.clutch)||i.clutch<0||i.clutch>1||!Angles(i.orientation,yaw,pitch)){
        Reset();return out;
    }
    if(i.clutch<=.1F){needs_release_=false;clutched_=false;return out;}
    if(needs_release_)return out;
    if(i.clutch<.7F){clutched_=false;return out;}
    if(!clutched_){clutched_=true;last_yaw_=yaw;last_pitch_=pitch;accumulated_yaw_=0;start_pitch_=pitch;start_pan_=i.feedback.pan;start_tilt_=i.feedback.tilt;}
    const float delta=std::remainder(yaw-last_yaw_,6.283185307179586F);last_yaw_=yaw;
    const float pitch_delta=pitch-last_pitch_;last_pitch_=pitch;
    if(std::abs(delta)>.5F||std::abs(pitch_delta)>.5F){Reset();return out;} // A pose jump is not a deliberate slew.
    accumulated_yaw_+=delta;
    out.operation=GimbalOperation::Aim;out.clutch=true;
    out.pan=std::clamp(start_pan_+accumulated_yaw_,i.feedback.pan_min,i.feedback.pan_max);
    out.tilt=std::clamp(start_tilt_+pitch-start_pitch_,i.feedback.tilt_min,i.feedback.tilt_max);
    return out;
}
}
