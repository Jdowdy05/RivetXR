#pragma once
#include "quest_newton/franka_kinematics.h"
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace quest_newton {
using GimbalTime=std::chrono::steady_clock::time_point;
enum class GimbalOperation:std::uint32_t { Hold=0,Aim=1,ResetMap=2 };
struct GimbalFeedback {
    std::uint64_t session=0,challenge=0,ack=0,server_ns=0,source_id=0,map_generation=0;
    float pan=0,tilt=0,pan_min=0,pan_max=0,tilt_min=0,tilt_max=0,max_rate=0;
    std::uint32_t flags=0,lease_ms=0;
    GimbalTime received{};
};
struct GimbalIntent {
    GimbalOperation operation=GimbalOperation::Hold;
    bool clutch=false;
    float pan=0,tilt=0;
    GimbalTime sampled{};
};
bool DecodeGimbalFeedback(std::span<const std::byte> bytes,GimbalFeedback& output,std::string& error);
std::array<std::byte,64> EncodeGimbalCommand(const GimbalFeedback& feedback,const GimbalIntent& intent,
                                           std::uint64_t sequence,std::uint64_t client_ns);
struct GimbalAimInput {
    GimbalTime now{};
    std::array<std::uint64_t,5> epoch{};
    bool eligible=false,clutch_active=false;
    float clutch=0;
    kinematics::Pose orientation{};
    GimbalFeedback feedback;
};
class GimbalAim {
public:
    GimbalIntent Update(const GimbalAimInput& input);
    void Reset();
private:
    std::array<std::uint64_t,5> epoch_{};
    GimbalTime last_{};
    bool needs_release_=true,clutched_=false;
    float last_yaw_=0,last_pitch_=0,accumulated_yaw_=0,start_pitch_=0,start_pan_=0,start_tilt_=0;
};
}
