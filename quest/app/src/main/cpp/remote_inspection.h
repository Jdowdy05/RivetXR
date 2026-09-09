#pragma once
#include "arm_renderer.h"
#include "quest_newton/franka_kinematics.h"
#include <array>

namespace quest_newton {
using InspectionIdentity=std::array<std::uint64_t,5>; // source, world epoch, calibration, representation, map generation
enum class InspectionTimingScope { None,Local,Remote };
inline InspectionTimingScope RenderTimingScope(bool local_robot,bool remote_geometry){
    return remote_geometry?InspectionTimingScope::Remote:local_robot?InspectionTimingScope::Local:InspectionTimingScope::None;
}
// XR-owned visual navigation. It never writes robot targets or physics poses.
class RemoteInspection {
public:
    bool Align(InspectionIdentity identity,const kinematics::Pose& map_from_observer,
               const kinematics::Pose& stage_from_head,std::uint64_t reference);
    bool Matches(InspectionIdentity identity,std::uint64_t reference) const;
    void Invalidate(){valid_=false;}
    bool Step(const kinematics::Pose& stage_from_head,float map_metres);
    bool SetScale(float scale,const kinematics::Pose& stage_from_head);
    float Scale() const{return scale_;}
    Mat4 StageFromMap() const;
private:
    InspectionIdentity identity_{};
    std::uint64_t reference_=0;
    kinematics::Pose stage_rotation_;
    kinematics::Vec3 stage_anchor_{},map_anchor_{};
    float scale_=1;
    bool valid_=false;
};
} // namespace quest_newton
