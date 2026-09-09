#pragma once
#include "registered_input.h"
#include "sim_settings.h"
#include <functional>
#include <utility>

namespace quest_newton {
enum class FullControlOperation : unsigned char { Reset, Invalidate, Suppress, Calibrate, Control };
struct FullControlOutput {
    MappedTarget mapped;
    kinematics::JointVector targets=kinematics::kHome;
    float gripper=0;
    bool input_allowed=false,gripper_rearmed=false;
};
struct FullControlEvent {
    FullControlOperation operation=FullControlOperation::Reset;
    ControllerSample sample;
    InputValue gripper;
    kinematics::Pose stage_from_base,palm_offset;
    bool allowed=false;
    FullControlOutput expected;
};
// Worker-owned. Existing clock/boundary guards decide WHEN to invoke operations.
// The observer must copy/serialize synchronously; mapped.reason names literals.
class FullControlMachine {
public:
    using Observer=std::function<void(const FullControlEvent&)>;
    void SetObserver(Observer observer){observer_=std::move(observer);}
    void Reset();
    void Invalidate();
    void Suppress();
    void Calibrate(const ControllerSample& sample,const kinematics::Pose& stage_from_base,
                   const kinematics::Pose& palm_offset);
    void Control(const ControllerSample& sample,InputValue gripper,const kinematics::Pose& stage_from_base,
                 const kinematics::Pose& palm_offset,bool allowed);
    const MappedTarget& Current() const{return mapper_.Current();}
    const kinematics::JointVector& Targets() const{return targets_;}
    float Gripper() const{return gripper_;}
    bool InputAllowed() const{return input_allowed_;}
    bool GripperRearmed() const{return gripper_rearmed_;}
    FullControlOutput Output() const{return {mapper_.Current(),targets_,gripper_,input_allowed_,gripper_rearmed_};}
private:
    void Emit(FullControlEvent event);
    RegisteredInputMapper mapper_;
    kinematics::JointVector targets_=kinematics::kHome;
    float gripper_=0;
    bool input_allowed_=false,gripper_rearmed_=false;
    Observer observer_;
};
} // namespace quest_newton
