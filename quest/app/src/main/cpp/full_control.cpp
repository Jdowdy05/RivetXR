#include "full_control.h"
#include <algorithm>
#include <cmath>

namespace quest_newton {
void FullControlMachine::Emit(FullControlEvent event){if(observer_){event.expected=Output();observer_(event);}}
void FullControlMachine::Reset(){
    mapper_.Reset();targets_=kinematics::kHome;gripper_=0;input_allowed_=gripper_rearmed_=false;
    Emit({FullControlOperation::Reset});
}
void FullControlMachine::Invalidate(){
    mapper_.Invalidate();input_allowed_=gripper_rearmed_=false;
    Emit({FullControlOperation::Invalidate});
}
void FullControlMachine::Suppress(){
    mapper_.Update({}, {}, {});input_allowed_=gripper_rearmed_=false;
    Emit({FullControlOperation::Suppress});
}
void FullControlMachine::Calibrate(const ControllerSample& sample,const kinematics::Pose& base,const kinematics::Pose& offset){
    auto captured=sample;captured.calibrate_pressed=false;
    mapper_.RequestCalibration();mapper_.Update(captured,base,offset);
    Emit({FullControlOperation::Calibrate,sample,{},base,offset});
}
void FullControlMachine::Control(const ControllerSample& original,InputValue grip,const kinematics::Pose& base,
                                const kinematics::Pose& offset,bool allowed){
    // Keep the production loss transition even if a caller has not separately
    // emitted Suppress at the intervening physics boundary.
    if(!allowed && input_allowed_){mapper_.Update({}, {}, {});input_allowed_=gripper_rearmed_=false;}
    auto sample=original;if(!allowed)sample.focused=false;sample.calibrate_pressed=false;
    const auto& mapped=mapper_.Update(sample,base,offset);
    input_allowed_=allowed;
    if(mapped.engaged)targets_=mapped.joints;
    if(allowed && sample.trigger<.2F)gripper_rearmed_=true;
    if(allowed && gripper_rearmed_ && grip.active && std::isfinite(grip.value))gripper_=std::clamp(grip.value,0.F,1.F);
    Emit({FullControlOperation::Control,original,grip,base,offset,allowed});
}
} // namespace quest_newton
