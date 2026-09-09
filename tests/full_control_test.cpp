#include "full_control.h"
#include <iostream>
#include <stdexcept>
#include <vector>
#include <limits>

int main() {
    using namespace quest_newton;
    try {
        FullControlMachine control;
        std::vector<FullControlEvent> events;
        control.SetObserver([&](const auto& event){events.push_back(event);});
        control.Reset();
        if (control.Targets()!=kinematics::kHome || control.Gripper()!=0 || control.InputAllowed() || control.GripperRearmed())
            throw std::runtime_error("reset does not establish the known control seed");
        const auto check=[](bool value,const char* reason){if(!value)throw std::runtime_error(reason);};
        ControllerSample sample;
        sample.focused=sample.stage_valid=sample.pose_active=sample.trigger_active=sample.calibrate_active=true;
        sample.position_valid=sample.orientation_valid=sample.position_tracked=sample.orientation_tracked=true;
        sample.stage_from_grip.position={.3F,.8F,-.4F};
        kinematics::Pose base;base.position={0,0,.7F};
        control.Calibrate(sample,base,{});
        check(control.Current().calibrated&&!control.Current().engaged,"released calibration must calibrate without engagement");
        control.Control(sample,{.3F,true},base,{},true);
        check(control.GripperRearmed()&&control.Gripper()==.3F&&!control.Current().engaged,"gripper works independently after release");
        sample.trigger=1;
        control.Control(sample,{.8F,true},base,{},true);
        check(control.Current().engaged&&control.Gripper()==.8F,"released then engaged control accepts arm and gripper");
        sample.stage_from_grip.position[0]+=.02F;
        control.Control(sample,{.9F,true},base,{},true);
        const auto held=control.Targets();const auto grip=control.Gripper();
        check(held!=kinematics::kHome,"moving engaged target must exercise IK");
        control.Suppress();
        check(control.Targets()==held&&control.Gripper()==grip&&!control.InputAllowed()&&!control.GripperRearmed(),"loss preserves targets and requires rearm");
        control.Control(sample,{0,true},base,{},true);
        check(!control.Current().engaged&&control.Targets()==held&&control.Gripper()==grip,"held trigger cannot bypass loss rearm");
        sample.trigger=0;control.Control(sample,{.2F,true},base,{},true);
        sample.trigger=1;control.Control(sample,{.4F,true},base,{},true);
        check(control.Current().engaged&&control.Gripper()==.4F,"release rearms both channels");
        control.Control(sample,{std::numeric_limits<float>::quiet_NaN(),true},base,{},true);
        check(control.Gripper()==.4F,"invalid gripper must hold");
        const auto prior=control.Targets();control.Invalidate();
        check(!control.Current().calibrated&&!control.Current().engaged&&control.Targets()==prior,"reference invalidation retains targets but retires calibration");
        control.Reset();check(control.Targets()==kinematics::kHome&&control.Gripper()==0,"explicit Reset restores seed");
        check(events.size()==12&&events.front().operation==FullControlOperation::Reset&&events.back().operation==FullControlOperation::Reset,
              "each machine operation emits exactly one event");
        std::cout << "full control calibration, independent grip, loss/rearm and observer passed\n";
    } catch (const std::exception& e) {std::cerr << e.what() << '\n';return 1;}
}
