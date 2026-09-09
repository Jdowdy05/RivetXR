#include "control_boundary.h"
#include "base_registration.h"
#include "registered_input.h"
#include "simulation_clock.h"
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace quest_newton;
using Clock=std::chrono::steady_clock;
void Check(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}
FullInputFrame Frame(Clock::time_point now) {
    FullInputFrame f;f.sampled=now;f.registered=true;f.reference=2;f.suspension=3;
    auto& s=f.controller;s.focused=s.stage_valid=s.pose_active=s.trigger_active=s.calibrate_active=true;
    s.position_valid=s.orientation_valid=s.position_tracked=s.orientation_tracked=true;
    s.stage_from_grip.position={.2F,1.F,-.4F};f.world_from_base.position={0,1,0};return f;
}
}
int main(){
    using namespace quest_newton;
    using namespace std::chrono_literals;
    try {
        const auto now=Clock::now();auto press=Frame(now);
        press.controller.trigger=1;
        Check(!CaptureCalibration(press,7,false,false,now),"held-trigger calibration press must be rejected at capture");
        press.controller.trigger=0;
        Check(!CaptureCalibration(press,7,true,false,now),"manual pause cannot queue calibration");
        Check(!CaptureCalibration(press,7,false,true,now),"room pause cannot queue calibration");
        press.controller.position_tracked=false;
        Check(!CaptureCalibration(press,7,false,false,now),"tracking-loss calibration press cannot survive recovery");
        press=Frame(now);auto event=CaptureCalibration(press,7,false,false,now);
        Check(event.has_value(),"eligible press captured");
        auto current=press;current.sampled=now+10ms;current.controller.stage_from_grip.position[0]+=.3F;
        Check(CalibrationIsCurrent(*event,current,7,false,false,now+10ms),"prompt captured event stays eligible after hand movement");
        RegisteredInputMapper mapper;mapper.RequestCalibration();
        auto sample=event->controller;sample.calibrate_pressed=false;
        mapper.Update(sample,kinematics::Compose(event->stage_from_world,event->world_from_base),{});
        Check(mapper.Current().calibrated,"calibration consumes captured press immediately without control tick");
        sample.trigger=1;mapper.Update(sample,event->world_from_base,{});
        Check(mapper.Current().engaged,"captured pose has no delayed-pose rebase");
        for(std::size_t i=0;i<kinematics::kHome.size();++i)
            Check(std::abs(mapper.Current().joints[i]-kinematics::kHome[i])<1e-4F,"press-time pose keeps neutral target unchanged");
        for(auto* epoch:{&current.reference,&current.suspension}) {
            ++*epoch;Check(!CalibrationIsCurrent(*event,current,7,false,false,now+10ms),"epoch change retires calibration");--*epoch;
        }
        Check(!CalibrationIsCurrent(*event,current,8,false,false,now+10ms),"settings change retires calibration");
        Check(!CalibrationIsCurrent(*event,current,7,false,false,now+101ms),"stale calibration retired before slow control tick");

        // The root stays fixed through a room load and rebases at recovery.
        SimSettings settings;BaseRegistration base;kinematics::Pose head;head.position={0,1.7F,0};
        auto update=[&](bool room_paused){base.Update(head,RegistrationTracking(base.Valid(),true,true,true,false,room_paused),false,settings);};
        update(true);Check(base.Valid(),"initial registration must break room-readiness dependency");
        const auto original=base.WorldFromBase().position[2];update(false);
        head.position[1]=1.3F;update(true);update(true);update(false);
        Check(base.WorldFromBase().position[2]==original,"room recovery cannot apply accrued height movement");
        head.position[1]=1.4F;update(false);
        Check(std::abs(base.WorldFromBase().position[2]-original-.1F)<1e-5F,"new height motion resumes normally");
        // An inactive remapped engage action must freeze/rebase the same way.
        auto action_frame=Frame(now);
        auto action_update=[&](){base.Update(head,RegistrationTracking(base.Valid(),true,true,true,
            ControlsSuppressed(action_frame.controller,false,false),false),false,settings);};
        const auto before_action_loss=base.WorldFromBase().position[2];
        action_frame.controller.trigger_active=false;head.position[1]=.9F;action_update();
        action_frame.controller.trigger_active=true;action_update();
        Check(base.WorldFromBase().position[2]==before_action_loss,"engage-action recovery cannot apply accrued height movement");
        for(const float invalid:{-1.F,2.F,std::numeric_limits<float>::quiet_NaN()}) {
            action_frame.controller.trigger=invalid;
            Check(ControlsSuppressed(action_frame.controller,false,false),"invalid engage values suppress registration and worker together");
        }
        RoomEnvironment desired,applied;applied.enabled=true;applied.revision=4;
        Check(RoomPhysicsBlocked(false,false,true,desired,&applied),"pending Off remains a registration pause");

        // A four-step off-mode batch must yield after new settings or loss,
        // while ordinary unchanged off-mode input permits the existing step.
        auto batch=Frame(now),fresh=batch;
        Check(!SubstepBoundaryChanged(batch,7,fresh,7,false,now),"unchanged boundary permits physics");
        Check(SubstepBoundaryChanged(batch,7,fresh,8,false,now),"off-to-on settings stop remaining substeps");
        ++fresh.suspension;Check(SubstepBoundaryChanged(batch,7,fresh,7,false,now),"known loss stops remaining substeps");
        fresh=batch;++fresh.reference;Check(SubstepBoundaryChanged(batch,7,fresh,7,false,now),"new reference stops remaining substeps");
        fresh=batch;fresh.controller.focused=false;Check(SubstepBoundaryChanged(batch,7,fresh,7,false,now),"fresh focus loss wins over cached valid input");
        fresh=batch;Check(SubstepBoundaryChanged(batch,7,fresh,7,true,now),"queued Pause or Reset stops remaining substeps");
        fresh=batch;++fresh.calibration;Check(SubstepBoundaryChanged(batch,7,fresh,7,false,now),"new calibration yields promptly");
        auto grip_batch=Frame(now),grip_fresh=grip_batch;
        const auto grip_index=static_cast<std::size_t>(SimAction::Gripper);
        grip_batch.actions[grip_index]=grip_fresh.actions[grip_index]={.8F,true};
        Check(GripperInputCurrent(grip_batch,grip_fresh,now),"fresh valid independent grip input is eligible");
        grip_fresh.actions[grip_index].active=false;
        Check(!GripperInputCurrent(grip_batch,grip_fresh,now),"fresh grip-only tracking loss beats cached batch");
        grip_fresh=grip_batch;grip_fresh.actions[grip_index].value=std::numeric_limits<float>::quiet_NaN();
        Check(!GripperInputCurrent(grip_batch,grip_fresh,now),"nonfinite grip cannot advance targets");
        grip_fresh=grip_batch;grip_fresh.menu_open=true;
        Check(!GripperInputCurrent(grip_batch,grip_fresh,now),"menu/inspection/placement suppression freezes applied grip");
        grip_fresh=grip_batch;
        Check(!GripperInputCurrent(grip_batch,grip_fresh,now+101ms),"stale grip cannot continue closing");
        ++grip_fresh.reference;
        Check(!GripperInputCurrent(grip_batch,grip_fresh,now),"reference change retires grip eligibility");
        SimulationClock clock;clock.Configure(settings,0);clock.Reset(0);
        Check(clock.Accumulate(.02,false)==4,"four-step fixture");clock.Advance();
        if(SubstepBoundaryChanged(batch,7,fresh,7,false,now))clock.Accumulate(.021,true);
        Check(clock.SimulationTime()==.005 && clock.Accumulate(.022,false)==0,"barrier flush preserves completed simulation time");
        std::cout<<"press-time calibration, room pause registration and fresh substep barriers passed\n";
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
