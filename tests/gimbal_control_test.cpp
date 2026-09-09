#include "gimbal_control.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
namespace {void Check(bool b,const char*s){if(!b)throw std::runtime_error(s);}}
int main(){try{
    using namespace quest_newton;
    GimbalAim machine;GimbalAimInput i;i.epoch={1,2,3,4,5};i.now=GimbalTime{}+std::chrono::seconds(1);
    i.eligible=true;i.clutch_active=true;i.feedback.pan_min=-1;i.feedback.pan_max=1;i.feedback.tilt_min=-1;i.feedback.tilt_max=1;
    i.orientation.rotation={0,0,0,1};i.clutch=1;
    Check(!machine.Update(i).clutch,"held trigger cannot arm on entry");
    i.clutch=0;machine.Update(i);i.clutch=1;Check(machine.Update(i).clutch,"release then clutch arms");
    i.now+=std::chrono::milliseconds(10);i.orientation.rotation={0,std::sin(.1F),0,std::cos(.1F)};
    Check(std::abs(machine.Update(i).pan-.2F)<1e-5F,"positive head yaw pans left");
    i.eligible=false;Check(!machine.Update(i).clutch,"menu/focus loss holds");
    i.eligible=true;Check(!machine.Update(i).clutch,"loss requires release");
    i.clutch=0;machine.Update(i);i.clutch=1;machine.Update(i);
    i.now+=std::chrono::seconds(1);Check(!machine.Update(i).clutch,"render gap consumes clutch");
    i.clutch=0;machine.Update(i);i.clutch=1;machine.Update(i);++i.epoch[2];Check(!machine.Update(i).clutch,"map reference change disarms");
    GimbalAim pitch;auto j=i;j.epoch={1,2,3,4,5};j.orientation.rotation={0,0,0,1};j.clutch=0;pitch.Update(j);j.clutch=1;pitch.Update(j);
    j.now+=std::chrono::milliseconds(10);j.orientation.rotation={std::sin(.4F),0,0,std::cos(.4F)};
    Check(!pitch.Update(j).clutch,"pitch discontinuity must consume clutch like yaw");
    GimbalAim partial;j.orientation.rotation={0,0,0,1};j.clutch=0;partial.Update(j);j.clutch=1;partial.Update(j);
    j.clutch=.5F;j.now+=std::chrono::milliseconds(10);j.orientation.rotation={0,std::sin(.2F),0,std::cos(.2F)};
    Check(!partial.Update(j).clutch,"partial clutch release must Hold");j.clutch=1;j.now+=std::chrono::milliseconds(10);
    Check(std::abs(partial.Update(j).pan-j.feedback.pan)<1e-5F,"re-clutch after partial release must restart from measured position");
    GimbalFeedback feedback;feedback.session=7;feedback.challenge=8;
    auto bytes=EncodeGimbalCommand(feedback,{},1,2);Check(bytes.size()==64&&std::to_integer<unsigned>(bytes[56])==100,"bounded command wire");
    GimbalFeedback old;old.session=999;std::string error;std::array<std::byte,96> invalid{};
    Check(!DecodeGimbalFeedback(invalid,old,error)&&old.session==999,"invalid feedback preserves output");
    std::cout<<"gimbal clutch, epochs, limits and wire contracts passed\n";
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
