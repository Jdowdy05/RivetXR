#include "remote_inspection.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void Check(bool condition,const char* why){if(!condition)throw std::runtime_error(why);}
void Near(const quest_newton::Vec3& a,const quest_newton::Vec3& b){
    for(std::size_t i=0;i<3;++i)Check(std::abs(a[i]-b[i])<1e-5F,"inspection transform mismatch");
}
}
int main(){
    using namespace quest_newton;
    try {
        Check(RenderTimingScope(false,true)==InspectionTimingScope::Remote,"remote workload is measured without Franka visuals");
        Check(RenderTimingScope(true,false)==InspectionTimingScope::Local,"local timing keeps its qualification gate");
        Check(RenderTimingScope(false,false)==InspectionTimingScope::None,"missing geometry cannot claim workload timing");
        RemoteInspection view;InspectionIdentity key{1,2,3};
        kinematics::Pose observer{{-2,0,1.4F},{.5F,-.5F,-.5F,.5F}};
        kinematics::Pose head{{.5F,1.7F,-.2F},{0,0,0,1}};
        Check(view.Align(key,observer,head,4) && view.Matches(key,4),"valid visual alignment");
        Check(!view.Matches({1,2,3,1},4) && !view.Matches({1,2,3,2},4),"representation changes retire alignment");
        Check(!view.Matches({1,2,3,0,1},4),"retained map reset retires alignment");
        Check(!view.Align({1,2,3,3},observer,head,4),"unknown representation cannot align");
        Near(TransformPoint(view.StageFromMap(),observer.position),head.position);
        Near(TransformPoint(view.StageFromMap(),{-1,0,1.4F}),{.5F,1.7F,-1.2F});
        Near(TransformPoint(view.StageFromMap(),{-2,0,2.4F}),{.5F,2.7F,-.2F});
        const auto original=view.StageFromMap();
        auto walked=head;walked.position[0]+=.3F;
        Check(view.SetScale(.5F,walked),"visual scale changes around current eye position");
        // At this stage location map Y is -.3, so it remains under the same eye.
        Near(TransformPoint(view.StageFromMap(),{-2,-.3F,1.4F}),walked.position);
        Check(view.Step(walked,.5F),"explicit navigation step");
        Near(TransformPoint(view.StageFromMap(),{-1.5F,-.3F,1.4F}),walked.position);
        Check(!view.Matches({1,3,3},4) && !view.Matches(key,5),"source/epoch/reference changes retire alignment");
        Check(!view.SetScale(0,head) && !view.SetScale(std::numeric_limits<float>::quiet_NaN(),head),"invalid view scales rejected");
        view.Invalidate();Check(!view.Step(head,1) && !view.Matches(key,4),"invalidated view cannot navigate");
        Check(!view.Align({0,2,3},observer,head,4),"zero identity cannot align");
        Check(view.Align(key,observer,head,4),"fresh explicit alignment recovers");
        Check(view.Scale()==.5F,"alignment preserves chosen visual scale");
        (void)original;
        std::cout<<"remote visual navigation preserves metric frames and identity boundaries\n";
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
