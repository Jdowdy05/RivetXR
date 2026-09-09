#include "scene_snapshot.h"
#include "registered_input.h"
#include <bit>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void Check(bool v,const char* why){if(!v)throw std::runtime_error(why);}
template<class T> void Put(std::vector<std::byte>& b,std::size_t at,T v){std::memcpy(b.data()+at,&v,sizeof(v));}
quest_newton::ControllerSample Tracked() {
    quest_newton::ControllerSample s;
    s.focused=s.stage_valid=s.pose_active=s.trigger_active=s.calibrate_active=true;
    s.position_valid=s.orientation_valid=s.position_tracked=s.orientation_tracked=true;
    s.stage_from_grip.position={.2F,1.F,-.4F};return s;
}
}
int main(){
    using namespace quest_newton;
    static_assert(std::endian::native==std::endian::little);
    try {
        std::vector<std::byte> data(48+13*28+20);
        std::memcpy(data.data(),"QSIM",4);Put<std::uint16_t>(data,4,1);Put<std::uint16_t>(data,6,48);
        Put<std::uint32_t>(data,8,13);Put<std::uint32_t>(data,12,1);Put<std::uint32_t>(data,16,2);
        Put<std::uint32_t>(data,20,3);Put<std::uint64_t>(data,24,100);Put<double>(data,32,.5);Put<double>(data,40,1.2);
        for(std::size_t i=0;i<13;++i)Put<float>(data,48+i*28+24,1);
        Put<std::uint32_t>(data,48+13*28,12);Put<std::uint32_t>(data,48+13*28+4,1);
        for(std::size_t i=0;i<3;++i)Put<float>(data,48+13*28+8+4*i,.05F);
        SceneSnapshot snapshot;std::string error;
        Check(DecodeSceneSnapshot(data,snapshot,error),"valid snapshot");
        Check(snapshot.bodies.size()==13 && snapshot.objects.size()==1 && snapshot.contact_count==3,"snapshot fields");
        Check(snapshot.step_index==100 && snapshot.simulation_time==.5,"snapshot timing");
        const auto saved=snapshot;
        auto bad=data;bad.pop_back();Check(!DecodeSceneSnapshot(bad,snapshot,error),"reject truncation");
        Check(snapshot.step_index==saved.step_index && snapshot.bodies.size()==saved.bodies.size(),"transactional decode");
        bad=data;Put<std::uint32_t>(bad,8,1000000);Check(!DecodeSceneSnapshot(bad,snapshot,error),"reject excessive count");
        bad=data;Put<float>(bad,48,std::numeric_limits<float>::infinity());Check(!DecodeSceneSnapshot(bad,snapshot,error),"reject nonfinite body");
        bad=data;Put<std::uint32_t>(bad,48+13*28,0);Check(!DecodeSceneSnapshot(bad,snapshot,error),"object cannot overwrite robot body");
        bad=data;Put<float>(bad,48+24,0);Check(!DecodeSceneSnapshot(bad,snapshot,error),"reject invalid rotation");

        RegisteredInputMapper mapper;
        kinematics::Pose base; base.position={0,1,0};
        auto input=Tracked();mapper.Update(input,base,{});
        input.calibrate_pressed=true;mapper.Update(input,base,{});
        input.calibrate_pressed=false;mapper.Update(input,base,{});
        const auto home=mapper.Current().joints;
        input.trigger=1;auto mapped=mapper.Update(input,base,{});
        Check(mapped.calibrated && mapped.engaged,"registered control engages");
        for(std::size_t i=0;i<7;++i)Check(std::abs(mapped.joints[i]-home[i])<1e-4,"registration starts without a target jump");
        input.stage_from_grip.position[0]+=.05F;
        for(int i=0;i<10;++i)mapped=mapper.Update(input,base,{});
        Check(mapped.joints!=home && kinematics::WithinJointLimits(mapped.joints),"controller changes bounded IK target");
        const auto held=mapped.joints;input.trigger=0;mapper.Update(input,base,{});
        input.stage_from_grip.position[0]+=.2F;Check(mapper.Update(input,base,{}).joints==held,"release holds target");
        input.focused=false;mapper.Update(input,base,{});input.focused=true;input.trigger=1;
        Check(!mapper.Update(input,base,{}).engaged,"focus reacquisition requires release");
        input.trigger=0;mapper.Update(input,base,{});input.trigger=1;Check(mapper.Update(input,base,{}).engaged,"released trigger rearms");
        mapper.Reset();Check(!mapper.Current().calibrated && mapper.Current().joints==kinematics::kHome,"reset mapper");
        input=Tracked();input.trigger=1;mapper.RequestCalibration();mapper.Update(input,base,{});
        Check(!mapper.Current().calibrated,"queued calibration must not synthesize trigger release");
        input.trigger=0;mapper.RequestCalibration();mapper.Update(input,base,{});
        Check(mapper.Current().calibrated,"queued calibration event accepted when released");
        mapper.Reset();input=Tracked();input.stage_from_grip.rotation={0,.258819045F,0,.965925826F};
        const kinematics::Pose offset{{0,0,0},{0,0,.258819045F,.965925826F}};
        mapper.RequestCalibration();mapper.Update(input,base,offset);input.trigger=1;
        for(int i=0;i<30;++i)mapper.Update(input,base,offset);
        const auto orientation=mapper.Current().robot_base_from_grip.rotation;
        kinematics::Pose neutral;kinematics::ForwardKinematics(kinematics::kHome,neutral);
        double dot=0;for(std::size_t i=0;i<4;++i)dot+=orientation[i]*neutral.rotation[i];
        Check(std::abs(dot)<.999,"default hand orientation remains meaningful after calibration");
        input.trigger=0;mapper.RequestCalibration();mapper.Update(input,base,offset);input.trigger=1;
        const auto rebased=mapper.Update(input,base,offset).robot_base_from_grip.rotation;
        dot=0;for(std::size_t i=0;i<4;++i)dot+=orientation[i]*rebased[i];
        Check(std::abs(std::abs(dot)-1)<1e-5,"repeated calibration does not accumulate hand offsets");
        std::cout<<"versioned scene snapshots and registered IK controls passed\n";
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
